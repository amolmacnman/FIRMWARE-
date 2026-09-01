/*
 * Sub-node OTA campaign manager - see nodeota.h.
 */
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/sys/crc.h>
#include <stdio.h>
#include <string.h>

#include "nodeota.h"
#include "ec200.h"
#include "sensor_proto.h"
#include "ble_sensors.h"   /* SW_MAX_NODES */

#define STAGE_WB     512u          /* flash write staging buffer */

/*
 * FRAM map (see storage.c and config.c for the neighbours):
 *   0   ring-buffer meta   (16 B)
 *   64  struct app_cfg     (24 B)
 *   128 this campaign      (below)
 */
#define CAMP_FRAM_OFF 128
#define CAMP_MAGIC    0x4E4F5442u  /* 'NOTB' - layout changed for
                                    * multi-target campaigns; the
                                    * old record must be rejected,
                                    * not read with shifted fields */

/*
 * ONE image, ONE target node - forced by how a sub-node is identified, not a
 * simplification to lift later.
 *
 * NODE_ID is COMPILE-TIME on a node: baked into the binary, sealed into every
 * advert, checked on every downlink frame. So the four door nodes on a wagon do
 * NOT run the same image, and pushing one binary to a whole type would leave
 * several nodes all claiming one id - the gateway could no longer tell which
 * door it was hearing from.
 *
 * Updating "door 1 on every wagon" is therefore a FLEET action, not a per-wagon
 * one: publish ota_start with "node":N once to smartwagon/v1/all/dn/cmd and
 * every gateway updates ITS node N. Same node id across wagons IS the same
 * binary; different node ids on one wagon are not.
 */
struct campaign {
	uint32_t magic;
	uint8_t  state;
	uint8_t  node_id;    /* node being streamed RIGHT NOW               */
	uint8_t  node_type;
	uint8_t  rsvd;
	uint32_t targets;    /* ids still to deliver to (bit per node id)   */
	uint32_t done;       /* ids that accepted the image                 */
	uint32_t failed;     /* ids that were tried and refused it          */
	uint32_t size;
	uint32_t sent;
	uint16_t crc;
	uint16_t rsvd2;
	char     ver[24];
};


static struct campaign c;
static const struct device *const fram = DEVICE_DT_GET(DT_NODELABEL(fram));
static const struct flash_area *s_fa;

/* Download-time state: the HTTP sink is called chunk by chunk. */
static uint32_t s_wr_off;
static uint8_t  s_wr_buf[STAGE_WB];
static uint32_t s_wr_len;
static uint16_t s_crc;

static void camp_save(void)
{
	if (device_is_ready(fram)) {
		(void)eeprom_write(fram, CAMP_FRAM_OFF, &c, sizeof(c));
	}
}

static void camp_load(void)
{
	if (!device_is_ready(fram) ||
	    eeprom_read(fram, CAMP_FRAM_OFF, &c, sizeof(c)) != 0 ||
	    c.magic != CAMP_MAGIC) {
		memset(&c, 0, sizeof(c));
		c.magic = CAMP_MAGIC;
		c.state = NODEOTA_IDLE;
	}
}

static int stage_open(void)
{
	if (s_fa) {
		return 0;
	}
	return flash_area_open(FIXED_PARTITION_ID(ota_partition), &s_fa);
}

/* ---- download path ------------------------------------------------------ */

static int stage_flush(void)
{
	if (s_wr_len == 0) {
		return 0;
	}

	uint32_t wb  = flash_get_write_block_size(flash_area_get_device(s_fa));
	uint32_t len = ROUND_UP(s_wr_len, wb);

	if (len > STAGE_WB) {
		return -EINVAL;
	}
	/* Pad with 0xFF: beyond the image, and 0xFF is the erased value so the
	 * write is a no-op for those cells. */
	memset(s_wr_buf + s_wr_len, 0xFF, len - s_wr_len);

	int rc = flash_area_write(s_fa, s_wr_off, s_wr_buf, len);

	s_wr_off += s_wr_len;
	s_wr_len  = 0;
	return rc;
}

static int stage_sink(void *ctx, const uint8_t *data, int len)
{
	ARG_UNUSED(ctx);

	/* CRC over the image as it arrives - this is the value the NODE will
	 * check at OTA_END, so it must cover exactly the bytes we stage. */
	s_crc = crc16_itu_t(s_crc, data, (size_t)len);

	while (len > 0) {
		uint32_t take = MIN(STAGE_WB - s_wr_len, (uint32_t)len);

		memcpy(s_wr_buf + s_wr_len, data, take);
		s_wr_len += take;
		data     += take;
		len      -= (int)take;

		if (s_wr_len == STAGE_WB) {
			int rc = flash_area_write(s_fa, s_wr_off, s_wr_buf,
						  STAGE_WB);
			s_wr_off += STAGE_WB;
			s_wr_len  = 0;
			if (rc != 0) {
				printk("nodeota: stage write failed (%d)\n", rc);
				return rc;
			}
		}
	}
	return 0;
}

/*
 * Move to the next target, or finish the campaign.
 *
 * Each node starts from sent = 0: they are separate BLE transfers of the same
 * staged bytes, and a node that joins late must not inherit another's progress.
 */
static void camp_next(void)
{
	for (uint8_t id = 0; id < SW_MAX_NODES; id++) {
		if (c.targets & ((uint32_t)1u << id)) {
			c.node_id = id;
			c.sent    = 0;
			c.state   = NODEOTA_READY;
			printk("nodeota: next target node %u (%u left)\n",
			       id, __builtin_popcount(c.targets));
			camp_save();
			return;
		}
	}
	c.state = (c.failed == 0) ? NODEOTA_DONE : NODEOTA_ERROR;
	printk("nodeota: campaign complete - %u accepted, %u failed\n",
	       __builtin_popcount(c.done), __builtin_popcount(c.failed));
	camp_save();
}

int nodeota_start(const char *url, const char *ver, int size_bytes,
		  uint32_t targets, uint8_t node_type)
{
	if (!url || !ver || size_bytes <= 0 || targets == 0 ||
	    (targets & ~((1u << SW_MAX_NODES) - 1u))) {
		return -EINVAL;
	}
	if (c.state == NODEOTA_FETCHING || c.state == NODEOTA_SENDING) {
		return -EBUSY;          /* one campaign at a time */
	}
	if (stage_open() != 0) {
		printk("nodeota: no staging partition\n");
		return -ENODEV;
	}
	if ((uint32_t)size_bytes > s_fa->fa_size) {
		printk("nodeota: image %d B exceeds %u B staging area\n",
		       size_bytes, (unsigned)s_fa->fa_size);
		return -EFBIG;
	}

	if (flash_area_erase(s_fa, 0, s_fa->fa_size) != 0) {
		printk("nodeota: staging erase failed\n");
		return -EIO;
	}

	s_wr_off = 0;
	s_wr_len = 0;
	s_crc    = 0;

	c.state     = NODEOTA_FETCHING;
	c.targets   = targets;
	c.done      = 0;
	c.failed    = 0;
	c.node_id   = SW_MAX_NODES;      /* none selected until staging ends */
	c.node_type = node_type;
	c.size      = (uint32_t)size_bytes;
	c.sent      = 0;
	strncpy(c.ver, ver, sizeof(c.ver) - 1);
	c.ver[sizeof(c.ver) - 1] = '\0';
	camp_save();

	printk("nodeota: fetching %s (%d B) for %u node(s), type %u\n",
	       ver, size_bytes, __builtin_popcount(targets), node_type);

	int got = ec200_http_download(url, stage_sink, NULL);

	if (got < 0 || stage_flush() != 0) {
		printk("nodeota: download failed (%d)\n", got);
		c.state = NODEOTA_ERROR;
		camp_save();
		return -EIO;
	}

	/*
	 * Insist the staged length matches what the command announced. The node
	 * is told `size` in OTA_BEGIN and refuses anything that overruns it, so
	 * a mismatch here would only be discovered after minutes of radio.
	 */
	uint32_t staged = s_wr_off;

	if (staged != c.size) {
		printk("nodeota: staged %u B but %u B announced - refused\n",
		       staged, c.size);
		c.state = NODEOTA_ERROR;
		camp_save();
		return -EINVAL;
	}

	c.crc   = s_crc;
	/* Staged once; now walk the target list. */
	camp_next();
	camp_save();
	printk("nodeota: staged %u B crc %04x - waiting for node %u window\n",
	       c.size, c.crc, __builtin_popcount(c.targets));
	return 0;
}

/* ---- delivery accessors (used by node_link) ----------------------------- */

bool nodeota_pending(uint8_t node_id)
{
	return (c.state == NODEOTA_READY || c.state == NODEOTA_SENDING) &&
	       c.node_id == node_id;
}

int nodeota_info(uint8_t node_id, uint32_t *size, uint16_t *crc, uint32_t *sent)
{
	if (!nodeota_pending(node_id)) {
		return -ENOENT;
	}
	if (size) { *size = c.size; }
	if (crc)  { *crc  = c.crc;  }
	if (sent) { *sent = c.sent; }
	return 0;
}

int nodeota_read(uint32_t offset, uint8_t *buf, uint16_t len)
{
	if (stage_open() != 0) {
		return -ENODEV;
	}
	if (offset + len > c.size) {
		return -EINVAL;
	}
	return flash_area_read(s_fa, offset, buf, len);
}

void nodeota_advance(uint32_t sent)
{
	if (sent <= c.sent) {
		return;                 /* never move backwards */
	}
	c.sent  = sent;
	c.state = NODEOTA_SENDING;

	/*
	 * Persist progress, but not on every 192-byte chunk: that would be
	 * ~2700 FRAM writes per image. FRAM has effectively unlimited endurance
	 * so wear is not the issue - the SPI transaction inside the transfer
	 * loop is. Checkpoint every 16 KB; a reset costs at most that much
	 * re-sent, which is seconds of radio, not minutes.
	 */
	if ((c.sent % 16384u) < SW_IMG_CHUNK_MAX || c.sent == c.size) {
		camp_save();
	}
}

void nodeota_finish(bool ok)
{
	uint32_t bit = (c.node_id < SW_MAX_NODES)
		     ? ((uint32_t)1u << c.node_id) : 0u;

	printk("nodeota: node %u %s (%u/%u B)\n",
	       c.node_id, ok ? "accepted" : "FAILED", c.sent, c.size);

	/*
	 * Record the outcome and move on rather than stopping the campaign.
	 *
	 * One node out of eight failing is not a reason to deny the other
	 * seven an update - it is usually that node being asleep or out of
	 * range, and the roll-out should carry on and report which ids were
	 * missed. Re-publishing the same command later then retries only those,
	 * because the ones that took it are already at the new version.
	 */
	c.targets &= ~bit;
	if (ok) {
		c.done   |= bit;
	} else {
		c.failed |= bit;
	}
	camp_next();
}

void nodeota_status_json(char *buf, unsigned int n)
{
	static const char *const names[] = {
		"idle", "fetching", "ready", "sending", "done", "error"
	};
	const char *s = (c.state <= NODEOTA_ERROR) ? names[c.state] : "error";
	int pct = (c.size > 0) ? (int)((uint64_t)c.sent * 100 / c.size) : 0;

	snprintf(buf, n,
		 "{\"state\":\"%s\",\"node\":%u,\"pct\":%d,\"ver\":\"%s\"}",
		 s, c.node_id, pct, c.ver);
}

/* ---- init --------------------------------------------------------------- */

static int nodeota_init(void)
{
	camp_load();
	if (c.state == NODEOTA_FETCHING) {
		/* Reset mid-download: the staged image is incomplete and its CRC
		 * was never computed, so it can only be thrown away. */
		printk("nodeota: interrupted download discarded\n");
		c.state = NODEOTA_IDLE;
		camp_save();
	} else if (c.state == NODEOTA_SENDING) {
		printk("nodeota: resuming node %u at %u/%u B\n",
		       c.node_id, c.sent, c.size);
	}
	return 0;
}
SYS_INIT(nodeota_init, APPLICATION, 90);
