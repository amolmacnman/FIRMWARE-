/*
 * Gateway -> sub-node BLE delivery - see node_link.h.
 *
 * Sequence, driven entirely by the observer already scanning for uplinks:
 *
 *   1. node opens its connectable window and beacons {company, node_id, type}
 *   2. observer's scan callback fires window_seen() (BLE RX context)
 *   3. if that node has a pending command, a work item is submitted - we must
 *      NOT connect from the scan callback itself
 *   4. the work item pauses scanning, connects, discovers the downlink
 *      characteristic, writes one sealed frame, disconnects, resumes scanning
 *
 * The frame is sealed with sw_secure_seal_dn(), so the node applies it only if
 * it authenticates under the per-wagon key, is addressed to that node, and
 * carries a strictly higher downlink counter. The gateway keeps that counter
 * per node and persists it, so a gateway reboot cannot emit a counter the node
 * has already seen (which the node would reject as a replay, silently wedging
 * that node's config channel forever).
 */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>       /* bt_hci_get_conn_handle, cmd_alloc */
#include <zephyr/bluetooth/hci_vs.h>    /* VS Write_Tx_Power_Level           */
#include <zephyr/sys/byteorder.h>
#include "config.h"                     /* cfg_ble_tx_dbm()                  */
#include <zephyr/settings/settings.h>
#include <string.h>

#include "node_link.h"
#include "config.h"
#include "ble_sensors.h"
#include "sensor_proto.h"
#include "sw_secure.h"
#include "sw_ids.h"
#include "app_config.h"
#include "nodeota.h"    /* staged sub-node image + campaign progress */

#define DN_CONNECT_TIMEOUT_MS  8000

/*
 * Connection parameters for the config write. One 20-byte GATT write, so
 * throughput is irrelevant - a relaxed 30-50 ms interval keeps the node's radio
 * duty low. Latency 0 so the node cannot skip the events we need.
 */
#define DN_CONN_INT_MIN        24     /* x1.25 ms = 30 ms */
#define DN_CONN_INT_MAX        40     /* x1.25 ms = 50 ms */
#define DN_CONN_TIMEOUT        400    /* x10   ms =  4  s */

struct pending {
	bool     active;
	uint8_t  node_type;
	uint8_t  cmd;        /* SW_DN_* - which threshold this frame sets */
	uint8_t  sel;        /* frame rsvd byte; SW_DN_SET_BATTCAL selector */
	int16_t  threshold;
	int16_t  hyst;
};

static struct pending  s_pend[SW_MAX_NODES];
static uint32_t        s_dn_ctr[SW_MAX_NODES];   /* per-node downlink counter */
static uint16_t        s_grp;

/* Target of the in-flight connection attempt. */
static bt_addr_le_t    s_target_addr;
static uint8_t         s_target_id;
static atomic_t        s_busy;

static struct bt_conn *s_conn;
static uint16_t        s_chr_handle;
static struct k_sem    s_done;

static const struct bt_uuid_128 dn_svc_uuid = BT_UUID_INIT_128(SW_DN_SVC_UUID_VAL);
static const struct bt_uuid_128 dn_chr_uuid = BT_UUID_INIT_128(SW_DN_CHR_UUID_VAL);
static const struct bt_uuid_128 img_chr_uuid = BT_UUID_INIT_128(SW_IMG_CHR_UUID_VAL);

static struct bt_gatt_discover_params  s_disc;
static struct bt_gatt_write_params     s_wr;
static struct sw_dn_enc                s_frame;
static uint16_t s_img_handle;

/*
 * Image nonce counter. ONE counter shared by every node rather than one each:
 * each node only checks that the value is strictly greater than the last it
 * accepted, so a single monotonic source satisfies all of them, and it means
 * one persisted word instead of SW_MAX_NODES.
 *
 * It MUST be persisted. A gateway reset that rewound this would emit a counter
 * a node had already seen, the node would reject every chunk as a replay, and
 * that node could never be updated again until IT rebooted.
 */
static uint32_t s_img_seq;

/* ---- persisted downlink counters ---------------------------------------- */

static int nl_settings_set(const char *name, size_t len,
			   settings_read_cb read_cb, void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "ctr", &next) && !next) {
		if (len != sizeof(s_dn_ctr)) {
			return -EINVAL;
		}
		return read_cb(cb_arg, s_dn_ctr, sizeof(s_dn_ctr)) > 0 ? 0 : -EIO;
	}
	if (settings_name_steq(name, "img", &next) && !next) {
		if (len != sizeof(s_img_seq)) {
			return -EINVAL;
		}
		return read_cb(cb_arg, &s_img_seq, sizeof(s_img_seq)) > 0 ? 0 : -EIO;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(swnl, "sw/nl", NULL, nl_settings_set, NULL, NULL);

static void save_ctrs(void)
{
	settings_save_one("sw/nl/ctr", s_dn_ctr, sizeof(s_dn_ctr));
}

/*
 * The image counter moves thousands of times per campaign, so it is checkpointed
 * in BLOCKS rather than per chunk: persist a value well ahead of where we are,
 * and only persist again when we reach it. A reset then resumes from the
 * reserved ceiling, skipping the unused values - the same reserve-ahead trick
 * the sub-nodes use for their advert nonce. Skipping is free; reusing is fatal.
 */
#define IMG_SEQ_BATCH  1024u
static uint32_t s_img_seq_ceil;

static void img_seq_reserve(void)
{
	if (s_img_seq < s_img_seq_ceil) {
		return;
	}
	s_img_seq_ceil = s_img_seq + IMG_SEQ_BATCH;
	settings_save_one("sw/nl/img", &s_img_seq_ceil, sizeof(s_img_seq_ceil));
}

/* ---- GATT: discover -> write -> disconnect ------------------------------ */

/*
 * Delivery phase for the connection in flight.
 *
 * A config write is one frame and finishes in a single callback. An image is
 * thousands of frames spread over many windows, so it needs a state machine:
 * BEGIN on the config characteristic, then chunks on the image characteristic
 * until the node's window closes, then END once the last byte is in.
 *
 * The phase lives here rather than in nodeota.c because it is a property of
 * THIS CONNECTION, not of the campaign - a dropped link rewinds the phase but
 * not the campaign's progress.
 */
enum nl_phase {
	NL_CFG = 0,      /* single threshold/command write */
	NL_IMG_BEGIN,    /* OTA_BEGIN sent, awaiting its ack */
	NL_IMG_CHUNK,    /* streaming image chunks */
	NL_IMG_END,      /* OTA_END sent, awaiting its ack */
};

static enum nl_phase s_phase;
static uint32_t      s_img_off;     /* offset of the chunk in flight */
static uint32_t      s_img_size;
static uint16_t      s_img_crc;
static struct sw_img_enc s_img_frame;
static struct bt_gatt_write_params s_img_wr;

static void img_next_chunk(struct bt_conn *conn);
static void send_dn_cmd(struct bt_conn *conn, uint8_t cmd, uint8_t sel,
			int16_t a, int16_t b);

static void write_cb(struct bt_conn *conn, uint8_t err,
		     struct bt_gatt_write_params *params)
{
	ARG_UNUSED(params);

	if (err) {
		/*
		 * Node rejected it (bad tag, wrong node, replay, out-of-order
		 * chunk) or the link dropped. Nothing is marked delivered, so
		 * the next window retries from the last ACKed byte. For an image
		 * that is the whole recovery story - otarx rejects any offset
		 * that is not exactly what it expects next, so a partial window
		 * can never leave a hole.
		 */
		printk("NL: write to node %u failed (att err 0x%02x, phase %d)\n",
		       s_target_id, err, (int)s_phase);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	switch (s_phase) {
	case NL_CFG:
		printk("NL: node %u command delivered\n", s_target_id);
		s_pend[s_target_id].active = false;
		s_dn_ctr[s_target_id]++;      /* only burn a counter on success */
		save_ctrs();
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		break;

	case NL_IMG_BEGIN:
		s_dn_ctr[s_target_id]++;
		save_ctrs();
		printk("NL: node %u armed for %u B\n", s_target_id, s_img_size);
		s_phase = NL_IMG_CHUNK;
		img_next_chunk(conn);
		break;

	case NL_IMG_CHUNK:
		/*
		 * The node has this chunk. s_img_off was advanced when the write
		 * was issued; only NOW is it committed, so a reset resumes from
		 * what the node actually took rather than what we hoped it took.
		 */
		nodeota_advance(s_img_off);
		if (s_img_off >= s_img_size) {
			s_phase = NL_IMG_END;
			send_dn_cmd(conn, SW_DN_OTA_END, 0,
				    (int16_t)s_img_crc, 0);
		} else {
			img_next_chunk(conn);
		}
		break;

	case NL_IMG_END:
		printk("NL: node %u image accepted - swaps on its next reset\n",
		       s_target_id);
		s_dn_ctr[s_target_id]++;
		save_ctrs();
		nodeota_finish(true);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		break;
	}
}

/* Seal and write one sw_dn_pt command on the CONFIG characteristic. */
static void send_dn_cmd(struct bt_conn *conn, uint8_t cmd, uint8_t sel,
			int16_t a, int16_t b)
{
	struct sw_dn_pt pt = {
		.cmd = cmd, .rsvd = sel, .threshold = a, .hyst = b,
	};

	if (sw_secure_seal_dn(s_grp, s_pend[s_target_id].node_type, s_target_id,
			      s_dn_ctr[s_target_id] + 1, &pt, &s_frame) != 0) {
		printk("NL: seal failed for node %u\n", s_target_id);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	s_wr.func   = write_cb;
	s_wr.handle = s_chr_handle;
	s_wr.offset = 0;
	s_wr.data   = &s_frame;
	s_wr.length = sizeof(s_frame);
	if (bt_gatt_write(conn, &s_wr) != 0) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

/* Read the next slice out of NOR, seal it, and write it to the node. */
static void img_next_chunk(struct bt_conn *conn)
{
	struct sw_img_pt pt = { 0 };
	uint32_t remain = s_img_size - s_img_off;
	uint16_t len    = (remain > SW_IMG_CHUNK_MAX) ?
			  SW_IMG_CHUNK_MAX : (uint16_t)remain;

	if (nodeota_read(s_img_off, pt.data, len) != 0) {
		printk("NL: staging read failed at %u\n", s_img_off);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	pt.offset = s_img_off;
	pt.len    = len;

	/*
	 * Only the used bytes are sealed - a short final chunk is not padded.
	 * pt_len is authenticated through the AAD, so the frame cannot be
	 * truncated to shorten the image without failing the tag.
	 */
	uint16_t pt_len = (uint16_t)(sizeof(pt.offset) + sizeof(pt.len) + len);

	s_img_seq++;
	img_seq_reserve();
	if (sw_secure_seal_img(s_grp, s_pend[s_target_id].node_type, s_target_id,
			       s_img_seq, &pt, pt_len, &s_img_frame) != 0) {
		printk("NL: image seal failed\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	/* Advance our idea of progress only after the ACK (see write_cb). */
	s_img_off += len;

	s_img_wr.func   = write_cb;
	s_img_wr.handle = s_img_handle;
	s_img_wr.offset = 0;
	s_img_wr.data   = &s_img_frame;
	s_img_wr.length = sizeof(s_img_frame);
	if (bt_gatt_write(conn, &s_img_wr) != 0) {
		/* Window closed mid-stream: normal, not an error. Roll back the
		 * unacked chunk so the next window resends exactly it. */
		s_img_off -= len;
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static uint8_t discover_cb(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("NL: node %u has no downlink characteristic\n", s_target_id);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return BT_GATT_ITER_STOP;
	}

	if (params->type != BT_GATT_DISCOVER_CHARACTERISTIC) {
		return BT_GATT_ITER_CONTINUE;
	}

	if (s_phase == NL_CFG && !nodeota_pending(s_target_id)) {
		/* Plain config write: one frame, done. */
		s_chr_handle = bt_gatt_attr_value_handle(attr);
		send_dn_cmd(conn, s_pend[s_target_id].cmd,
			    s_pend[s_target_id].sel,
			    s_pend[s_target_id].threshold,
			    s_pend[s_target_id].hyst);
		return BT_GATT_ITER_STOP;
	}

	/*
	 * Image delivery needs BOTH handles - OTA_BEGIN and OTA_END travel on
	 * the config characteristic, the chunks on the image one. Discovery
	 * therefore runs twice: config first (we are here), then image.
	 */
	if (s_chr_handle == 0) {
		s_chr_handle = bt_gatt_attr_value_handle(attr);

		s_disc.uuid         = &img_chr_uuid.uuid;
		s_disc.func         = discover_cb;
		s_disc.start_handle = 0x0001;
		s_disc.end_handle   = 0xffff;
		s_disc.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
		if (bt_gatt_discover(conn, &s_disc) != 0) {
			bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		return BT_GATT_ITER_STOP;
	}

	s_img_handle = bt_gatt_attr_value_handle(attr);

	uint32_t sent = 0;

	if (nodeota_info(s_target_id, &s_img_size, &s_img_crc, &sent) != 0) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return BT_GATT_ITER_STOP;
	}
	s_img_off = sent;

	if (sent == 0) {
		/*
		 * Fresh campaign: arm the node. OTA_BEGIN erases its secondary
		 * slot, so it is sent exactly once - a resumed transfer must NOT
		 * repeat it or every window would wipe the progress before it.
		 * Size is split lo/hi across the two int16 fields.
		 */
		s_phase = NL_IMG_BEGIN;
		send_dn_cmd(conn, SW_DN_OTA_BEGIN, 0,
			    (int16_t)(s_img_size & 0xFFFF),
			    (int16_t)(s_img_size >> 16));
	} else {
		printk("NL: node %u resuming image at %u/%u B\n",
		       s_target_id, sent, s_img_size);
		s_phase = NL_IMG_CHUNK;
		img_next_chunk(conn);
	}
	return BT_GATT_ITER_STOP;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (conn != s_conn) {
		return;              /* not our central link */
	}
	if (err) {
		printk("NL: connect to node %u failed (0x%02x)\n", s_target_id, err);
		k_sem_give(&s_done);
		return;
	}

	/*
	 * Link is up, so the scanner is free again. Restart the observer NOW
	 * rather than after the transfer: the controller only has to stop
	 * scanning to INITIATE a connection, not to maintain one. Without this
	 * the gateway would be deaf to every other node's alarm for the whole
	 * firmware push - the one moment it most needs to hear an impact or a
	 * door opening on a moving wagon.
	 */
	ble_sensors_scan_resume();

	/*
	 * Apply the configured output power to THIS connection.
	 *
	 * The gateway never advertises - it scans and initiates - so a connection
	 * is the only time it transmits at all. That makes the connection handle
	 * the right target and this the right moment: the level is per-connection,
	 * so one set at boot would not survive to the next node link.
	 *
	 * Failure is deliberately silent. The link is already up and the frames
	 * will go out at the controller default, which is a working link at a
	 * slightly different power - not a reason to abort a downlink.
	 */
	{
		uint16_t h;

		if (bt_hci_get_conn_handle(conn, &h) == 0) {
			struct bt_hci_cp_vs_write_tx_power_level *cp;
			struct net_buf *b = bt_hci_cmd_alloc(K_NO_WAIT);

			if (b) {
				cp = net_buf_add(b, sizeof(*cp));
				cp->handle_type    = BT_HCI_VS_LL_HANDLE_TYPE_CONN;
				cp->handle         = sys_cpu_to_le16(h);
				cp->tx_power_level = cfg_ble_tx_dbm();
				(void)bt_hci_cmd_send_sync(
					BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, b, NULL);
			}
		}
	}

	/* Fresh connection: forget the handles and phase from the last one. */
	s_chr_handle = 0;
	s_img_handle = 0;
	s_phase      = NL_CFG;

	s_disc.uuid         = &dn_chr_uuid.uuid;
	s_disc.func         = discover_cb;
	s_disc.start_handle = 0x0001;
	s_disc.end_handle   = 0xffff;
	s_disc.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

	if (bt_gatt_discover(conn, &s_disc) != 0) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (conn != s_conn) {
		return;
	}
	ARG_UNUSED(reason);
	k_sem_give(&s_done);
}

BT_CONN_CB_DEFINE(node_link_conn_cb) = {
	.connected    = connected_cb,
	.disconnected = disconnected_cb,
};

/* ---- work item: the whole connect sequence ------------------------------ */

static void deliver_fn(struct k_work *w)
{
	ARG_UNUSED(w);

	/* The controller cannot scan and initiate at the same time. */
	ble_sensors_scan_pause();

	k_sem_reset(&s_done);

	static const struct bt_le_conn_param fast = BT_LE_CONN_PARAM_INIT(
		DN_CONN_INT_MIN, DN_CONN_INT_MAX, 0, DN_CONN_TIMEOUT);

	int rc = bt_conn_le_create(&s_target_addr, BT_CONN_LE_CREATE_CONN,
				   &fast, &s_conn);
	if (rc != 0) {
		printk("NL: conn_le_create failed (%d)\n", rc);
	} else {
		/* Bounded: the node's window is only ~4 s, so never block the
		 * gateway's own reporting cycle waiting for a node that left. */
		if (k_sem_take(&s_done, K_MSEC(DN_CONNECT_TIMEOUT_MS)) != 0) {
			printk("NL: node %u timed out\n", s_target_id);
			bt_conn_disconnect(s_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			k_sem_take(&s_done, K_MSEC(1000));
		}
		bt_conn_unref(s_conn);
		s_conn = NULL;
	}

	ble_sensors_scan_resume();
	atomic_set(&s_busy, 0);
}

K_WORK_DEFINE(deliver_work, deliver_fn);

/* ---- observer hook ------------------------------------------------------ */

/* BLE RX context - do the minimum and defer. */
static void window_seen(uint8_t node_id, uint8_t node_type,
			const bt_addr_le_t *addr)
{
	if (node_id >= SW_MAX_NODES) {
		return;
	}
	/* Connect if there is a config command queued OR a staged image to push. */
	if (!s_pend[node_id].active && !nodeota_pending(node_id)) {
		return;
	}
	if (!atomic_cas(&s_busy, 0, 1)) {
		return;              /* a delivery is already in flight */
	}
	s_target_id   = node_id;
	s_target_addr = *addr;
	s_pend[node_id].node_type = node_type;
	k_work_submit(&deliver_work);
}

/* ---- public API --------------------------------------------------------- */

int node_link_init(void)
{
	k_sem_init(&s_done, 0, 1);
	if (settings_subsys_init() == 0) {
		settings_load();
	}
	s_grp = sw_wgn_group(cfg_wagon());
	/* Resume at the reserved ceiling so a reset can never re-issue a counter
	 * a node has already accepted. */
	s_img_seq_ceil = s_img_seq;
	ble_sensors_register_window_cb(window_seen);
	printk("NL: node downlink ready\n");
	return 0;
}

int node_link_queue_cmd(uint8_t node_id, uint8_t node_type, uint8_t cmd,
			int16_t threshold, int16_t hyst)
{
	/* rsvd has always gone out as 0 for every command except the gauge
	 * calibration one, so the plain form is this with sel = 0. */
	return node_link_queue_cmd_sel(node_id, node_type, cmd, 0,
				       threshold, hyst);
}

int node_link_queue_cmd_sel(uint8_t node_id, uint8_t node_type, uint8_t cmd,
			    uint8_t sel, int16_t threshold, int16_t hyst)
{
	if (node_id >= SW_MAX_NODES) {
		return -EINVAL;
	}
	s_pend[node_id].cmd       = cmd;
	s_pend[node_id].sel       = sel;
	s_pend[node_id].node_type = node_type;
	s_pend[node_id].threshold = threshold;
	s_pend[node_id].hyst      = hyst;
	s_pend[node_id].active    = true;
	printk("NL: node %u threshold=%d hyst=%d queued\n",
	       node_id, threshold, hyst);
	return 0;
}

bool node_link_pending(uint8_t node_id)
{
	return node_id < SW_MAX_NODES && s_pend[node_id].active;
}

int node_link_pending_count(void)
{
	int n = 0;

	for (int i = 0; i < SW_MAX_NODES; i++) {
		if (s_pend[i].active) {
			n++;
		}
	}
	return n;
}
