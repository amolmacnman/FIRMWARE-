/*
 * OTA firmware update (MQTT signals, HTTP transfers).
 *
 * Flow: the server sends `ota_start{url,ver,size}` over MQTT (dn/cmd). This
 * downloads the MCUboot-signed image over HTTP(S) via the EC200 and streams it
 * straight into the MCUboot SECONDARY slot using Zephyr's dfu_target - the
 * image never sits in RAM. On success it requests the swap; ota_tick() reboots
 * at a safe point (after the resp is published). MCUboot verifies the signature
 * at boot and swaps; if the new image never confirms itself (see
 * boot_write_img_confirmed() in main.c after a healthy cycle), MCUboot REVERTS
 * on the next reset - so a bad update cannot brick the wagon.
 *
 * NOTE: this is real dfu_target/HTTP code but CANNOT be tested here without the
 * modem + MCUboot build. Validate on hardware; see BUILD notes.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <dfu/dfu_target.h>            /* NCS (nrf) header, not zephyr/ */
#include <dfu/dfu_target_mcuboot.h>    /* NCS (nrf) header, not zephyr/ */
#include <zephyr/dfu/mcuboot.h>        /* this one IS a zephyr header  */
#include <stdio.h>
#include <string.h>
#include "ota.h"
#include "app_config.h"      /* FW_VERSION for the already-at-version guard */
#include "ec200.h"

static enum ota_state st = OTA_IDLE;
static int  pct;
static char cur_ver[24];
static int  img_size;
static int  written;

/* dfu_target needs a small scratch buffer for the MCUboot backend. */
static uint8_t dfu_buf[512];

static const char *state_str(enum ota_state s)
{
	switch (s) {
	case OTA_IDLE:        return "idle";
	case OTA_DOWNLOADING: return "downloading";
	case OTA_VERIFY:      return "verify";
	case OTA_READY:       return "ready";
	case OTA_APPLYING:    return "applying";
	default:              return "error";
	}
}

/* Called for each downloaded chunk: write it into the secondary slot. */
static int ota_sink(void *ctx, const uint8_t *data, int len)
{
	ARG_UNUSED(ctx);
	int rc = dfu_target_write(data, (size_t)len);
	if (rc != 0) {
		printk("OTA: dfu_target_write failed (%d)\n", rc);
		return rc;
	}
	written += len;
	if (img_size > 0) {
		pct = (int)((int64_t)written * 100 / img_size);
	}
	return 0;
}

int ota_start(const char *url, const char *ver, int size_bytes)
{
	if (st == OTA_DOWNLOADING || st == OTA_APPLYING) {
		return -EBUSY;
	}
	if (!url || !ver || size_bytes <= 0) {
		return -EINVAL;
	}

	/*
	 * ALREADY-AT-THIS-VERSION GUARD.
	 *
	 * This is what makes a GROUP or BULK ota_start safe to repeat. A fleet
	 * roll-out is not atomic: some wagons are asleep, some are out of
	 * coverage, so the operator re-publishes the same command over days
	 * until every wagon has answered. Without this check, the wagons that
	 * already updated would re-download and re-flash the identical image on
	 * every repeat - burning cellular data, flash write cycles and battery
	 * across the whole fleet.
	 *
	 * It also contains the worst-case accident: a broadcast ota_start
	 * published with RETAIN set would otherwise re-flash every gateway on
	 * every reconnect, forever. With this guard the second pass is a no-op.
	 */
	if (strcmp(ver, FW_VERSION) == 0) {
		printk("OTA: already running %s - ignoring\n", ver);
		return -EALREADY;
	}
	strncpy(cur_ver, ver, sizeof(cur_ver) - 1);
	cur_ver[sizeof(cur_ver) - 1] = '\0';
	img_size = size_bytes;
	written  = 0;
	pct      = 0;
	st       = OTA_DOWNLOADING;
	printk("OTA: start ver=%s size=%d\n", cur_ver, img_size);

	/* Prepare the MCUboot secondary slot for a streamed write. */
	int rc = dfu_target_mcuboot_set_buf(dfu_buf, sizeof(dfu_buf));
	if (rc == 0) {
		rc = dfu_target_init(DFU_TARGET_IMAGE_TYPE_MCUBOOT, 0,
				     (size_t)img_size, NULL);
	}
	if (rc != 0) {
		printk("OTA: dfu_target init failed (%d)\n", rc);
		st = OTA_ERROR;
		return rc;
	}

	/* Download over HTTP, streaming each chunk into slot-1 (modem is online). */
	int total = ec200_http_download(url, ota_sink, NULL);
	if (total < 0 || written == 0) {
		printk("OTA: download failed (%d, wrote %d)\n", total, written);
		(void)dfu_target_done(false);
		st = OTA_ERROR;
		return -EIO;
	}

	/* Finalise the write. MCUboot verifies the signature at swap/boot time. */
	st = OTA_VERIFY;
	rc = dfu_target_done(true);
	if (rc != 0) {
		printk("OTA: dfu_target_done failed (%d)\n", rc);
		st = OTA_ERROR;
		return rc;
	}

	/* Mark slot-1 to be swapped in on the next boot (TEST = revert if the new
	 * image doesn't confirm itself). */
	rc = boot_request_upgrade(BOOT_UPGRADE_TEST);
	if (rc != 0) {
		printk("OTA: boot_request_upgrade failed (%d)\n", rc);
		st = OTA_ERROR;
		return rc;
	}
	pct = 100;
	st  = OTA_READY;
	printk("OTA: image staged - will reboot to apply\n");
	return 0;
}

void ota_tick(void)
{
	if (st == OTA_READY) {
		/* Reboot at a safe point (the ota_start resp has been published). */
		st = OTA_APPLYING;
		printk("OTA: rebooting into MCUboot to apply update\n");
		k_msleep(300);
		sys_reboot(SYS_REBOOT_COLD);
	}
}

void ota_status_json(char *buf, size_t n)
{
	snprintf(buf, n, "{\"state\":\"%s\",\"pct\":%d,\"ver\":\"%s\"}",
		 state_str(st), pct, cur_ver);
}
