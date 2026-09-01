#ifndef OTA_H
#define OTA_H
#include <stddef.h>

/*
 * OTA firmware update (protocol cmd ota_start / ota_status).
 *
 * Real flow on nRF54L15 (MCUboot dual-slot):
 *   1. ota_start{url,ver,size} -> download image over the EC200 (AT+QHTTPGET
 *      or QFTP into a modem file, then read it out in chunks).
 *   2. Write chunks into MCUboot slot-1 via Zephyr dfu_target_* / flash_img.
 *   3. Verify hash/signature.
 *   4. boot_request_upgrade(false) + sys_reboot() -> MCUboot swaps + runs new
 *      image; confirm with boot_write_img_confirmed() after a healthy boot.
 *
 * IMPLEMENTED (ota.c): HTTP download via the EC200 streamed into the MCUboot
 * secondary slot with dfu_target, then boot_request_upgrade + reboot. MCUboot
 * verifies the signature at swap and auto-reverts if the new image never
 * confirms itself. Needs bench validation with the modem + MCUboot build.
 * REMAINING: the ota_start trigger arrives over MQTT dn/cmd, whose RECEIVE path
 * (ec200_mqtt_poll_cmd / QMTSUB+QMTRECV) is still to be wired.
 */

enum ota_state { OTA_IDLE, OTA_DOWNLOADING, OTA_VERIFY, OTA_READY,
		 OTA_APPLYING, OTA_ERROR };

/* Begin an update. Returns 0 if accepted (async), negative on bad args/busy. */
int  ota_start(const char *url, const char *ver, int size_bytes);

/* Fill buf with a status JSON object: {"state":"...","pct":N,"ver":"..."} */
void ota_status_json(char *buf, size_t n);

/* Call periodically (e.g. each wake) to advance the OTA state machine. */
void ota_tick(void);

#endif /* OTA_H */
