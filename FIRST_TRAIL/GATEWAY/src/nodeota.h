#ifndef NODEOTA_H
#define NODEOTA_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Sub-node OTA campaign manager (gateway side).
 *
 * The node half of this already existed - otarx.c writes chunks into MCUboot's
 * secondary slot and the sealed image characteristic authenticates every frame.
 * What was missing was the sender: sw_secure_seal_img() was defined and never
 * called, so all 19 nodes were sitting ready to receive an image nothing could
 * transmit. This module is that sender.
 *
 * WHY STAGE THE IMAGE INSTEAD OF STREAMING IT
 * The two links run on completely different clocks. The modem is up for a few
 * seconds per report; a node is reachable only during its own connectable
 * window. They are never up together by design, so an image cannot be piped
 * from HTTP to BLE. It is downloaded once into the NOR "ota" partition and then
 * fed to the node across as many windows as it takes.
 *
 * WHY IT SURVIVES A RESET
 * A 512 KB image at 192 B per chunk is ~2700 writes. Even in a fast window that
 * is minutes of radio, and a gateway watchdog reset or a scheduled OTA of the
 * gateway itself must not restart it from zero on a battery-powered node. The
 * campaign - target, size, CRC and how far we got - lives in FRAM, so delivery
 * resumes at the next chunk boundary.
 *
 * SECURITY IS NOT THIS MODULE'S JOB. It only moves bytes. The node enforces
 * AES-CCM per chunk, in-order offsets, its own replay counter, a CRC16 over the
 * whole image, and finally MCUboot signature verification with auto-revert.
 * A corrupt or hostile stage here cannot produce a running image on a node.
 */

enum nodeota_state {
	NODEOTA_IDLE = 0,
	NODEOTA_FETCHING,     /* pulling the image into NOR over HTTP      */
	NODEOTA_READY,        /* staged and verified, waiting for a window */
	NODEOTA_SENDING,      /* chunks in flight to the node              */
	NODEOTA_DONE,
	NODEOTA_ERROR,
};

/*
 * Download `url` into the staging partition and arm a campaign for every node
 * id set in the `targets` bitmask. The image is fetched ONCE however many nodes
 * want it, then delivered to them one at a time as each opens a window.
 * Blocks for the duration of the download (the modem is already up - this is
 * called from the command handler mid-report).
 *
 * Returns 0 if the image is staged and armed, <0 otherwise. Does NOT talk to
 * the node: delivery happens later, from node_link, when that node next opens
 * a window.
 */
/*
 * Start a campaign. `targets` is a BITMASK of node ids, so one staged image can
 * serve every node of a type - a single bit is an individual update and needs
 * no separate path.
 *
 * The image is fetched over cellular ONCE regardless of how many nodes are
 * targeted; only the BLE leg repeats. That asymmetry is the whole point: the
 * modem leg is metered and slow, the radio leg is neither.
 */
int nodeota_start(const char *url, const char *ver, int size_bytes,
		  uint32_t targets, uint8_t node_type);

/* Is there a staged image waiting for this node? */
bool nodeota_pending(uint8_t node_id);

/*
 * Campaign parameters for the node currently being served. `crc` is over the
 * whole image and is what the node checks at OTA_END.
 */
int nodeota_info(uint8_t node_id, uint32_t *size, uint16_t *crc,
		 uint32_t *sent);

/* Read `len` staged bytes at `offset` into `buf`. */
int nodeota_read(uint32_t offset, uint8_t *buf, uint16_t len);

/*
 * Record progress. Called after a chunk is ACKed by the node, so a reset
 * resumes from the last byte the NODE actually took, never from what we
 * hoped it took.
 */
void nodeota_advance(uint32_t sent);

/* Campaign finished (node accepted the image) or failed - clears the slot. */
void nodeota_finish(bool ok);

/* Human-readable state for the ota_status response. */
void nodeota_status_json(char *buf, unsigned int n);

#endif /* NODEOTA_H */
