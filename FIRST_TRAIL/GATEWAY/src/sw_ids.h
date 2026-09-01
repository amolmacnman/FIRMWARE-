#ifndef SW_IDS_H
#define SW_IDS_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/*
 * Deterministic identity from the wagon number (single source of truth).
 * The gateway and EVERY sub-node on a wagon derive the SAME wgn_group from the
 * SAME wagon number, so provisioning is one value per wagon (WAGON_NUMBER).
 *
 * NOTE: IDs are NOT secret - the wagon number is public. Confidentiality is
 * provided separately by sw_secure (AES-CCM), whose key is NOT derivable from
 * the wagon number. Keep this file identical in the gateway and sub-node.
 */

/* CRC16-CCITT (poly 0x1021, init 0xFFFF). */
static inline uint16_t sw_crc16(const uint8_t *d, size_t n)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)d[i] << 8;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
					     : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* This wagon's BLE isolation group id, from the wagon number. */
static inline uint16_t sw_wgn_group(const char *wagon_no)
{
	return sw_crc16((const uint8_t *)wagon_no, strlen(wagon_no));
}

/* Global sub-node serial "wagon-type-id" for the cloud/back-office. The 1-byte
 * node_id stays on air; this is the human-readable identity. buf >= 24. */
static inline void sw_node_serial(char *buf, size_t n, const char *wagon_no,
				  uint8_t node_type, uint8_t node_id)
{
	snprintf(buf, n, "%s-%u-%u", wagon_no, node_type, node_id);
}

#endif /* SW_IDS_H */
