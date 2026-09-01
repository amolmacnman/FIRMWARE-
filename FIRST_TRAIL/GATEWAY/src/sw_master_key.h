#ifndef SW_MASTER_KEY_H
#define SW_MASTER_KEY_H
#include <stdint.h>

/*
 * ============================ PROVISIONING SECRET ============================
 * Fleet master secret. The per-wagon AES-CCM key = HKDF-SHA256(this, wagon_no).
 *
 *  !!!  REPLACE THIS DEMO VALUE BEFORE PRODUCTION  !!!
 *
 *  - Must be IDENTICAL on the gateway AND every sub-node.
 *  - Must be kept OUT of version control in production. Provision it into the
 *    nRF54L15 KMU / secure storage at manufacturing instead of compiling it in;
 *    this file then reads it from there rather than holding the bytes.
 *  - If this leaks, every wagon's BLE traffic is exposed. Treat it as a root key.
 *
 * The demo bytes below are 0x00..0x1f purely so the reference build links.
 * ===========================================================================
 */
static const uint8_t SW_MASTER_SECRET[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
};

#endif /* SW_MASTER_KEY_H */
