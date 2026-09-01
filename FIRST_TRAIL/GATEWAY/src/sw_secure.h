#ifndef SW_SECURE_H
#define SW_SECURE_H
#include <stdint.h>
#include "sensor_proto.h"

/*
 * App-layer authenticated encryption for the BLE sensor advert (AES-CCM).
 * Key = HKDF-SHA256(fleet master secret, wagon number) -> per-wagon, so the
 * gateway and its sub-nodes share it without ever transmitting a key, and a
 * neighbouring wagon (different number) cannot decrypt or forge.
 *
 * Keep this file identical in the gateway and sub-node.
 */

/* Init PSA crypto and derive this wagon's key. Call once at boot. 0 = ok. */
int sw_secure_init(const char *wagon_no);

/* Seal a plaintext reading into the on-air encrypted advert.
 * `ctr` MUST be unique per node for the life of the key (nonce reuse breaks
 * CCM) - the sub-node persists it in NVS. Returns 0 on success. */
int sw_secure_seal(uint16_t wgn_group, uint8_t node_type, uint8_t node_id,
		   uint32_t ctr, const struct sw_adv_pt *pt,
		   struct sw_adv_enc *out);

/* Verify + decrypt an on-air advert into a plaintext sw_adv. Returns 0 on
 * success (tag verified); <0 if authentication fails (caller drops it). */
int sw_secure_open(const struct sw_adv_enc *in, struct sw_adv *out);

/*
 * Downlink (gateway -> node), same key, same AES-CCM, but nonce byte 7 = 0x01
 * instead of 0x00 so an uplink and a downlink can never share a nonce. `ctr`
 * is the downlink counter and must strictly increase per node; the receiver
 * persists the highest value it has accepted and rejects replays.
 */
int sw_secure_seal_dn(uint16_t wgn_group, uint8_t node_type, uint8_t node_id,
		      uint32_t ctr, const struct sw_dn_pt *pt,
		      struct sw_dn_enc *out);

/* Verify + decrypt a downlink frame. 0 on success; <0 if the tag fails, in
 * which case the caller MUST ignore the write entirely. */
int sw_secure_open_dn(const struct sw_dn_enc *in, struct sw_dn_pt *out);

/*
 * Bulk image chunk (gateway -> node), same key and construction, but nonce
 * byte 7 = 0x02 so an image chunk can never share a nonce with an advert
 * (0x00) or a config write (0x01). `ctr` is the IMAGE counter and must
 * strictly increase; the node keeps its own bar for it.
 *
 * VARIABLE LENGTH: only the first `pt_len` bytes of *pt are sealed, so a short
 * final chunk does not have to be padded. `pt_len` is written into the frame
 * as ct_len and is authenticated through the AAD, so an attacker cannot shorten
 * a frame to truncate an image and still pass verification.
 */
int sw_secure_seal_img(uint16_t wgn_group, uint8_t node_type, uint8_t node_id,
		       uint32_t ctr, const struct sw_img_pt *pt, uint16_t pt_len,
		       struct sw_img_enc *out);

/* Verify + decrypt an image chunk. Returns the plaintext length (>0) on
 * success, <0 if the tag fails - in which case the caller MUST discard it and
 * NOT advance any counter. */
int sw_secure_open_img(const struct sw_img_enc *in, struct sw_img_pt *out);

#endif /* SW_SECURE_H */
