/*
 * AES-CCM seal/open for the BLE advert, via the PSA Crypto API (CRACEN/Oberon
 * backend on nRF54L15). Layout (verified against a reference vector):
 *   AAD   (11 B, cleartext, authenticated): company_id, ver, wgn_group,
 *                                            node_type, node_id, ctr
 *   PT    (6 B, encrypted): flags, batt, value, value2   (struct sw_adv_pt)
 *   nonce (13 B): node_id | ctr(LE) | wgn_group(LE) | 6x00
 *   tag   (4 B)
 * Key = HKDF-SHA256(SW_MASTER_SECRET, salt="SmartWagon-v1", info="wgn:<no>").
 */
#include <zephyr/kernel.h>
#include <psa/crypto.h>
#include <string.h>
#include "sw_secure.h"
#include "sw_master_key.h"

#define SW_CCM_ALG   PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 4)
static const uint8_t SW_SALT[] = "SmartWagon-v1";   /* 13 chars, exclude NUL */

static psa_key_id_t g_key;

static void put16(uint8_t *b, uint16_t v){ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *b, uint32_t v){ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8);
					   b[2]=(uint8_t)(v>>16); b[3]=(uint8_t)(v>>24); }

static void build_aad(uint8_t a[11], uint16_t grp, uint8_t nt, uint8_t nid, uint32_t ctr)
{
	put16(a + 0, SW_COMPANY_ID);
	a[2] = SW_PROTO_VER;
	put16(a + 3, grp);
	a[5] = nt;
	a[6] = nid;
	put32(a + 7, ctr);
}
/*
 * Nonce byte 7 is the DIRECTION tag: 0 = uplink advert, 1 = downlink config.
 * Uplink keeps 0, which is what the original layout already had in that pad
 * byte, so the on-air advert format is unchanged.
 *
 * This separation is load-bearing, not cosmetic: uplink and downlink use the
 * same key and each has its own counter, so without it an advert with ctr=N
 * and a config write with ctr=N would produce an IDENTICAL nonce. Reusing a
 * nonce under one key in CCM leaks the keystream and lets an attacker forge
 * messages - it is the classic way to destroy an otherwise sound AEAD design.
 */
#define SW_DIR_UP    0x00
#define SW_DIR_DOWN  0x01

static void build_nonce_dir(uint8_t n[13], uint16_t grp, uint8_t nid,
			    uint32_t ctr, uint8_t dir)
{
	n[0] = nid;
	put32(n + 1, ctr);
	put16(n + 5, grp);
	memset(n + 7, 0, 6);
	n[7] = dir;
}

static void build_nonce(uint8_t n[13], uint16_t grp, uint8_t nid, uint32_t ctr)
{
	build_nonce_dir(n, grp, nid, ctr, SW_DIR_UP);
}

int sw_secure_init(const char *wagon_no)
{
	if (psa_crypto_init() != PSA_SUCCESS) {
		return -1;
	}

	/* Import the fleet master secret as an HKDF derivation key. */
	psa_key_attributes_t ma = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_usage_flags(&ma, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&ma, PSA_ALG_HKDF(PSA_ALG_SHA_256));
	psa_set_key_type(&ma, PSA_KEY_TYPE_DERIVE);
	psa_key_id_t mk;
	if (psa_import_key(&ma, SW_MASTER_SECRET, sizeof(SW_MASTER_SECRET), &mk)
	    != PSA_SUCCESS) {
		return -2;
	}

	char info[40];
	int ilen = snprintk(info, sizeof(info), "wgn:%s", wagon_no);

	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	int rc = -3;
	if (psa_key_derivation_setup(&op, PSA_ALG_HKDF(PSA_ALG_SHA_256)) == PSA_SUCCESS &&
	    psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT,
					   SW_SALT, sizeof(SW_SALT) - 1) == PSA_SUCCESS &&
	    psa_key_derivation_input_key(&op, PSA_KEY_DERIVATION_INPUT_SECRET, mk) == PSA_SUCCESS &&
	    psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO,
					   info, ilen) == PSA_SUCCESS) {
		psa_key_attributes_t ka = PSA_KEY_ATTRIBUTES_INIT;
		psa_set_key_usage_flags(&ka, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&ka, SW_CCM_ALG);
		psa_set_key_type(&ka, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&ka, 128);
		if (psa_key_derivation_output_key(&ka, &op, &g_key) == PSA_SUCCESS) {
			rc = 0;
		}
	}
	psa_key_derivation_abort(&op);
	psa_destroy_key(mk);
	return rc;
}

int sw_secure_seal(uint16_t grp, uint8_t nt, uint8_t nid, uint32_t ctr,
		   const struct sw_adv_pt *pt, struct sw_adv_enc *out)
{
	uint8_t aad[11], nonce[13], obuf[16];
	size_t olen;

	build_aad(aad, grp, nt, nid, ctr);
	build_nonce(nonce, grp, nid, ctr);

	if (psa_aead_encrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     (const uint8_t *)pt, sizeof(*pt),
			     obuf, sizeof(obuf), &olen) != PSA_SUCCESS ||
	    olen != sizeof(out->ct) + sizeof(out->mic)) {
		return -1;
	}
	out->company_id = SW_COMPANY_ID;
	out->proto_ver  = SW_PROTO_VER;
	out->wgn_group  = grp;
	out->node_type  = nt;
	out->node_id    = nid;
	out->ctr        = ctr;
	memcpy(out->ct,  obuf, sizeof(out->ct));
	memcpy(out->mic, obuf + sizeof(out->ct), sizeof(out->mic));
	return 0;
}

int sw_secure_open(const struct sw_adv_enc *in, struct sw_adv *out)
{
	uint8_t aad[11], nonce[13], cbuf[16], pbuf[8];
	size_t plen;

	build_aad(aad, in->wgn_group, in->node_type, in->node_id, in->ctr);
	build_nonce(nonce, in->wgn_group, in->node_id, in->ctr);
	memcpy(cbuf, in->ct, sizeof(in->ct));
	memcpy(cbuf + sizeof(in->ct), in->mic, sizeof(in->mic));

	if (psa_aead_decrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     cbuf, sizeof(in->ct) + sizeof(in->mic),
			     pbuf, sizeof(pbuf), &plen) != PSA_SUCCESS ||
	    plen != sizeof(struct sw_adv_pt)) {
		return -1;   /* authentication failed -> caller drops the advert */
	}

	struct sw_adv_pt pt;
	memcpy(&pt, pbuf, sizeof(pt));
	out->company_id = in->company_id;
	out->proto_ver  = in->proto_ver;
	out->wgn_group  = in->wgn_group;
	out->node_type  = in->node_type;
	out->node_id    = in->node_id;
	out->seq        = (uint8_t)in->ctr;
	out->flags      = pt.flags;
	out->batt       = pt.batt;
	out->value      = pt.value;
	out->value2     = pt.value2;
	return 0;
}

/* ---- downlink (gateway -> node) ---------------------------------------- */

int sw_secure_seal_dn(uint16_t grp, uint8_t nt, uint8_t nid, uint32_t ctr,
		      const struct sw_dn_pt *pt, struct sw_dn_enc *out)
{
	uint8_t aad[11], nonce[13], obuf[16];
	size_t olen;

	build_aad(aad, grp, nt, nid, ctr);
	build_nonce_dir(nonce, grp, nid, ctr, SW_DIR_DOWN);

	if (psa_aead_encrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     (const uint8_t *)pt, sizeof(*pt),
			     obuf, sizeof(obuf), &olen) != PSA_SUCCESS ||
	    olen != sizeof(out->ct) + sizeof(out->mic)) {
		return -1;
	}
	out->company_id = SW_COMPANY_ID;
	out->proto_ver  = SW_PROTO_VER;
	out->wgn_group  = grp;
	out->node_type  = nt;
	out->node_id    = nid;
	out->ctr        = ctr;
	memcpy(out->ct,  obuf, sizeof(out->ct));
	memcpy(out->mic, obuf + sizeof(out->ct), sizeof(out->mic));
	return 0;
}

int sw_secure_open_dn(const struct sw_dn_enc *in, struct sw_dn_pt *out)
{
	uint8_t aad[11], nonce[13], cbuf[16], pbuf[8];
	size_t plen;

	build_aad(aad, in->wgn_group, in->node_type, in->node_id, in->ctr);
	build_nonce_dir(nonce, in->wgn_group, in->node_id, in->ctr, SW_DIR_DOWN);
	memcpy(cbuf, in->ct, sizeof(in->ct));
	memcpy(cbuf + sizeof(in->ct), in->mic, sizeof(in->mic));

	if (psa_aead_decrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     cbuf, sizeof(in->ct) + sizeof(in->mic),
			     pbuf, sizeof(pbuf), &plen) != PSA_SUCCESS ||
	    plen != sizeof(struct sw_dn_pt)) {
		return -1;   /* forged or corrupted -> caller ignores the write */
	}
	memcpy(out, pbuf, sizeof(*out));
	return 0;
}

/* ---- bulk image chunk (gateway -> node) ---------------------------------
 *
 * Third nonce domain. Uplink adverts use 0x00 and config writes 0x01; image
 * chunks use 0x02. All three share one key and each keeps its own counter, so
 * without the separation a chunk with ctr=N and an advert with ctr=N would
 * build the SAME nonce. CCM nonce reuse under one key leaks keystream and
 * permits forgery - the classic way to destroy an otherwise sound AEAD design.
 *
 * The AAD is extended with the ciphertext LENGTH. Without that, an attacker
 * could truncate a frame and the shortened body would still authenticate,
 * silently corrupting the image being assembled. Authenticating the length
 * makes any such edit fail the tag.
 */
#define SW_DIR_IMG   0x02

static void build_aad_img(uint8_t a[13], uint16_t grp, uint8_t nt, uint8_t nid,
			  uint32_t ctr, uint16_t ct_len)
{
	build_aad(a, grp, nt, nid, ctr);      /* first 11 bytes, shared layout */
	put16(a + 11, ct_len);
}

int sw_secure_seal_img(uint16_t wgn_group, uint8_t node_type, uint8_t node_id,
		       uint32_t ctr, const struct sw_img_pt *pt, uint16_t pt_len,
		       struct sw_img_enc *out)
{
	uint8_t aad[13], nonce[13];
	uint8_t obuf[sizeof(struct sw_img_pt) + 4];
	size_t  olen;

	if (pt_len == 0 || pt_len > sizeof(struct sw_img_pt)) {
		return -1;
	}

	build_aad_img(aad, wgn_group, node_type, node_id, ctr, pt_len);
	build_nonce_dir(nonce, wgn_group, node_id, ctr, SW_DIR_IMG);

	if (psa_aead_encrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     (const uint8_t *)pt, pt_len,
			     obuf, sizeof(obuf), &olen) != PSA_SUCCESS ||
	    olen != (size_t)pt_len + sizeof(out->mic)) {
		return -1;
	}

	out->company_id = SW_COMPANY_ID;
	out->proto_ver  = SW_PROTO_VER;
	out->wgn_group  = wgn_group;
	out->node_type  = node_type;
	out->node_id    = node_id;
	out->ctr        = ctr;
	out->ct_len     = pt_len;
	memcpy(out->ct,  obuf, pt_len);
	memcpy(out->mic, obuf + pt_len, sizeof(out->mic));
	return 0;
}

int sw_secure_open_img(const struct sw_img_enc *in, struct sw_img_pt *out)
{
	uint8_t aad[13], nonce[13];
	uint8_t cbuf[sizeof(struct sw_img_pt) + 4];
	uint8_t pbuf[sizeof(struct sw_img_pt)];
	size_t  plen;
	uint16_t ct_len = in->ct_len;

	/* Bound BEFORE using it as a length - this value arrives off the air. */
	if (ct_len == 0 || ct_len > sizeof(struct sw_img_pt)) {
		return -1;
	}

	build_aad_img(aad, in->wgn_group, in->node_type, in->node_id,
		      in->ctr, ct_len);
	build_nonce_dir(nonce, in->wgn_group, in->node_id, in->ctr, SW_DIR_IMG);

	memcpy(cbuf, in->ct, ct_len);
	memcpy(cbuf + ct_len, in->mic, sizeof(in->mic));

	if (psa_aead_decrypt(g_key, SW_CCM_ALG, nonce, sizeof(nonce),
			     aad, sizeof(aad),
			     cbuf, (size_t)ct_len + sizeof(in->mic),
			     pbuf, sizeof(pbuf), &plen) != PSA_SUCCESS ||
	    plen != ct_len) {
		return -1;   /* forged, truncated or corrupted -> discard */
	}

	memset(out, 0, sizeof(*out));
	memcpy(out, pbuf, plen);
	return (int)plen;
}
