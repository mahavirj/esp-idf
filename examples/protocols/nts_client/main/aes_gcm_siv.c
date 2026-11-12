/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * AES-128-GCM-SIV (RFC 8452) on top of the PSA Crypto API (AES-ECB only).
 * POLYVAL is a plain bit-by-bit implementation: small, not fast, which is fine
 * for the few hundred bytes of an NTP packet.
 */

#include <stdbool.h>
#include <string.h>
#include <endian.h>
#include "psa/crypto.h"
#include "mbedtls/constant_time.h"
#include "mbedtls/platform_util.h"
#include "aes_gcm_siv.h"

#define BLK 16

static psa_status_t aes_import(const uint8_t key[AES_GCM_SIV_KEY_LEN], psa_key_id_t *id)
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, AES_GCM_SIV_KEY_LEN * 8);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
    return psa_import_key(&attr, key, AES_GCM_SIV_KEY_LEN, id);
}

static psa_status_t aes_block(psa_key_id_t id, const uint8_t in[BLK], uint8_t out[BLK])
{
    size_t len = 0;
    return psa_cipher_encrypt(id, PSA_ALG_ECB_NO_PADDING, in, BLK, out, BLK, &len);
}

/*
 * POLYVAL(H, X) = ByteReverse(GHASH(mulX_GHASH(ByteReverse(H)), ByteReverse(X))),
 * RFC 8452 appendix A. The state below is kept in the GHASH (byte-reversed) domain.
 */
typedef struct {
    uint8_t h[BLK];
    uint8_t y[BLK];
} polyval_t;

static void ghash_shift(uint8_t v[BLK])
{
    const uint8_t lsb = v[BLK - 1] & 1;
    for (int i = BLK - 1; i > 0; i--) {
        v[i] = (uint8_t)((v[i] >> 1) | (v[i - 1] << 7));
    }
    v[0] >>= 1;
    v[0] ^= (uint8_t)(0xe1 & -lsb);
}

static void polyval_init(polyval_t *p, const uint8_t h[BLK])
{
    for (int i = 0; i < BLK; i++) {
        p->h[i] = h[BLK - 1 - i];
    }
    ghash_shift(p->h);      /* mulX_GHASH */
    memset(p->y, 0, BLK);
}

static void polyval_block(polyval_t *p, const uint8_t x[BLK])
{
    uint8_t z[BLK] = {0};
    uint8_t v[BLK];

    for (int i = 0; i < BLK; i++) {
        p->y[i] ^= x[BLK - 1 - i];
    }
    memcpy(v, p->h, BLK);
    for (int i = 0; i < 128; i++) {
        const uint8_t mask = (uint8_t) - ((p->y[i / 8] >> (7 - i % 8)) & 1);
        for (int j = 0; j < BLK; j++) {
            z[j] ^= v[j] & mask;
        }
        ghash_shift(v);
    }
    memcpy(p->y, z, BLK);
}

/* Absorb a buffer, zero-padding the last block */
static void polyval_update(polyval_t *p, const uint8_t *data, size_t len)
{
    uint8_t last[BLK] = {0};
    for (; len >= BLK; data += BLK, len -= BLK) {
        polyval_block(p, data);
    }
    if (len) {
        memcpy(last, data, len);
        polyval_block(p, last);
    }
}

/* Per-nonce key derivation, RFC 8452 section 4 */
static psa_status_t derive_keys(const uint8_t *key, const uint8_t *nonce,
                                uint8_t auth_key[BLK], psa_key_id_t *enc_id)
{
    psa_key_id_t kid = 0;
    uint8_t in[BLK] = {0};
    uint8_t out[BLK];
    uint8_t derived[2 * BLK];
    psa_status_t st = aes_import(key, &kid);
    if (st != PSA_SUCCESS) {
        return st;
    }
    memcpy(in + 4, nonce, AES_GCM_SIV_NONCE_LEN);
    for (int i = 0; st == PSA_SUCCESS && i < 4; i++) {
        in[0] = (uint8_t)i;
        st = aes_block(kid, in, out);
        memcpy(derived + 8 * i, out, 8);
    }
    psa_destroy_key(kid);
    if (st == PSA_SUCCESS) {
        memcpy(auth_key, derived, BLK);
        st = aes_import(derived + BLK, enc_id);
    }
    mbedtls_platform_zeroize(out, sizeof(out));
    mbedtls_platform_zeroize(derived, sizeof(derived));
    return st;
}

static psa_status_t compute_tag(psa_key_id_t enc_id, const uint8_t auth_key[BLK], const uint8_t *nonce,
                                const uint8_t *ad, size_t ad_len,
                                const uint8_t *pt, size_t pt_len, uint8_t tag[BLK])
{
    polyval_t p;
    uint8_t s[BLK];

    polyval_init(&p, auth_key);
    polyval_update(&p, ad, ad_len);
    polyval_update(&p, pt, pt_len);
    le64enc(s, (uint64_t)ad_len * 8);
    le64enc(s + 8, (uint64_t)pt_len * 8);
    polyval_block(&p, s);

    for (int i = 0; i < BLK; i++) {
        s[i] = p.y[BLK - 1 - i];
    }
    for (int i = 0; i < AES_GCM_SIV_NONCE_LEN; i++) {
        s[i] ^= nonce[i];
    }
    s[BLK - 1] &= 0x7f;
    mbedtls_platform_zeroize(&p, sizeof(p));
    return aes_block(enc_id, s, tag);
}

/* AES-CTR with a 32-bit little-endian counter in the first four bytes */
static psa_status_t ctr_xor(psa_key_id_t enc_id, const uint8_t tag[BLK],
                            const uint8_t *in, size_t len, uint8_t *out)
{
    uint8_t ctr[BLK];
    uint8_t ks[BLK];
    psa_status_t st = PSA_SUCCESS;

    memcpy(ctr, tag, BLK);
    ctr[BLK - 1] |= 0x80;
    for (size_t off = 0; st == PSA_SUCCESS && off < len; off += BLK) {
        st = aes_block(enc_id, ctr, ks);
        for (size_t i = 0; i < BLK && off + i < len; i++) {
            out[off + i] = in[off + i] ^ ks[i];
        }
        for (int i = 0; i < 4 && ++ctr[i] == 0; i++) {
        }
    }
    mbedtls_platform_zeroize(ks, sizeof(ks));
    return st;
}

esp_err_t nts_aes_gcm_siv_encrypt(const uint8_t *key, const uint8_t *nonce,
                                  const uint8_t *ad, size_t ad_len,
                                  const uint8_t *pt, size_t pt_len, uint8_t *out)
{
    if (!key || !nonce || !out || (ad_len && !ad) || (pt_len && !pt)) {
        return ESP_ERR_INVALID_ARG;
    }
    psa_key_id_t enc_id = 0;
    uint8_t auth_key[BLK];
    uint8_t tag[BLK];

    psa_status_t st = derive_keys(key, nonce, auth_key, &enc_id);
    if (st != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    st = compute_tag(enc_id, auth_key, nonce, ad, ad_len, pt, pt_len, tag);
    if (st == PSA_SUCCESS) {
        st = ctr_xor(enc_id, tag, pt, pt_len, out);
    }
    psa_destroy_key(enc_id);
    mbedtls_platform_zeroize(auth_key, sizeof(auth_key));
    if (st != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    memcpy(out + pt_len, tag, BLK);
    return ESP_OK;
}

esp_err_t nts_aes_gcm_siv_decrypt(const uint8_t *key, const uint8_t *nonce,
                                  const uint8_t *ad, size_t ad_len,
                                  const uint8_t *ct, size_t ct_len, uint8_t *out)
{
    if (!key || !nonce || !ct || ct_len < BLK || (ad_len && !ad) || (ct_len > BLK && !out)) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t pt_len = ct_len - BLK;
    const uint8_t *tag = ct + pt_len;
    psa_key_id_t enc_id = 0;
    uint8_t auth_key[BLK];
    uint8_t expected[BLK];

    psa_status_t st = derive_keys(key, nonce, auth_key, &enc_id);
    if (st != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    st = ctr_xor(enc_id, tag, ct, pt_len, out);
    if (st == PSA_SUCCESS) {
        st = compute_tag(enc_id, auth_key, nonce, ad, ad_len, out, pt_len, expected);
    }
    psa_destroy_key(enc_id);
    mbedtls_platform_zeroize(auth_key, sizeof(auth_key));

    if (st != PSA_SUCCESS || mbedtls_ct_memcmp(expected, tag, BLK) != 0) {
        if (pt_len) {
            memset(out, 0, pt_len);
        }
        return st != PSA_SUCCESS ? ESP_FAIL : ESP_ERR_INVALID_MAC;
    }
    return ESP_OK;
}

/* RFC 8452 appendix C.1 test vector (8-byte plaintext, no associated data) */
bool nts_aes_gcm_siv_self_test(void)
{
    static const uint8_t key[AES_GCM_SIV_KEY_LEN] = { 0x01 };
    static const uint8_t nonce[AES_GCM_SIV_NONCE_LEN] = { 0x03 };
    static const uint8_t pt[8] = { 0x01 };
    static const uint8_t expected[] = {
        0xb5, 0xd8, 0x39, 0x33, 0x0a, 0xc7, 0xb7, 0x86,
        0x57, 0x87, 0x82, 0xff, 0xf6, 0x01, 0x3b, 0x81, 0x5b, 0x28, 0x7c, 0x22, 0x49, 0x3a, 0x36, 0x4c,
    };
    /* Unaligned associated data and plaintext, cross-checked with OpenSSL */
    static const uint8_t expected2[] = {
        0x78, 0x2e, 0x27, 0x69, 0x51, 0x14, 0x94, 0x17, 0x7d, 0xc1, 0xc1, 0xf4, 0x51, 0x86, 0x5c, 0x41,
        0x59, 0x79, 0x22, 0x7d, 0x2a, 0x37, 0xbb, 0xa0, 0xf6, 0xf9, 0xfd, 0x22, 0x95, 0x3c, 0x2c, 0x00,
        0xbd, 0x50, 0xae, 0xfa, 0x96, 0x9f, 0x77, 0x26, 0x76, 0x8c, 0xa5, 0x83, 0x9a, 0x95, 0xdb, 0x70,
        0x1b, 0x29, 0x23, 0xf5, 0xac,
    };
    uint8_t key2[AES_GCM_SIV_KEY_LEN], nonce2[AES_GCM_SIV_NONCE_LEN], ad2[20], pt2[37];
    uint8_t ct[sizeof(expected2)];
    uint8_t out[sizeof(pt2)];

    if (nts_aes_gcm_siv_encrypt(key, nonce, NULL, 0, pt, sizeof(pt), ct) != ESP_OK ||
            memcmp(ct, expected, sizeof(expected)) != 0) {
        return false;
    }
    /* key = 10 11 ..., nonce = a0 a1 ..., ad = 30 31 ..., pt = 50 51 ... */
    for (size_t i = 0; i < sizeof(key2); i++) {
        key2[i] = 0x10 + i;
    }
    for (size_t i = 0; i < sizeof(nonce2); i++) {
        nonce2[i] = 0xa0 + i;
    }
    for (size_t i = 0; i < sizeof(ad2); i++) {
        ad2[i] = 0x30 + i;
    }
    for (size_t i = 0; i < sizeof(pt2); i++) {
        pt2[i] = 0x50 + i;
    }
    if (nts_aes_gcm_siv_encrypt(key2, nonce2, ad2, sizeof(ad2), pt2, sizeof(pt2), ct) != ESP_OK ||
            memcmp(ct, expected2, sizeof(expected2)) != 0) {
        return false;
    }
    if (nts_aes_gcm_siv_decrypt(key2, nonce2, ad2, sizeof(ad2), ct, sizeof(ct), out) != ESP_OK ||
            memcmp(out, pt2, sizeof(pt2)) != 0) {
        return false;
    }
    ad2[0] ^= 1;    /* tampered associated data must be rejected */
    return nts_aes_gcm_siv_decrypt(key2, nonce2, ad2, sizeof(ad2), ct, sizeof(ct), out) == ESP_ERR_INVALID_MAC;
}
