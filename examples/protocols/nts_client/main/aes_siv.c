/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* AES-SIV-CMAC (RFC 5297) on top of the PSA Crypto API (AES-CMAC and AES-CTR). */

#include <stdbool.h>
#include <string.h>
#include "psa/crypto.h"
#include "mbedtls/constant_time.h"
#include "mbedtls/platform_util.h"
#include "aes_siv.h"

#define BLK 16

static psa_status_t import_aes(const uint8_t *key, size_t key_len, psa_algorithm_t alg,
                               psa_key_usage_t usage, psa_key_id_t *id)
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, key_len * 8);
    psa_set_key_algorithm(&attr, alg);
    psa_set_key_usage_flags(&attr, usage);
    return psa_import_key(&attr, key, key_len, id);
}

/* CMAC over the concatenation of up to two buffers */
static psa_status_t cmac2(psa_key_id_t key, const uint8_t *a, size_t a_len,
                          const uint8_t *b, size_t b_len, uint8_t mac[BLK])
{
    psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;
    size_t mac_len = 0;
    psa_status_t st = psa_mac_sign_setup(&op, key, PSA_ALG_CMAC);
    if (st == PSA_SUCCESS && a_len) {
        st = psa_mac_update(&op, a, a_len);
    }
    if (st == PSA_SUCCESS && b_len) {
        st = psa_mac_update(&op, b, b_len);
    }
    if (st == PSA_SUCCESS) {
        st = psa_mac_sign_finish(&op, mac, BLK, &mac_len);
    }
    if (st != PSA_SUCCESS) {
        psa_mac_abort(&op);
    }
    return st;
}

/* Doubling in GF(2^128), RFC 5297 section 2.3 */
static void dbl(uint8_t d[BLK])
{
    uint8_t carry = d[0] >> 7;
    for (int i = 0; i < BLK - 1; i++) {
        d[i] = (uint8_t)((d[i] << 1) | (d[i + 1] >> 7));
    }
    d[BLK - 1] = (uint8_t)((d[BLK - 1] << 1) ^ (carry ? 0x87 : 0));
}

static void xor_blk(uint8_t *dst, const uint8_t *src)
{
    for (int i = 0; i < BLK; i++) {
        dst[i] ^= src[i];
    }
}

/* S2V, RFC 5297 section 2.4: the plaintext is the final string */
static psa_status_t s2v(const uint8_t *key, size_t key_len,
                        const uint8_t *const ad[], const size_t ad_len[], size_t ad_count,
                        const uint8_t *pt, size_t pt_len, uint8_t v[BLK])
{
    psa_key_id_t id = 0;
    uint8_t d[BLK] = {0};
    uint8_t t[BLK];
    psa_status_t st = import_aes(key, key_len, PSA_ALG_CMAC, PSA_KEY_USAGE_SIGN_MESSAGE, &id);
    if (st != PSA_SUCCESS) {
        return st;
    }

    st = cmac2(id, d, BLK, NULL, 0, d);     /* D = CMAC(K, <zero>) */
    for (size_t i = 0; st == PSA_SUCCESS && i < ad_count; i++) {
        dbl(d);
        st = cmac2(id, ad[i], ad_len[i], NULL, 0, t);
        xor_blk(d, t);
    }
    if (st == PSA_SUCCESS) {
        if (pt_len >= BLK) {
            /* T = pt xorend D */
            memcpy(t, pt + pt_len - BLK, BLK);
            xor_blk(t, d);
            st = cmac2(id, pt, pt_len - BLK, t, BLK, v);
        } else {
            /* T = dbl(D) xor pad(pt) */
            dbl(d);
            memset(t, 0, BLK);
            if (pt_len) {
                memcpy(t, pt, pt_len);
            }
            t[pt_len] = 0x80;
            xor_blk(t, d);
            st = cmac2(id, t, BLK, NULL, 0, v);
        }
    }
    psa_destroy_key(id);
    mbedtls_platform_zeroize(d, sizeof(d));
    mbedtls_platform_zeroize(t, sizeof(t));
    return st;
}

/* AES-CTR keystream XOR with the SIV-derived counter, RFC 5297 section 2.6 */
static psa_status_t siv_ctr(const uint8_t *key, size_t key_len, const uint8_t v[BLK],
                            const uint8_t *in, size_t len, uint8_t *out)
{
    if (len == 0) {
        return PSA_SUCCESS;
    }
    psa_key_id_t id = 0;
    psa_cipher_operation_t op = PSA_CIPHER_OPERATION_INIT;
    uint8_t q[BLK];
    size_t n = 0, fin = 0;

    memcpy(q, v, BLK);
    q[8] &= 0x7f;
    q[12] &= 0x7f;

    psa_status_t st = import_aes(key, key_len, PSA_ALG_CTR, PSA_KEY_USAGE_ENCRYPT, &id);
    if (st != PSA_SUCCESS) {
        return st;
    }
    st = psa_cipher_encrypt_setup(&op, id, PSA_ALG_CTR);
    if (st == PSA_SUCCESS) {
        st = psa_cipher_set_iv(&op, q, BLK);
    }
    if (st == PSA_SUCCESS) {
        st = psa_cipher_update(&op, in, len, out, len, &n);
    }
    if (st == PSA_SUCCESS) {
        st = psa_cipher_finish(&op, out + n, len - n, &fin);
    }
    if (st != PSA_SUCCESS) {
        psa_cipher_abort(&op);
    }
    psa_destroy_key(id);
    return st;
}

static bool key_len_ok(size_t key_len)
{
    return key_len == 32 || key_len == 48 || key_len == 64;
}

esp_err_t nts_aes_siv_encrypt(const uint8_t *key, size_t key_len,
                              const uint8_t *const ad[], const size_t ad_len[], size_t ad_count,
                              const uint8_t *pt, size_t pt_len, uint8_t *out)
{
    if (!key || !out || !key_len_ok(key_len) || (pt_len && !pt)) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t half = key_len / 2;
    uint8_t v[BLK];

    if (s2v(key, half, ad, ad_len, ad_count, pt, pt_len, v) != PSA_SUCCESS ||
            siv_ctr(key + half, half, v, pt, pt_len, out + BLK) != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    memcpy(out, v, BLK);
    return ESP_OK;
}

esp_err_t nts_aes_siv_decrypt(const uint8_t *key, size_t key_len,
                              const uint8_t *const ad[], const size_t ad_len[], size_t ad_count,
                              const uint8_t *ct, size_t ct_len, uint8_t *out)
{
    if (!key || !ct || !key_len_ok(key_len) || ct_len < BLK || (ct_len > BLK && !out)) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t half = key_len / 2;
    const size_t pt_len = ct_len - BLK;
    uint8_t v[BLK];
    esp_err_t ret = ESP_OK;

    if (siv_ctr(key + half, half, ct, ct + BLK, pt_len, out) != PSA_SUCCESS ||
            s2v(key, half, ad, ad_len, ad_count, out, pt_len, v) != PSA_SUCCESS) {
        ret = ESP_FAIL;
    } else if (mbedtls_ct_memcmp(v, ct, BLK) != 0) {
        ret = ESP_ERR_INVALID_MAC;
    }
    if (ret != ESP_OK && pt_len) {
        memset(out, 0, pt_len);
    }
    return ret;
}

/* RFC 5297 appendix A.1 test vector */
bool nts_aes_siv_self_test(void)
{
    static const uint8_t key[] = {
        0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6, 0xf5, 0xf4, 0xf3, 0xf2, 0xf1, 0xf0,
        0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
    };
    static const uint8_t ad0[] = {
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
        0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    };
    static const uint8_t pt[] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
    };
    static const uint8_t expected[] = {
        0x85, 0x63, 0x2d, 0x07, 0xc6, 0xe8, 0xf3, 0x7f, 0x95, 0x0a, 0xcd, 0x32, 0x0a, 0x2e, 0xcc, 0x93,
        0x40, 0xc0, 0x2b, 0x96, 0x90, 0xc4, 0xdc, 0x04, 0xda, 0xef, 0x7f, 0x6a, 0xfe, 0x5c,
    };
    const uint8_t *const ad[] = { ad0 };
    const size_t ad_len[] = { sizeof(ad0) };
    uint8_t ct[sizeof(expected)];
    uint8_t out[sizeof(pt)];

    if (nts_aes_siv_encrypt(key, sizeof(key), ad, ad_len, 1, pt, sizeof(pt), ct) != ESP_OK ||
            memcmp(ct, expected, sizeof(expected)) != 0) {
        return false;
    }
    if (nts_aes_siv_decrypt(key, sizeof(key), ad, ad_len, 1, ct, sizeof(ct), out) != ESP_OK ||
            memcmp(out, pt, sizeof(pt)) != 0) {
        return false;
    }
    ct[sizeof(ct) - 1] ^= 1;    /* a tampered ciphertext must be rejected */
    return nts_aes_siv_decrypt(key, sizeof(key), ad, ad_len, 1, ct, sizeof(ct), out) == ESP_ERR_INVALID_MAC;
}
