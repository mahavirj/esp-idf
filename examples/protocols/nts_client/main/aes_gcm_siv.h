/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AES_GCM_SIV_KEY_LEN     16  /*!< Only AES-128-GCM-SIV is implemented */
#define AES_GCM_SIV_NONCE_LEN   12
#define AES_GCM_SIV_TAG_LEN     16

/**
 * @brief AES-128-GCM-SIV (RFC 8452) encryption
 *
 * @param key       16-byte key
 * @param nonce     12-byte nonce
 * @param ad        Associated data (may be NULL when ad_len is 0)
 * @param pt        Plaintext (may be NULL when pt_len is 0)
 * @param out       Output: ciphertext followed by the tag, pt_len + AES_GCM_SIV_TAG_LEN bytes
 */
esp_err_t nts_aes_gcm_siv_encrypt(const uint8_t *key, const uint8_t *nonce,
                                  const uint8_t *ad, size_t ad_len,
                                  const uint8_t *pt, size_t pt_len, uint8_t *out);

/**
 * @brief AES-128-GCM-SIV (RFC 8452) authenticated decryption
 *
 * @param ct        Ciphertext followed by the tag
 * @param ct_len    Length of ct, at least AES_GCM_SIV_TAG_LEN
 * @param out       Output plaintext, ct_len - AES_GCM_SIV_TAG_LEN bytes. Zeroed on failure.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_MAC if authentication fails
 */
esp_err_t nts_aes_gcm_siv_decrypt(const uint8_t *key, const uint8_t *nonce,
                                  const uint8_t *ad, size_t ad_len,
                                  const uint8_t *ct, size_t ct_len, uint8_t *out);

/**
 * @brief Known-answer self test
 *
 * @return true if the implementation reproduces the reference vectors and rejects tampered input
 */
bool nts_aes_gcm_siv_self_test(void);

#ifdef __cplusplus
}
#endif
