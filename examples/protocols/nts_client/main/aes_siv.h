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

/** Size of the synthetic IV that AES-SIV prepends to the ciphertext */
#define AES_SIV_TAG_LEN 16

/**
 * @brief AES-SIV-CMAC (RFC 5297) encryption
 *
 * @param key       SIV key: 32, 48 or 64 bytes (CMAC key followed by CTR key)
 * @param key_len   Length of key
 * @param ad        Array of associated data components (S2V inputs, in order)
 * @param ad_len    Length of each associated data component
 * @param ad_count  Number of associated data components
 * @param pt        Plaintext (may be NULL when pt_len is 0)
 * @param pt_len    Plaintext length
 * @param out       Output: synthetic IV followed by ciphertext, pt_len + AES_SIV_TAG_LEN bytes
 */
esp_err_t nts_aes_siv_encrypt(const uint8_t *key, size_t key_len,
                              const uint8_t *const ad[], const size_t ad_len[], size_t ad_count,
                              const uint8_t *pt, size_t pt_len, uint8_t *out);

/**
 * @brief AES-SIV-CMAC (RFC 5297) authenticated decryption
 *
 * @param ct        Synthetic IV followed by ciphertext
 * @param ct_len    Length of ct, at least AES_SIV_TAG_LEN
 * @param out       Output plaintext, ct_len - AES_SIV_TAG_LEN bytes. Zeroed on failure.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_MAC if authentication fails
 */
esp_err_t nts_aes_siv_decrypt(const uint8_t *key, size_t key_len,
                              const uint8_t *const ad[], const size_t ad_len[], size_t ad_count,
                              const uint8_t *ct, size_t ct_len, uint8_t *out);

/**
 * @brief Known-answer self test
 *
 * @return true if the implementation reproduces the reference vectors and rejects tampered input
 */
bool nts_aes_siv_self_test(void);

#ifdef __cplusplus
}
#endif
