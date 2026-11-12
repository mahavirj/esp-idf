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

#define NTS_MAX_COOKIES     8       /*!< Cookie pool size recommended by RFC 8915 */
#define NTS_MAX_COOKIE_LEN  256     /*!< Largest cookie this client stores */
#define NTS_MAX_KEY_LEN     32      /*!< AES-SIV-CMAC-256 key length; AES-128-GCM-SIV uses 16 */
#define NTS_AEAD_AES_SIV_CMAC_256   15  /*!< IANA AEAD ID, mandatory in RFC 8915 */
#define NTS_AEAD_AES_128_GCM_SIV    30  /*!< IANA AEAD ID, RFC 8452 */
#define NTS_MAX_HOST_LEN    254     /*!< Longest DNS name plus terminator */

/**
 * @brief State of one NTS association: keys and cookies from NTS-KE
 *
 * About 2.4 KB, so keep it off small task stacks.
 */
typedef struct {
    uint16_t aead;                              /*!< Negotiated AEAD algorithm ID */
    uint8_t c2s[NTS_MAX_KEY_LEN];               /*!< Client-to-server key, length depends on aead */
    uint8_t s2c[NTS_MAX_KEY_LEN];               /*!< Server-to-client key */
    char ntp_host[NTS_MAX_HOST_LEN];            /*!< NTP server to query */
    uint16_t ntp_port;                          /*!< NTP server UDP port */
    uint8_t cookie[NTS_MAX_COOKIES][NTS_MAX_COOKIE_LEN];
    uint16_t cookie_len[NTS_MAX_COOKIES];       /*!< 0 marks a free slot */
} nts_session_t;

/**
 * @brief Result of one authenticated NTP exchange
 */
typedef struct {
    int64_t offset_us;      /*!< Server clock minus local clock */
    int64_t delay_us;       /*!< Round-trip network delay */
    uint8_t stratum;        /*!< Server stratum */
    uint8_t new_cookies;    /*!< Fresh cookies received in this response */
    char kiss[5];           /*!< Kiss code (e.g. "RATE", "DENY") of an authenticated stratum 0 reply, else empty */
} nts_time_t;

/**
 * @brief Run the NTS key establishment (RFC 8915 section 4) over TLS 1.3
 *
 * Negotiates NTPv4 with AES-128-GCM-SIV (preferred) or AES-SIV-CMAC-256, derives
 * the session keys from the TLS exporter and fills the cookie pool. The server certificate is verified
 * against the ESP x509 certificate bundle.
 *
 * @param host      NTS-KE server host name, shorter than NTS_MAX_HOST_LEN
 * @param port      NTS-KE server TCP port (4460 by default)
 * @param session   Output session, overwritten. It is cleared if the handshake fails.
 */
esp_err_t nts_ke_handshake(const char *host, uint16_t port, nts_session_t *session);

/**
 * @brief Number of unused cookies left in the session
 */
int nts_cookie_count(const nts_session_t *session);

/**
 * @brief Send one NTS-protected NTPv4 request and verify the response
 *
 * Spends one cookie once the request is ready to be sent, whether or not a response arrives.
 *
 * @return
 *      - ESP_OK: response authenticated, result filled
 *      - ESP_ERR_NOT_FOUND: no cookies left, run nts_ke_handshake() again
 *      - ESP_ERR_TIMEOUT: no valid response within timeout_ms
 *      - ESP_ERR_INVALID_STATE: the server rejected the cookie (NTS NAK), run nts_ke_handshake() again
 *      - ESP_ERR_INVALID_RESPONSE: authenticated response that carries no usable time: a kiss-o'-death
 *        (result->kiss is set) or an unsynchronised server. This is not a reason to run NTS-KE again.
 */
esp_err_t nts_ntp_query(nts_session_t *session, int timeout_ms, nts_time_t *result);

#ifdef __cplusplus
}
#endif
