/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * Minimal Network Time Security (RFC 8915) client:
 *  - NTS-KE over TLS 1.3 (ALPN "ntske/1"), keys from the TLS exporter
 *  - NTPv4 client mode request protected with AES-128-GCM-SIV or AES-SIV-CMAC-256
 */

#include <string.h>
#include <endian.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "mbedtls/ssl.h"
#include "mbedtls/platform_util.h"
#include "psa/crypto.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "aes_siv.h"
#include "aes_gcm_siv.h"
#include "nts_client.h"

static const char *TAG = "nts";

/* NTS-KE (RFC 8915 section 4) */
#define KE_ALPN                 "ntske/1"
#define KE_EXPORTER_LABEL       "EXPORTER-network-time-security"
#define KE_REC_CRITICAL         0x8000
#define KE_REC_END              0
#define KE_REC_NEXT_PROTO       1
#define KE_REC_ERROR            2
#define KE_REC_WARNING          3
#define KE_REC_AEAD             4
#define KE_REC_COOKIE           5
#define KE_REC_SERVER           6
#define KE_REC_PORT             7
/* Non-standard record introduced by chrony: "I export AES-128-GCM-SIV keys the RFC 8915 way" */
#define KE_REC_COMPLIANT_EXPORT 1024
#define KE_PROTO_NTPV4          0
#define KE_TIMEOUT_MS           20000   /* for the connection, and for the whole record exchange */
#define KE_MAX_RESPONSE         4096

/* NTP extension fields (RFC 8915 section 5) */
#define NTP_HDR_LEN             48
#define NTP_PORT                123
#define EF_UNIQUE_ID            0x0104
#define EF_COOKIE               0x0204
#define EF_COOKIE_PLACEHOLDER   0x0304
#define EF_AUTHENTICATOR        0x0404
#define UID_LEN                 32
#define AEAD_TAG_LEN            16      /* same for both supported algorithms */
#define NTP_MAX_PACKET          1280
#define NTP_UNIX_OFFSET         2208988800ULL

static inline size_t pad4(size_t n)
{
    return (n + 3) & ~(size_t)3;
}

int nts_cookie_count(const nts_session_t *s)
{
    int n = 0;
    for (int i = 0; i < NTS_MAX_COOKIES; i++) {
        n += s->cookie_len[i] != 0;
    }
    return n;
}

/* Index of the first unused cookie, or -1 */
static int cookie_first(const nts_session_t *s)
{
    for (int i = 0; i < NTS_MAX_COOKIES; i++) {
        if (s->cookie_len[i]) {
            return i;
        }
    }
    return -1;
}

static bool cookie_store(nts_session_t *s, const uint8_t *cookie, size_t len)
{
    if (len == 0 || len > NTS_MAX_COOKIE_LEN) {
        return false;
    }
    for (int i = 0; i < NTS_MAX_COOKIES; i++) {
        if (s->cookie_len[i] == 0) {
            memcpy(s->cookie[i], cookie, len);
            s->cookie_len[i] = (uint16_t)len;
            return true;
        }
    }
    return false;   /* pool full, drop the extra cookie */
}

/* ------------------------------------------------------------------ AEAD */

static size_t aead_key_len(const nts_session_t *s)
{
    return s->aead == NTS_AEAD_AES_128_GCM_SIV ? AES_GCM_SIV_KEY_LEN : 32;
}

static size_t aead_nonce_len(const nts_session_t *s)
{
    return s->aead == NTS_AEAD_AES_128_GCM_SIV ? AES_GCM_SIV_NONCE_LEN : 16;
}

/* out receives pt_len + AEAD_TAG_LEN bytes */
static esp_err_t aead_seal(const nts_session_t *s, const uint8_t *ad, size_t ad_len,
                           const uint8_t *nonce, const uint8_t *pt, size_t pt_len, uint8_t *out)
{
    if (s->aead == NTS_AEAD_AES_128_GCM_SIV) {
        return nts_aes_gcm_siv_encrypt(s->c2s, nonce, ad, ad_len, pt, pt_len, out);
    }
    /* RFC 8915 section 5.6: the nonce is the S2V component after the associated data */
    const uint8_t *const comp[] = { ad, nonce };
    const size_t comp_len[] = { ad_len, aead_nonce_len(s) };
    return nts_aes_siv_encrypt(s->c2s, aead_key_len(s), comp, comp_len, 2, pt, pt_len, out);
}

/* Decrypts in place: *pt points at the ct_len - AEAD_TAG_LEN plaintext bytes inside ct */
static esp_err_t aead_open(const nts_session_t *s, const uint8_t *ad, size_t ad_len,
                           const uint8_t *nonce, size_t nonce_len,
                           uint8_t *ct, size_t ct_len, uint8_t **pt)
{
    if (s->aead == NTS_AEAD_AES_128_GCM_SIV) {
        if (nonce_len != AES_GCM_SIV_NONCE_LEN) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        *pt = ct;                   /* ciphertext, then tag */
        return nts_aes_gcm_siv_decrypt(s->s2c, nonce, ad, ad_len, ct, ct_len, *pt);
    }
    const uint8_t *const comp[] = { ad, nonce };
    const size_t comp_len[] = { ad_len, nonce_len };
    *pt = ct + AEAD_TAG_LEN;        /* synthetic IV, then ciphertext */
    return nts_aes_siv_decrypt(s->s2c, aead_key_len(s), comp, comp_len, 2, ct, ct_len, *pt);
}

/* ---------------------------------------------------------------- NTS-KE */

/* True once the buffer holds every record up to and including End of Message */
static bool ke_message_complete(const uint8_t *buf, size_t len)
{
    for (size_t off = 0; len - off >= 4;) {
        size_t body_len = be16dec(buf + off + 2);
        if (len - off - 4 < body_len) {
            return false;
        }
        if ((be16dec(buf + off) & ~KE_REC_CRITICAL) == KE_REC_END) {
            return true;
        }
        off += 4 + body_len;
    }
    return false;
}

/* Interpret a complete NTS-KE response (see ke_message_complete) */
static esp_err_t ke_parse(const uint8_t *buf, size_t len, const char *ke_host, nts_session_t *s,
                          bool *compliant_export)
{
    bool got_proto = false, got_aead = false, got_server = false, got_port = false;

    memset(s->cookie_len, 0, sizeof(s->cookie_len));
    strlcpy(s->ntp_host, ke_host, sizeof(s->ntp_host));
    s->ntp_port = NTP_PORT;

    for (size_t off = 0;;) {
        uint16_t type = be16dec(buf + off);
        size_t body_len = be16dec(buf + off + 2);
        const uint8_t *body = buf + off + 4;
        bool critical = type & KE_REC_CRITICAL;
        type &= ~KE_REC_CRITICAL;
        off += 4 + body_len;

        if (type == KE_REC_END) {
            break;
        }
        switch (type) {
        case KE_REC_ERROR:
            ESP_LOGE(TAG, "NTS-KE server error %d", body_len >= 2 ? be16dec(body) : -1);
            return ESP_ERR_INVALID_RESPONSE;
        case KE_REC_WARNING:
            ESP_LOGW(TAG, "NTS-KE server warning %d", body_len >= 2 ? be16dec(body) : -1);
            break;
        case KE_REC_NEXT_PROTO:
            ESP_RETURN_ON_FALSE(body_len == 2 && be16dec(body) == KE_PROTO_NTPV4, ESP_ERR_INVALID_RESPONSE,
                                TAG, "server did not select NTPv4");
            got_proto = true;
            break;
        case KE_REC_AEAD:
            s->aead = body_len == 2 ? be16dec(body) : 0;
            ESP_RETURN_ON_FALSE(s->aead == NTS_AEAD_AES_128_GCM_SIV || s->aead == NTS_AEAD_AES_SIV_CMAC_256,
                                ESP_ERR_INVALID_RESPONSE, TAG, "server selected an AEAD that was not offered");
            got_aead = true;
            break;
        case KE_REC_COMPLIANT_EXPORT:
            *compliant_export = true;
            break;
        case KE_REC_COOKIE:
            if (!cookie_store(s, body, body_len)) {
                ESP_LOGW(TAG, "dropping cookie of %u bytes", (unsigned)body_len);
            }
            break;
        case KE_REC_SERVER:
            ESP_RETURN_ON_FALSE(!got_server, ESP_ERR_INVALID_RESPONSE, TAG, "multiple NTP server records");
            got_server = true;
            ESP_RETURN_ON_FALSE(body_len > 0 && body_len < sizeof(s->ntp_host), ESP_ERR_INVALID_RESPONSE,
                                TAG, "bad NTP server name length %u", (unsigned)body_len);
            memcpy(s->ntp_host, body, body_len);
            s->ntp_host[body_len] = '\0';
            break;
        case KE_REC_PORT:
            ESP_RETURN_ON_FALSE(!got_port, ESP_ERR_INVALID_RESPONSE, TAG, "multiple NTP port records");
            got_port = true;
            ESP_RETURN_ON_FALSE(body_len == 2 && be16dec(body) != 0, ESP_ERR_INVALID_RESPONSE,
                                TAG, "bad NTP port record");
            s->ntp_port = be16dec(body);
            break;
        default:
            ESP_RETURN_ON_FALSE(!critical, ESP_ERR_INVALID_RESPONSE,
                                TAG, "unknown critical record type %u", type);
            break;
        }
    }

    ESP_RETURN_ON_FALSE(got_proto && got_aead, ESP_ERR_INVALID_RESPONSE, TAG, "incomplete NTS-KE response");
    ESP_RETURN_ON_FALSE(nts_cookie_count(s) > 0, ESP_ERR_INVALID_RESPONSE, TAG, "NTS-KE response has no cookies");
    return ESP_OK;
}

static esp_err_t ke_export_key(mbedtls_ssl_context *ssl, uint8_t context_aead, uint8_t direction,
                               uint8_t *key, size_t key_len)
{
    /* RFC 8915 section 5.1: protocol ID, AEAD ID, direction (0 = C2S, 1 = S2C) */
    const uint8_t context[5] = { 0, KE_PROTO_NTPV4, 0, context_aead, direction };
    int ret = mbedtls_ssl_export_keying_material(ssl, key, key_len,
                                                 KE_EXPORTER_LABEL, strlen(KE_EXPORTER_LABEL),
                                                 context, sizeof(context), 1);
    ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "TLS key export failed: -0x%x", -ret);
    return ESP_OK;
}

esp_err_t nts_ke_handshake(const char *host, uint16_t port, nts_session_t *s)
{
    ESP_RETURN_ON_FALSE(host && s && strlen(host) < NTS_MAX_HOST_LEN, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    static const char *alpn[] = { KE_ALPN, NULL };
    /* Next Protocol = NTPv4; AEAD in order of preference; compliant export (not critical); End of Message */
    static const uint8_t request[] = {
        0x80, KE_REC_NEXT_PROTO, 0x00, 0x02, 0x00, KE_PROTO_NTPV4,
        0x80, KE_REC_AEAD,       0x00, 0x04, 0x00, NTS_AEAD_AES_128_GCM_SIV, 0x00, NTS_AEAD_AES_SIV_CMAC_256,
        KE_REC_COMPLIANT_EXPORT >> 8, KE_REC_COMPLIANT_EXPORT & 0xff, 0x00, 0x00,
        0x80, KE_REC_END,        0x00, 0x00,
    };
    const esp_tls_cfg_t cfg = {
        .alpn_protos = alpn,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .tls_version = ESP_TLS_VER_TLS_1_3,
        .timeout_ms = KE_TIMEOUT_MS,
    };

    esp_err_t ret = ESP_OK;
    uint8_t *buf = NULL;
    mbedtls_platform_zeroize(s, sizeof(*s));
    esp_tls_t *tls = esp_tls_init();
    ESP_RETURN_ON_FALSE(tls, ESP_ERR_NO_MEM, TAG, "esp_tls_init failed");

    ESP_GOTO_ON_FALSE(esp_tls_conn_new_sync(host, strlen(host), port, &cfg, tls) == 1,
                      ESP_FAIL, out, TAG, "TLS connection to %s:%u failed", host, port);

    mbedtls_ssl_context *ssl = esp_tls_get_ssl_context(tls);
    const char *proto = mbedtls_ssl_get_alpn_protocol(ssl);
    ESP_GOTO_ON_FALSE(mbedtls_ssl_get_version_number(ssl) == MBEDTLS_SSL_VERSION_TLS1_3,
                      ESP_ERR_INVALID_VERSION, out, TAG, "NTS-KE requires TLS 1.3");
    ESP_GOTO_ON_FALSE(proto && strcmp(proto, KE_ALPN) == 0,
                      ESP_ERR_INVALID_RESPONSE, out, TAG, "server did not negotiate ALPN " KE_ALPN);

    /*
     * A socket timeout shows up as WANT_READ/WANT_WRITE, so the loops below need their
     * own deadline or a server that goes silent after the TLS handshake would hold us forever.
     */
    const int64_t deadline = esp_timer_get_time() + KE_TIMEOUT_MS * 1000LL;

    for (size_t sent = 0; sent < sizeof(request);) {
        ESP_GOTO_ON_FALSE(esp_timer_get_time() < deadline, ESP_ERR_TIMEOUT, out, TAG, "NTS-KE request timed out");
        int n = esp_tls_conn_write(tls, request + sent, sizeof(request) - sent);
        if (n == ESP_TLS_ERR_SSL_WANT_READ || n == ESP_TLS_ERR_SSL_WANT_WRITE) {
            continue;
        }
        ESP_GOTO_ON_FALSE(n > 0, ESP_FAIL, out, TAG, "NTS-KE write failed: -0x%x", -n);
        sent += n;
    }

    buf = malloc(KE_MAX_RESPONSE);
    ESP_GOTO_ON_FALSE(buf, ESP_ERR_NO_MEM, out, TAG, "no memory for NTS-KE response");

    size_t len = 0;
    bool compliant_export = false;
    while (!ke_message_complete(buf, len)) {
        ESP_GOTO_ON_FALSE(len < KE_MAX_RESPONSE, ESP_ERR_INVALID_SIZE, out, TAG, "NTS-KE response too large");
        ESP_GOTO_ON_FALSE(esp_timer_get_time() < deadline, ESP_ERR_TIMEOUT, out, TAG, "NTS-KE response timed out");
        int n = esp_tls_conn_read(tls, buf + len, KE_MAX_RESPONSE - len);
        if (n == ESP_TLS_ERR_SSL_WANT_READ || n == ESP_TLS_ERR_SSL_WANT_WRITE) {
            continue;
        }
        ESP_GOTO_ON_FALSE(n > 0, ESP_FAIL, out, TAG, "NTS-KE read failed: %s0x%x", n < 0 ? "-" : "", n < 0 ? -n : n);
        len += n;
    }
    ESP_GOTO_ON_ERROR(ke_parse(buf, len, host, s, &compliant_export), out, TAG, "NTS-KE response rejected");

    /*
     * Early chrony exported AES-128-GCM-SIV keys with the AES-SIV-CMAC-256 ID in the
     * exporter context. Servers that follow RFC 8915 here echo the compliant-export record.
     */
    const uint8_t context_aead = compliant_export ? s->aead : NTS_AEAD_AES_SIV_CMAC_256;
    ESP_GOTO_ON_ERROR(ke_export_key(ssl, context_aead, 0, s->c2s, aead_key_len(s)), out, TAG, "C2S key");
    ESP_GOTO_ON_ERROR(ke_export_key(ssl, context_aead, 1, s->s2c, aead_key_len(s)), out, TAG, "S2C key");

    ESP_LOGI(TAG, "NTS-KE done: AEAD %u, %d cookies of %u bytes, NTP server %s:%u",
             s->aead, nts_cookie_count(s), s->cookie_len[0], s->ntp_host, s->ntp_port);

out:
    if (ret != ESP_OK) {
        /* Never leave new cookies next to missing or stale keys */
        mbedtls_platform_zeroize(s, sizeof(*s));
    }
    free(buf);
    esp_tls_conn_destroy(tls);
    return ret;
}

/* ------------------------------------------------------- NTS-protected NTP */

static int64_t ntp_ts_to_unix_us(const uint8_t *p)
{
    uint64_t sec = be32dec(p);
    uint64_t frac = be32dec(p + 4);
    if (!(sec & 0x80000000)) {
        sec += 1ULL << 32;      /* NTP era 1, after 2036-02-07 */
    }
    return (int64_t)(sec - NTP_UNIX_OFFSET) * 1000000 + (int64_t)((frac * 1000000) >> 32);
}

static size_t ef_put(uint8_t *p, uint16_t type, const uint8_t *body, size_t body_len)
{
    size_t len = 4 + pad4(body_len);
    memset(p, 0, len);
    be16enc(p, type);
    be16enc(p + 2, (uint16_t)len);
    if (body) {
        memcpy(p + 4, body, body_len);
    }
    return len;
}

/* Length of the extension field at off, or 0 if it is malformed or does not fit in the buffer */
static size_t ef_len_at(const uint8_t *buf, size_t len, size_t off)
{
    if (len - off < 4) {
        return 0;
    }
    const size_t ef_len = be16dec(buf + off + 2);
    return (ef_len < 4 || (ef_len & 3) || ef_len > len - off) ? 0 : ef_len;
}

/* Build the request, spending the cookie in the given slot */
static esp_err_t ntp_build_request(nts_session_t *s, int slot, uint8_t *pkt, uint8_t uid[UID_LEN], size_t *pkt_len)
{
    const size_t cookie_len = s->cookie_len[slot];
    const size_t cookie_ef_len = 4 + pad4(cookie_len);
    const size_t nonce_len = aead_nonce_len(s);
    const size_t auth_ef_len = 4 + 4 + pad4(nonce_len) + AEAD_TAG_LEN;

    memset(pkt, 0, NTP_HDR_LEN);
    pkt[0] = (0 << 6) | (4 << 3) | 3;   /* LI = 0, VN = 4, Mode = 3 (client) */
    size_t len = NTP_HDR_LEN;

    /* A random transmit timestamp avoids leaking the local clock; the server echoes it back */
    if (psa_generate_random(pkt + 40, 8) != PSA_SUCCESS ||
            psa_generate_random(uid, UID_LEN) != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    len += ef_put(pkt + len, EF_UNIQUE_ID, uid, UID_LEN);
    len += ef_put(pkt + len, EF_COOKIE, s->cookie[slot], cookie_len);
    s->cookie_len[slot] = 0;    /* a cookie must never be reused */

    /* Ask for one extra cookie for each one lost earlier, to refill the pool */
    int placeholders = NTS_MAX_COOKIES - nts_cookie_count(s) - 1;
    while (placeholders-- > 0 && len + cookie_ef_len + auth_ef_len <= NTP_MAX_PACKET) {
        len += ef_put(pkt + len, EF_COOKIE_PLACEHOLDER, NULL, cookie_len);
    }

    /* Authenticator: everything before it is the associated data, there is no plaintext */
    uint8_t *auth = pkt + len;
    be16enc(auth, EF_AUTHENTICATOR);
    be16enc(auth + 2, (uint16_t)auth_ef_len);
    be16enc(auth + 4, (uint16_t)nonce_len);
    be16enc(auth + 6, AEAD_TAG_LEN);
    if (psa_generate_random(auth + 8, nonce_len) != PSA_SUCCESS) {
        return ESP_FAIL;
    }

    *pkt_len = len + auth_ef_len;
    return aead_seal(s, pkt, len, auth + 8, NULL, 0, auth + 8 + pad4(nonce_len));
}

/*
 * Authenticate a response and collect the fresh cookies from its encrypted part.
 * Returns ESP_ERR_INVALID_MAC / ESP_ERR_INVALID_RESPONSE for packets to ignore.
 */
static esp_err_t ntp_verify_response(nts_session_t *s, uint8_t *pkt, size_t len,
                                     const uint8_t uid[UID_LEN], uint8_t *new_cookies)
{
    bool uid_ok = false;
    size_t off = NTP_HDR_LEN;

    for (;;) {
        const size_t ef_len = ef_len_at(pkt, len, off);
        if (!ef_len) {
            return ESP_ERR_INVALID_RESPONSE;    /* malformed, or no authenticator */
        }
        const uint16_t type = be16dec(pkt + off);
        if (type == EF_AUTHENTICATOR) {
            /* It must be the last field, anything after it would be unauthenticated */
            if (off + ef_len != len || ef_len < 8) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            break;
        }
        if (type == EF_UNIQUE_ID) {
            uid_ok = ef_len == 4 + UID_LEN && memcmp(pkt + off + 4, uid, UID_LEN) == 0;
        }
        off += ef_len;
    }
    if (!uid_ok) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t *auth = pkt + off;
    const size_t body_len = len - off - 4;
    const size_t nonce_len = be16dec(auth + 4);
    const size_t ct_len = be16dec(auth + 6);
    if (nonce_len == 0 || ct_len < AEAD_TAG_LEN || 4 + pad4(nonce_len) + pad4(ct_len) > body_len) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    const uint8_t *nonce = auth + 8;
    const size_t pt_len = ct_len - AEAD_TAG_LEN;
    uint8_t *pt = NULL;
    esp_err_t err = aead_open(s, pkt, off, nonce, nonce_len, pkt + off + 8 + pad4(nonce_len), ct_len, &pt);

    /* The plaintext must be a well-formed list of extension fields; only cookies are expected */
    *new_cookies = 0;
    for (size_t p = 0; err == ESP_OK && p < pt_len;) {
        const size_t ef_len = ef_len_at(pt, pt_len, p);
        if (!ef_len) {
            err = ESP_ERR_INVALID_RESPONSE;
            break;
        }
        p += ef_len;
    }
    for (size_t p = 0; err == ESP_OK && p < pt_len;) {
        size_t ef_len = be16dec(pt + p + 2);
        if (be16dec(pt + p) == EF_COOKIE && cookie_store(s, pt + p + 4, ef_len - 4)) {
            (*new_cookies)++;
        }
        p += ef_len;
    }
    return err;
}

static int ntp_open_socket(const char *host, uint16_t port)
{
    /* IPv4 first: an IPv6 address on the interface does not imply an IPv6 route to the server */
    static const int families[] = { AF_INET, AF_INET6 };
    char service[6];
    snprintf(service, sizeof(service), "%u", port);

    for (size_t i = 0; i < sizeof(families) / sizeof(families[0]); i++) {
        const struct addrinfo hints = { .ai_family = families[i], .ai_socktype = SOCK_DGRAM };
        struct addrinfo *res = NULL;
        if (getaddrinfo(host, service, &hints, &res) != 0 || !res) {
            continue;
        }
        int fd = socket(res->ai_family, res->ai_socktype, 0);
        /* connect() so that only datagrams from the server address are delivered */
        if (fd >= 0 && connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
            close(fd);
            fd = -1;
        }
        freeaddrinfo(res);
        if (fd >= 0) {
            return fd;
        }
    }
    ESP_LOGE(TAG, "cannot resolve or reach %s", host);
    return -1;
}

esp_err_t nts_ntp_query(nts_session_t *s, int timeout_ms, nts_time_t *result)
{
    ESP_RETURN_ON_FALSE(s && result && timeout_ms > 0, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    esp_err_t ret = ESP_OK;
    uint8_t uid[UID_LEN];
    uint8_t origin[8];
    bool nak = false;
    int fd = -1;
    memset(result, 0, sizeof(*result));

    const int slot = cookie_first(s);
    ESP_RETURN_ON_FALSE(slot >= 0, ESP_ERR_NOT_FOUND, TAG, "no cookies left");
    uint8_t *pkt = malloc(NTP_MAX_PACKET);
    ESP_RETURN_ON_FALSE(pkt, ESP_ERR_NO_MEM, TAG, "no memory for NTP packet");

    /* Resolve and open the socket first, so that a local failure does not cost a cookie */
    fd = ntp_open_socket(s->ntp_host, s->ntp_port);
    ESP_GOTO_ON_FALSE(fd >= 0, ESP_FAIL, out, TAG, "cannot open socket to %s:%u", s->ntp_host, s->ntp_port);

    size_t req_len = 0;
    ESP_GOTO_ON_ERROR(ntp_build_request(s, slot, pkt, uid, &req_len), out, TAG, "cannot build the request");
    memcpy(origin, pkt + 40, sizeof(origin));

    const int64_t t1_mono = esp_timer_get_time();
    ESP_GOTO_ON_FALSE(send(fd, pkt, req_len, 0) == (ssize_t)req_len, ESP_FAIL, out, TAG, "send failed: errno %d", errno);

    /* Wait for an authentic response, ignoring anything that does not verify */
    for (;;) {
        const int64_t left_us = t1_mono + (int64_t)timeout_ms * 1000 - esp_timer_get_time();
        if (left_us <= 0) {
            /* An NTS NAK is unauthenticated, so it only counts if nothing better arrived */
            ret = nak ? ESP_ERR_INVALID_STATE : ESP_ERR_TIMEOUT;
            goto out;
        }
        struct timeval tv = { .tv_sec = left_us / 1000000, .tv_usec = left_us % 1000000 };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        if (select(fd + 1, &rfds, NULL, NULL, &tv) == 0) {
            continue;   /* nothing arrived; the check above ends the wait */
        }

        ssize_t n = recv(fd, pkt, NTP_MAX_PACKET, MSG_DONTWAIT);
        const int64_t t4_mono = esp_timer_get_time();
        if (n < 0) {
            /*
             * For example an ICMP port unreachable. It is no more authentic than a forged
             * reply, so keep waiting for the real one, but yield so that this is not a busy loop.
             */
            vTaskDelay(1);
            continue;
        }
        /* Server mode, NTPv4, and the origin timestamp must echo our transmit timestamp */
        if (n < NTP_HDR_LEN || (pkt[0] & 0x07) != 4 || ((pkt[0] >> 3) & 0x07) != 4 ||
                memcmp(pkt + 24, origin, sizeof(origin)) != 0) {
            continue;
        }
        if (pkt[1] == 0 && memcmp(pkt + 12, "NTSN", 4) == 0) {
            nak = true;
            continue;
        }
        esp_err_t err = ntp_verify_response(s, pkt, n, uid, &result->new_cookies);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ignoring response that failed NTS verification (%s)", esp_err_to_name(err));
            continue;
        }

        result->stratum = pkt[1];
        if (pkt[1] == 0) {
            /* Authenticated kiss-o'-death: the reference ID is a four-character code */
            for (int i = 0; i < 4; i++) {
                const char c = (char)pkt[12 + i];
                result->kiss[i] = (c >= 0x20 && c < 0x7f) ? c : '?';
            }
            ESP_LOGW(TAG, "kiss-o'-death from server: %s", result->kiss);
            ret = ESP_ERR_INVALID_RESPONSE;
            goto out;
        }
        if ((pkt[0] >> 6) == 3) {
            ESP_LOGW(TAG, "server clock is not synchronised");
            ret = ESP_ERR_INVALID_RESPONSE;
            goto out;
        }

        /* T1 and T4 are on the local clock, T2 and T3 on the server clock */
        struct timeval now;
        gettimeofday(&now, NULL);
        const int64_t t4 = (int64_t)now.tv_sec * 1000000 + now.tv_usec - (esp_timer_get_time() - t4_mono);
        const int64_t t1 = t4 - (t4_mono - t1_mono);
        const int64_t t2 = ntp_ts_to_unix_us(pkt + 32);
        const int64_t t3 = ntp_ts_to_unix_us(pkt + 40);
        /* RFC 5905: a zero transmit timestamp is invalid, and the server cannot send before it receives */
        static const uint8_t zero_ts[8];
        if (memcmp(pkt + 40, zero_ts, sizeof(zero_ts)) == 0 || t3 < t2) {
            ESP_LOGW(TAG, "server timestamps are not usable");
            ret = ESP_ERR_INVALID_RESPONSE;
            goto out;
        }
        result->offset_us = ((t2 - t1) + (t3 - t4)) / 2;
        result->delay_us = (t4 - t1) - (t3 - t2);
        break;
    }

out:
    if (fd >= 0) {
        close(fd);
    }
    free(pkt);
    return ret;
}
