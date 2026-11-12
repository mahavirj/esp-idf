/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* NTS (Network Time Security, RFC 8915) authenticated time example */

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "aes_siv.h"
#include "aes_gcm_siv.h"
#include "nts_client.h"

static const char *TAG = "nts_example";

#define NTP_TIMEOUT_MS      3000
#define MAX_FAILURES        3       /* consecutive lost queries before a new key exchange */
#define REKEY_WAIT_MIN_S    10      /* back-off between key exchanges: first step ... */
#define REKEY_WAIT_MAX_S    3600    /* ... and cap */
#define POLL_MAX_S          1024    /* cap for the polling interval after "RATE" kiss codes */
#define MAX_HOSTS           8

static nts_session_t s_session;

/* The configured server list, split in place into host names */
static char s_hosts_buf[] = CONFIG_EXAMPLE_NTS_KE_HOSTS;
static const char *s_hosts[MAX_HOSTS];
static int s_host_count;

static void parse_hosts(void)
{
    char *save = NULL;
    for (char *h = strtok_r(s_hosts_buf, " ,", &save); h && s_host_count < MAX_HOSTS; h = strtok_r(NULL, " ,", &save)) {
        s_hosts[s_host_count++] = h;
    }
}

static void apply_offset(int64_t offset_us)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int64_t t = (int64_t)now.tv_sec * 1000000 + now.tv_usec + offset_us;
    struct timeval tv = { .tv_sec = t / 1000000, .tv_usec = t % 1000000 };
    settimeofday(&tv, NULL);
}

static void print_time(void)
{
    char buf[32];
    struct tm tm;
    time_t now = time(NULL);
    gmtime_r(&now, &tm);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    ESP_LOGI(TAG, "Authenticated time: %s UTC", buf);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (!nts_aes_siv_self_test() || !nts_aes_gcm_siv_self_test()) {
        ESP_LOGE(TAG, "AEAD self test failed");
        return;
    }
    ESP_LOGI(TAG, "AES-SIV and AES-GCM-SIV self tests passed");

    parse_hosts();
    if (s_host_count == 0) {
        ESP_LOGE(TAG, "no NTS-KE server configured");
        return;
    }

    ESP_ERROR_CHECK(example_connect());

    bool have_keys = false;
    int failures = 0;
    int rekey_wait_s = 0;
    int host = 0;
    int poll_s = CONFIG_EXAMPLE_NTS_POLL_INTERVAL_S;

    while (1) {
        if (!have_keys) {
            /*
             * Back off exponentially between key exchanges (RFC 8915 section 4.2). The first
             * one after a good query is immediate and goes to the same server; after that,
             * each attempt waits longer and moves to the next server in the list. The wait
             * is reset by the next good query.
             */
            if (rekey_wait_s) {
                host = (host + 1) % s_host_count;
                ESP_LOGW(TAG, "next NTS-KE in %d seconds", rekey_wait_s);
                vTaskDelay(pdMS_TO_TICKS(rekey_wait_s * 1000));
            }
            rekey_wait_s = rekey_wait_s ? MIN(rekey_wait_s * 2, REKEY_WAIT_MAX_S) : REKEY_WAIT_MIN_S;

            ESP_LOGI(TAG, "NTS-KE with %s:%d", s_hosts[host], CONFIG_EXAMPLE_NTS_KE_PORT);
            if (nts_ke_handshake(s_hosts[host], CONFIG_EXAMPLE_NTS_KE_PORT, &s_session) != ESP_OK) {
                ESP_LOGE(TAG, "NTS-KE failed");
                continue;
            }
            have_keys = true;
            failures = 0;
        }

        nts_time_t t;
        esp_err_t err = nts_ntp_query(&s_session, NTP_TIMEOUT_MS, &t);
        if (err == ESP_OK) {
            failures = 0;
            rekey_wait_s = 0;
            apply_offset(t.offset_us);
            ESP_LOGI(TAG, "offset %+lld us, delay %lld us, stratum %u, cookies %d (+%u)",
                     t.offset_us, t.delay_us, t.stratum, nts_cookie_count(&s_session), t.new_cookies);
            print_time();
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            /* Authenticated, but no time in it. The keys are fine, so do not run NTS-KE again. */
            if (strcmp(t.kiss, "DENY") == 0 || strcmp(t.kiss, "RSTR") == 0) {
                ESP_LOGE(TAG, "server refused service (%s), stopping", t.kiss);
                return;
            }
            if (strcmp(t.kiss, "RATE") == 0) {
                poll_s = MAX(poll_s, MIN(poll_s * 2, POLL_MAX_S));     /* never speed up */
                ESP_LOGW(TAG, "server asked to slow down, polling every %d seconds", poll_s);
            }
        } else {
            ESP_LOGW(TAG, "NTS query failed: %s", esp_err_to_name(err));
            /* The server may not send an NTS NAK, so repeated silence also means "start over" */
            if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_STATE || ++failures >= MAX_FAILURES) {
                have_keys = false;
                continue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_s * 1000));
    }
}
