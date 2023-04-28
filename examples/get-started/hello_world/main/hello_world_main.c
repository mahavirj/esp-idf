/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "bootloader_random.h"
#include "soc/efuse_reg.h"
#include "esp_efuse_chip.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"

static const char *TAG = "Test RAM App";

static esp_err_t check_and_generate_flash_encryption_keys(void)
{
    size_t key_size = 32;
    enum { BLOCKS_NEEDED = 1 };
    esp_efuse_purpose_t purposes[BLOCKS_NEEDED] = {
        ESP_EFUSE_KEY_PURPOSE_FLASH_ENCRYPTION,
    };
    esp_efuse_coding_scheme_t coding_scheme = esp_efuse_get_coding_scheme(EFUSE_BLK_ENCRYPT_FLASH);
    if (coding_scheme != EFUSE_CODING_SCHEME_NONE && coding_scheme != EFUSE_CODING_SCHEME_3_4) {
        ESP_LOGE(TAG, "Unknown/unsupported CODING_SCHEME value 0x%x", coding_scheme);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (coding_scheme == EFUSE_CODING_SCHEME_3_4) {
        key_size = 24;
    }

    /* Initialize all efuse block entries to invalid (max) value */
    esp_efuse_block_t blocks[BLOCKS_NEEDED] = {[0 ... BLOCKS_NEEDED-1] = EFUSE_BLK_KEY_MAX};
    bool has_key = true;
    for (unsigned i = 0; i < BLOCKS_NEEDED; i++) {
        bool tmp_has_key = esp_efuse_find_purpose(purposes[i], &blocks[i]);
        if (tmp_has_key) { // For ESP32: esp_efuse_find_purpose() always returns True, need to check whether the key block is used or not.
            tmp_has_key &= !esp_efuse_key_block_unused(blocks[i]);
        }
        if (i == 1 && tmp_has_key != has_key) {
            ESP_LOGE(TAG, "Invalid efuse key blocks: Both AES-256 key blocks must be set.");
            return ESP_ERR_INVALID_STATE;
        }
        has_key &= tmp_has_key;
    }

    if (!has_key) {
        /* Generate key */
        uint8_t keys[BLOCKS_NEEDED][32] = { 0 };
        ESP_LOGI(TAG, "Generating new flash encryption key...");
        for (unsigned i = 0; i < BLOCKS_NEEDED; ++i) {
            bootloader_fill_random(keys[i], key_size);
        }
        ESP_LOGI(TAG, "Key generation complete");
        // Added the following print for debug purposes
        // It can be safely removed in production use-cases
        for (int i = 0; i < key_size; i++) {
            printf("%02X ", keys[0][i]);
            if ((i+1) % 8 == 0)
                printf("\n");
        }
        printf("\n");
        // Uncommenting the following line to disable key burning operation
        // Please note that uncommenting the following line and executing the firmware would burn one eFuse block with randomly generated flash encryption key
        esp_err_t err = ESP_FAIL;
        //err = esp_efuse_write_keys(purposes, keys, BLOCKS_NEEDED);
        if (err != ESP_OK) {
            if (err == ESP_ERR_NOT_ENOUGH_UNUSED_KEY_BLOCKS) {
                ESP_LOGE(TAG, "Not enough free efuse key blocks (need %d) to continue", BLOCKS_NEEDED);
            } else {
                ESP_LOGE(TAG, "Failed to write efuse block with purpose (err=0x%x). Can't continue.", err);
            }
            return err;
        }
    } else {
        for (unsigned i = 0; i < BLOCKS_NEEDED; i++) {
            if (!esp_efuse_get_key_dis_write(blocks[i])
                || !esp_efuse_get_key_dis_read(blocks[i])
                || !esp_efuse_get_keypurpose_dis_write(blocks[i])) { // For ESP32: no keypurpose, it returns always True.
                ESP_LOGE(TAG, "Invalid key state, check read&write protection for key and keypurpose(if exists)");
                return ESP_ERR_INVALID_STATE;
            }
        }
        ESP_LOGI(TAG, "Using pre-loaded flash encryption key in efuse");
    }
    return ESP_OK;
}

void app_main(void)
{
    // A delay of 1 sec is added in order to delay the logs
    // This is required if we are executing esptool load_ram and idf_monitor.py commands in succession
    // delay may be removed if not required

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    bootloader_random_enable();
    ESP_LOGI(TAG, "RAM lodable utility START");
    esp_err_t esp_ret = ESP_FAIL;
    esp_ret = check_and_generate_flash_encryption_keys();
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to burn flash encryption keys, error code = %04X", esp_ret);
        ESP_LOGE(TAG, "Status: Fail");
    } else {
        ESP_LOGI(TAG, "Status: Success");
    }
}
