idf_component_register(SRCS "nts_example_main.c" "nts_client.c" "aes_siv.c" "aes_gcm_siv.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp-tls mbedtls lwip esp_timer esp_netif esp_event nvs_flash)
