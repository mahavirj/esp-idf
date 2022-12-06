/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <stdint.h>
#include "atca_status.h"

extern ATCA_STATUS __real_calib_get_pubkey(uint16_t key_id, uint8_t* public_key);

ATCA_STATUS __wrap_calib_get_pubkey(uint16_t key_id, uint8_t* public_key)
{
    printf("Authorizing...\n");
    //ATCA_STATUS result = authorizeDevice();
    //printf("Done with result %d\n", result);

    return __real_calib_get_pubkey(key_id, public_key);
}
