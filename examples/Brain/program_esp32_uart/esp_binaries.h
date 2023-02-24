/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ESP_BINARIES_H_
#define ESP_BINARIES_H_

#define BIN_ESP32_NINA_1_7_4 0 // nina 1.7.4
#define BIN_ESP32_WIFI_AP_SKETCH                                               \
  1 // esp32 wifi accesspoint sketch with ssdi "YourAP"

#define BIN_FEATHER_S2 10 // Feather esp32s2 factory firmware
#define BIN_FEATHER_S3 11 // Feather esp32s3 factory firmware
#define BIN_METRO_S2 12   // Metro esp32s2 factory firmware

#define BIN_DEVKIT_S2 20 // Espressif s2 devkit
#define BIN_DEVKIT_S3 21 // Espressif s3 devkit

// select which bins to flash
#define BIN_FILES BIN_METRO_S2

#if BIN_FILES == BIN_ESP32_WIFI_AP_SKETCH
#include "esp_binaries/wifi_ap_binaries.h"
#elif BIN_FILES == BIN_NINA_1_7_4
#include "esp_binaries/nina_1_7_4_binaries.h"
#elif BIN_FILES == BIN_FEATHER_S2
#include "esp_binaries/feather_esp32s2_binaries.h"
#elif BIN_FILES == BIN_METRO_S2
#include "esp_binaries/metro_esp32s2_binaries.h"
#elif BIN_FILES == BIN_FEATHER_S3
#include "esp_binaries/feather_esp32s3_binaries.h"
#elif BIN_FILES == BIN_DEVKIT_S2
#include "esp_binaries/esp32s2_devkit_binaries.h"
#elif BIN_FILES == BIN_DEVKIT_S3
#include "esp_binaries/esp32s3_devkit_binaries.h"
#endif

struct {
  uint32_t addr;
  esp32_zipfile_t const *zfile;
} bin_files[] = {
#if BIN_FILES == BIN_NINA_1_7_4
    {0x00000, &NINA_W102_1_7_4},

#elif BIN_FILES == BIN_FEATHER_S2
    {0x1000, &esp32s2_feather_test_ino_bootloader},
    {0x8000, &esp32s2_feather_test_ino_partitions},
    {0xe000, &boot_app0},
    {0x10000, &esp32s2_feather_test_ino},
    {0x2d0000, &tinyuf2},

#elif BIN_FILES == BIN_METRO_S2
    {0x1000, &selftest_ino_bootloader},
    {0x8000, &selftest_ino_partitions},
    {0xe000, &boot_app0},
    {0x10000, &selftest_ino},
    {0x2d0000, &tinyuf2},

#elif BIN_FILES == BIN_FEATHER_S3
    {0x0000, &esp32s3_feather_test_ino_bootloader},
    {0x8000, &esp32s3_feather_test_ino_partitions},
    {0xe000, &boot_app0},
    {0x10000, &esp32s3_feather_test_ino},
    {0x2d0000, &tinyuf2},

#elif BIN_FILES == BIN_DEVKIT_S2
    {0x1000, &Blink_ino_bootloader},
    {0x8000, &Blink_ino_partitions},
    {0xe000, &boot_app0},
    {0x10000, &Blink_ino},

#elif BIN_FILES == BIN_DEVKIT_S3
    {0x0000, &Blink_ino_bootloader},
    {0x8000, &Blink_ino_partitions},
    {0xe000, &boot_app0},
    {0x10000, &Blink_ino},
#endif
};

enum { BIN_FILES_COUNT = sizeof(bin_files) / sizeof(bin_files[0]) };

#endif
