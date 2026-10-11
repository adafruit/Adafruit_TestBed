/*
  ESP32BootROM - part of the Firmware Updater for the
  Arduino MKR WiFi 1010, Arduino MKR Vidor 4000, and Arduino UNO WiFi Rev.2.

  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef ESP32_BOOTROM_H
#define ESP32_BOOTROM_H

#include <Arduino.h>

#define ESP32_DEFAULT_TIMEOUT 3000
#define ESP32_ERASE_TIMEOUT_PER_MB 40000 ///< ROM erase timeout per MiB, in ms.
#define ESP32_MD5_TIMEOUT_PER_MB 8000    ///< Flash MD5 timeout per MiB, in ms.

enum {
  CHIP_DETECT_MAGIC_ESP32 = 0x00F01D83,
  CHIP_DETECT_MAGIC_ESP32S2 = 0x000007C6,
  CHIP_DETECT_MAGIC_ESP32S3 = 0x9,
  CHIP_DETECT_MAGIC_ESP8266 = 0xFFF0C101,
  CHIP_DETECT_MAGIC_ESP32C6 = 0x2CE0806F,
  // P4 has no magic register: this is its GET_SECURITY_INFO chip ID.
  CHIP_DETECT_MAGIC_ESP32P4 = 18,
};

enum {
  ESP32P4_MAC_EFUSE_REG = 0x5012D044,      ///< P4 factory MAC address.
  ESP32P4_REVISION_EFUSE_REG = 0x5012D04C, ///< P4 silicon revision.
  ESP32P4_WDT_CONFIG_REG = 0x50116000,     ///< P4 RTC watchdog control.
  ESP32P4_WDT_PROTECT_REG = 0x50116018, ///< P4 RTC watchdog write protection.
  ESP32P4_SWD_CONFIG_REG = 0x5011601C,  ///< P4 super watchdog control.
  ESP32P4_SWD_PROTECT_REG = 0x50116020, ///< P4 super watchdog write protection.
  ESP32P4_WDT_UNLOCK_KEY = 0x50D83AA1,  ///< Watchdog write unlock key.
  ESP32P4_SWD_AUTO_FEED =
      0x00040000, ///< Automatically feed the super watchdog.
};

typedef struct {
  uint32_t chip_detect;
  uint32_t entry;
  uint32_t text_start;
  uint32_t text_length;
  const uint8_t *text;
  uint32_t data_start;
  uint32_t data_length;
  const uint8_t *data;
  const char *chip_name;
} esp32_stub_loader_t;

class ESP32BootROMClass {
public:
  ESP32BootROMClass(HardwareSerial &hwSerial, int gpio0Pin = -1,
                    int resetnPin = -1);

  // return chip detect magic if success, otherwise 0
  uint32_t begin(unsigned long baudrate, bool reset = true);
  void end();

  bool isRunningStub(void);
  uint32_t getFlashWriteSize(void);
  bool setFlashSize(uint32_t size);

  // High Level commands
  bool read_reg(uint32_t regAddr, uint32_t *regValue,
                uint32_t timeout_ms = ESP32_DEFAULT_TIMEOUT);
  bool read_MAC(uint8_t mac[6]);
  uint32_t read_chip_detect(void);

  // uncompressed
  bool beginFlash(uint32_t offset, uint32_t size, uint32_t chunkSize);
  bool dataFlash(const void *data, uint32_t length);
  bool endFlash(uint32_t reboot);

  // compressed
  bool beginFlashDefl(uint32_t offset, uint32_t size, uint32_t zip_size);
  bool dataFlashDefl(const void *data, uint32_t len);
  bool endFlashDefl(uint32_t reboot);

  bool md5Flash(uint32_t offset, uint32_t size, uint8_t *result);

private:
  void init();
  bool resetBootloader(void);
  bool prepareP4(void);
  bool write_reg(uint32_t regAddr, uint32_t value);
  bool sync();
  bool uploadStub(const esp32_stub_loader_t *stub);
  bool syncStub(uint32_t timeout_ms);

  // Command and Response
  void command(uint8_t opcode, const void *data, uint16_t length,
               const void *data2 = NULL, uint16_t len2 = 0);
  int response(uint8_t opcode, uint32_t timeout_ms, void *body = NULL,
               uint16_t body_size = 0);

  bool sendCommandGetResponse(uint8_t opcode, const void *data, uint16_t length,
                              const void *data2 = NULL, uint16_t len2 = 0,
                              uint32_t timeout_ms = ESP32_DEFAULT_TIMEOUT,
                              void *body = NULL, uint16_t body_size = 0);

  // High Level commands
  bool changeBaudrate(uint32_t baudrate);
  bool spiAttach();

  bool beginMem(uint32_t offset, uint32_t size, uint32_t chunkSize);
  bool dataMem(const void *data, uint32_t length);
  bool endMem(uint32_t entry);

  // read and write packets
  uint16_t readBytes(void *buf, uint16_t length, uint32_t timeout_ms);
  bool readSLIP(uint32_t timeout_ms);
  void writeEscapedBytes(const uint8_t *data, uint16_t length);

  HardwareSerial *_serial;
  int _gpio0Pin;
  int _resetnPin;

  bool _supports_encrypted_flash;
  bool _stub_running;
  bool _rom_8266_running;
  uint32_t _chip_detect;

  uint32_t _flashSequenceNumber;
};

#endif
