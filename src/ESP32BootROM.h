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

enum {
  CHIP_DETECT_MAGIC_ESP32 = 0x00F01D83,
  CHIP_DETECT_MAGIC_ESP32S2 = 0x000007C6,
  CHIP_DETECT_MAGIC_ESP32S3 = 0x9,
};

typedef struct {
  uint32_t entry;
  uint32_t text_start;
  uint32_t text_length;
  const uint8_t *text;
  uint32_t data_start;
  uint32_t data_length;
  const uint8_t *data;
} esp32_stub_loader_t;

class ESP32BootROMClass {
public:
  ESP32BootROMClass(HardwareSerial &hwSerial, int gpio0Pin, int resetnPin);

  // return chip detect magic if success, otherwise 0
  uint32_t begin(unsigned long baudrate);
  void end();

  bool read_reg(uint32_t regAddr, uint32_t *regValue,
                uint32_t timeout_ms = ESP32_DEFAULT_TIMEOUT);
  bool read_MAC(uint8_t mac[6]);
  uint32_t read_chip_detect(void);

  bool isRunningStub(void);
  uint32_t getFlashWriteSize(void);

  // uncompressed
  int beginFlash(uint32_t offset, uint32_t size, uint32_t chunkSize);
  int dataFlash(const void *data, uint32_t length);
  int endFlash(uint32_t reboot);

  // compressed
  bool beginFlashDefl(uint32_t offset, uint32_t size, uint32_t zip_size);
  bool dataFlashDefl(const void *data, uint32_t len);
  bool endFlashDefl(uint32_t reboot);

  bool md5Flash(uint32_t offset, uint32_t size, uint8_t *result);

private:
  int sync();
  int changeBaudrate(uint32_t baudrate);
  int spiAttach();

  void command(uint8_t opcode, const void *data, uint16_t length,
               const void *data2 = NULL, uint16_t len2 = 0);
  int response(uint8_t opcode, uint32_t timeout_ms, void *body = NULL);

  uint16_t readBytes(void *buf, uint16_t length, uint32_t timeout_ms);
  bool readSLIP(uint32_t timeout_ms);
  void writeEscapedBytes(const uint8_t *data, uint16_t length);

  bool beginMem(uint32_t offset, uint32_t size, uint32_t chunkSize);
  bool dataMem(const void *data, uint32_t length);
  bool endMem(uint32_t entry);

  bool uploadStub(const esp32_stub_loader_t *stub);
  bool syncStub(uint32_t timeout_ms);

  void resetBootloader(void);

private:
  HardwareSerial *_serial;
  int _gpio0Pin;
  int _resetnPin;
  bool _supports_encrypted_flash;
  bool _stub_running;

  uint32_t _flashSequenceNumber;
};

#endif
