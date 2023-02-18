#if defined(RP2040)
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

#include <Arduino.h>

#define ESP32_DEFAULT_TIMEOUT 3000

class ESP32BootROMClass {
public:
  // supportsEncryptedFlash must be set to
  // - false for ESP32 (default)
  // - true for later chips such as S2, S3, H2, C3
  ESP32BootROMClass(HardwareSerial &hwSerial, int gpio0Pin, int resetnPin,
                    bool supportsEncryptedFlash = false);

  int begin(unsigned long baudrate);
  void end();

  int beginFlash(uint32_t offset, uint32_t size, uint32_t chunkSize);
  int dataFlash(const void *data, uint32_t length);
  int endFlash(uint32_t reboot);
  bool read_reg(uint32_t regAddr, uint32_t *regValue, uint32_t timeout_ms=ESP32_DEFAULT_TIMEOUT);
  int md5Flash(uint32_t offset, uint32_t size, uint8_t *result);

  bool read_MAC(uint32_t *machi, uint32_t *maclo);

private:
  int sync();
  int changeBaudrate(unsigned long baudrate);
  int spiAttach();

  void command(uint8_t opcode, const void *data, uint16_t length);
  int response(uint8_t opcode, uint32_t timeout_ms, void *body = NULL, uint16_t maxlen = 4);

  void writeEscapedBytes(const uint8_t *data, uint16_t length);

private:
  HardwareSerial *_serial;
  int _gpio0Pin;
  int _resetnPin;
  bool _supports_encrypted_flash;

  uint32_t _flashSequenceNumber;
  uint32_t _chunkSize;
};
#endif
