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

#include "ESP32BootROM.h"

#define DEBUG 1

#if DEBUG
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

enum {
  // Commands supported by ESP8266 ROM bootloader
  ESP_FLASH_BEGIN = 0x02,
  ESP_FLASH_DATA = 0x03,
  ESP_FLASH_END = 0x04,
  ESP_MEM_BEGIN = 0x05,
  ESP_MEM_END = 0x06,
  ESP_MEM_DATA = 0x07,
  ESP_SYNC = 0x08,
  ESP_WRITE_REG = 0x09,
  ESP_READ_REG = 0x0A,

  // Some comands supported by ESP32 and later chips ROM bootloader (or -8266 w/
  // stub)
  ESP_SPI_SET_PARAMS = 0x0B,
  ESP_SPI_ATTACH = 0x0D,
  ESP_READ_FLASH_SLOW = 0x0E, // ROM only, much slower than the stub flash read
  ESP_CHANGE_BAUDRATE = 0x0F,
  ESP_FLASH_DEFL_BEGIN = 0x10,
  ESP_FLASH_DEFL_DATA = 0x11,
  ESP_FLASH_DEFL_END = 0x12,
  ESP_SPI_FLASH_MD5 = 0x13,

  // Commands supported by ESP32-S2 and later chips ROM bootloader only
  ESP_GET_SECURITY_INFO = 0x14,

  // Some commands supported by stub only
  ESP_ERASE_FLASH = 0xD0,
  ESP_ERASE_REGION = 0xD1,
  ESP_READ_FLASH = 0xD2,
  ESP_RUN_USER_CODE = 0xD3,

  // Flash encryption encrypted data command
  ESP_FLASH_ENCRYPT_DATA = 0xD4,

  // Response code(s) sent by ROM
  ROM_INVALID_RECV_MSG = 0x05, // response if an invalid message is received
};

enum {
  // Maximum block sized for RAM and Flash writes, respectively.
  ESP_RAM_BLOCK = 0x1800,

  FLASH_WRITE_SIZE = 0x400,

  // Default baudrate. The ROM auto-bauds, so we can use more or less whatever
  // we want.
  ESP_ROM_BAUD = 115200,

  // First byte of the application image
  ESP_IMAGE_MAGIC = 0xE9,

  // Initial state for the checksum routine
  ESP_CHECKSUM_MAGIC = 0xEF,

  // Flash sector size, minimum unit of erase.
  // FLASH_SECTOR_SIZE = 0x1000,

  UART_DATE_REG_ADDR = 0x60000078,

  // This ROM address has a different value on each chip model
  CHIP_DETECT_MAGIC_REG_ADDR = 0x40001000,

  UART_CLKDIV_MASK = 0xFFFFF,

  // Memory addresses
  IROM_MAP_START = 0x40200000,
  IROM_MAP_END = 0x40300000,

  // The number of bytes in the UART response that signify command status
  // STATUS_BYTES_LENGTH = 2,

  // Bootloader flashing offset
  // BOOTLOADER_FLASH_OFFSET = 0x0,

  // ROM supports an encrypted flashing mode
  // SUPPORTS_ENCRYPTED_FLASH = False,

  // Response to ESP_SYNC might indicate that flasher stub is running
  // instead of the ROM bootloader
  // sync_stub_detected = False,

  EFUSE_RD_REG_BASE = 0x3FF5A000,

  // Device PIDs
  USB_JTAG_SERIAL_PID = 0x1001
};

enum {
  CHIP_DETECT_MAGIC_ESP32 = 0x00F01D83,
  CHIP_DETECT_MAGIC_ESP32S2 = 0x000007C6,
  CHIP_DETECT_MAGIC_ESP32S3 = 0x9,
};

ESP32BootROMClass::ESP32BootROMClass(HardwareSerial &serial, int gpio0Pin,
                                     int resetnPin, bool supportsEncryptedFlash)
    : _serial(&serial), _gpio0Pin(gpio0Pin), _resetnPin(resetnPin),
      _supports_encrypted_flash(supportsEncryptedFlash) {}

int ESP32BootROMClass::begin(unsigned long baudrate) {
  _serial->begin(115200);

  pinMode(_gpio0Pin, OUTPUT);
  pinMode(_resetnPin, OUTPUT);

  digitalWrite(_gpio0Pin, LOW);
  digitalWrite(_resetnPin, LOW);
  delay(100);

  digitalWrite(_resetnPin, HIGH);
  //delay(50);

  // Wait for serial, needed if using with SerialHost (host cdc)
  while (!_serial) {
    delay(10);
  }

  delay(50); // additional delay for SerialHost connected
  digitalWrite(_gpio0Pin, HIGH);

  int synced = 0;

  for (int retries = 0; !synced && (retries < 10); retries++) {
    Serial.println("Trying to sync");
    synced = sync();
  }
  if (!synced) {
    return 0;
  }

  Serial.println("Synced!");
  
  uint32_t regvalue;
  if (!read_reg(CHIP_DETECT_MAGIC_REG_ADDR, &regvalue)) {
    return 0;
  }
  switch (regvalue)
    {
    case 0x00F01D83:
      Serial.println("Found ESP32");
      break;
    default:
      Serial.println("Found unknown ESP");
    }

  uint8_t mac[6];
  read_MAC(mac);
  Serial.printf("MAC addr: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (baudrate != 115200) {
    if (!changeBaudrate(baudrate)) {
      Serial.print("Failed to change baudrate");
      return 0;
    }
    // _serial->end();
    delay(100);
    Serial.println("Updating local Serial baudrate");
    _serial->begin(baudrate);
  }

  while (!spiAttach()) {
    Serial.println("Failed to attach SPI");
    delay(100);
  }

  return 1;
}

void ESP32BootROMClass::end() {
  //_serial->end();
}

int ESP32BootROMClass::sync() {
  const uint8_t data[] = {0x07, 0x07, 0x12, 0x20, 0x55, 0x55, 0x55, 0x55, 0x55,
                          0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                          0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                          0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

  command(ESP_SYNC, data, sizeof(data));

  int results[8];

  for (int i = 0; i < 8; i++) {
    results[i] = response(0x08, 100);
  }

  return (results[0] == 0);
}

int ESP32BootROMClass::changeBaudrate(unsigned long baudrate) {
  const uint32_t data[2] = {baudrate, 0};

  command(ESP_CHANGE_BAUDRATE, data, sizeof(data));

  return (response(0x0f, 3000) == 0);
}

int ESP32BootROMClass::spiAttach() {
  const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  command(ESP_SPI_ATTACH, data, sizeof(data));

  return (response(0x0d, 3000) == 0);
}

int ESP32BootROMClass::beginFlash(uint32_t offset, uint32_t size,
                                  uint32_t chunkSize) {

  const uint32_t data[] = {size, size / chunkSize, chunkSize, offset, 0};
  uint16_t const len = _supports_encrypted_flash ? 20 : 16;

  command(ESP_FLASH_BEGIN, data, len);

  _flashSequenceNumber = 0;
  _chunkSize = chunkSize;

  return (response(0x02, 120000) == 0);
}

int ESP32BootROMClass::dataFlash(const void *data, uint32_t length) {
  uint32_t cmdData[4 + (_chunkSize / 4)];

  cmdData[0] = length;
  cmdData[1] = _flashSequenceNumber++;
  cmdData[2] = 0;
  cmdData[3] = 0;

  memcpy(&cmdData[4], data, length);

  if (length < _chunkSize) {
    memset(&cmdData[4 + (length / 4)], 0xff, _chunkSize - length);
  }

  command(ESP_FLASH_DATA, cmdData, sizeof(cmdData));

  return (response(0x03, 3000) == 0);
}

int ESP32BootROMClass::endFlash(uint32_t reboot) {
  const uint32_t data[1] = {reboot};

  command(ESP_FLASH_END, data, sizeof(data));

  return (response(0x04, 3000) == 0);
}

int ESP32BootROMClass::md5Flash(uint32_t offset, uint32_t size,
                                uint8_t *result) {
  const uint32_t data[4] = {offset, size, 0, 0};

  command(ESP_SPI_FLASH_MD5, data, sizeof(data));

  uint8_t asciiResult[32];

  if (response(0x13, 3000, asciiResult) != 0) {
    return 0;
  }

  char temp[3] = {0, 0, 0};

  for (int i = 0; i < 16; i++) {
    temp[0] = asciiResult[i * 2];
    temp[1] = asciiResult[i * 2 + 1];

    result[i] = strtoul(temp, NULL, 16);
  }

  return 1;
}

bool ESP32BootROMClass::read_reg(uint32_t addr, uint32_t *val, uint32_t timeout_ms) {
  const uint8_t data[4] = {(uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16),  (uint8_t)(addr >> 24)};
  command(ESP_READ_REG, data, sizeof(data));

  uint8_t reply[4];
  response(ESP_READ_REG, timeout_ms, &reply, sizeof(reply));
  *val = reply[3];
  *val <<= 8;
  *val |= reply[2];
  *val <<= 8;
  *val |= reply[1];
  *val <<= 8;
  *val |= reply[0];

  Serial.printf("Read register 0x%08x : 0x%08x\n\r", addr, *val);
  return true;
}

bool ESP32BootROMClass::read_MAC(uint8_t mac[6]) {
  union {
      uint8_t bytes[8];
      uint32_t words[2];
  } tmp_mac;

  if (!read_reg(EFUSE_RD_REG_BASE + 4, &tmp_mac.words[1])) {
    return false;
  }
  if (!read_reg(EFUSE_RD_REG_BASE + 8, &tmp_mac.words[0])) {
    return false;
  }

  // swap endian
  tmp_mac.words[0] = __builtin_bswap32(tmp_mac.words[0]);
  tmp_mac.words[1] = __builtin_bswap32(tmp_mac.words[1]);

  // [0] is highest byte
  memcpy(mac, tmp_mac.bytes+2, 6);
  return true;
}






/***************/

void ESP32BootROMClass::command(uint8_t opcode, const void *data, uint16_t length) {
  uint32_t checksum = 0;

  if (opcode == ESP_FLASH_DATA) {
    checksum = 0xef; // seed

    for (uint16_t i = 16; i < length; i++) {
      checksum ^= ((const uint8_t *)data)[i];
    }
  }

  DBG_PRINTF("=> c0 00 %02x %02x %02x ", opcode, length & 0x00ff, length >> 8);

  _serial->write(0xc0);
  _serial->write((uint8_t)0x00); // direction
  _serial->write(opcode);
  _serial->write((uint8_t *)&length, sizeof(length));
  writeEscapedBytes((uint8_t *)&checksum, sizeof(checksum));
  writeEscapedBytes((uint8_t *)data, length);
  _serial->write(0xc0);
  _serial->flush();

  DBG_PRINTF("c0\r\n");
}

int ESP32BootROMClass::response(uint8_t opcode, uint32_t timeout_ms, void *body, uint16_t maxlen) {
  uint8_t data[10 + 256];
  uint16_t index = 0;

  uint8_t responseLength = 4;

  unsigned long start = millis();
  while ((millis() - start) < timeout_ms) {
    if (_serial->available()) {
      data[index] = _serial->read();

      if (index == 3) {
        responseLength = data[index];
      }

      index++;
    }
    if (index >= (uint16_t)(10 + responseLength)) {
      break;
    }
  }

#if DEBUG
  if (index) {
    Serial.print("<= ");
    for (int i = 0; i < index; i++) {
      Serial.printf("%02x ", data[i]);
    }
    Serial.println();
  }
#endif

  if (index != (uint16_t)(10 + responseLength)) {
    return -1;
  }

  if (data[0] != 0xc0 || data[1] != 0x01 || data[2] != opcode ||
      data[responseLength + 5] != 0x00 || data[responseLength + 6] != 0x00 ||
      data[responseLength + 9] != 0xc0) {
    return -1;
  }

  if (body) {
    if (opcode == ESP_READ_REG) {
      memcpy(body, &data[5], maxlen);
    } else {
      memcpy(body, &data[9], responseLength - 4);
    }
  }

  return data[responseLength + 5];
}

void ESP32BootROMClass::writeEscapedBytes(const uint8_t *data,
                                          uint16_t length) {
  uint16_t written = 0;

#if DEBUG
  // skip flashing data since it is a lot to print
  bool const print_payload = (length > 1024) ? false : true;
#endif

  while (written < length) {
    uint8_t b = data[written++];

    if (b == 0xdb) {
      _serial->write(0xdb);
      _serial->write(0xdd);

      #if DEBUG
      if (print_payload) DBG_PRINTF("db db ");
      #endif
    } else if (b == 0xc0) {
      _serial->write(0xdb);
      _serial->write(0xdc);

      #if DEBUG
      if (print_payload) DBG_PRINTF("db dc ");
      #endif
    } else {
      _serial->write(b);

      #if DEBUG
      if (print_payload) DBG_PRINTF("%02x ", b);
      #endif
    }
  }
}
