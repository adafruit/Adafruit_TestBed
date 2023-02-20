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
#include "stub_esp32.h"

#define DEBUG 1

#if DEBUG
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

#define VERIFY(_cond)                                                          \
  do {                                                                         \
    if (!(_cond)) {                                                            \
      Serial.printf("Failed at line %u", __LINE__);                            \
      Serial.flush();                                                          \
      while (1) {                                                              \
        delay(10);                                                             \
      }                                                                        \
      return false;                                                            \
    }                                                                          \
  } while (0)

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

  // flash write without stub
  FLASH_WRITE_SIZE_NOSTUB = 0x400,

  // flassh write with stub
  FLASH_WRITE_SIZE_STUB = 0x4000,

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

static inline uint32_t div_ceil(uint32_t v, uint32_t d) {
  return (v + d - 1) / d;
}

ESP32BootROMClass::ESP32BootROMClass(HardwareSerial &serial, int gpio0Pin,
                                     int resetnPin, bool supportsEncryptedFlash)
    : _serial(&serial), _gpio0Pin(gpio0Pin), _resetnPin(resetnPin),
      _supports_encrypted_flash(supportsEncryptedFlash), _stub_running(false) {}

int ESP32BootROMClass::begin(unsigned long baudrate) {
  _serial->begin(115200);

  pinMode(_gpio0Pin, OUTPUT);
  pinMode(_resetnPin, OUTPUT);

  digitalWrite(_gpio0Pin, LOW);
  digitalWrite(_resetnPin, LOW);
  delay(100);

  digitalWrite(_resetnPin, HIGH);
  // delay(50);

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

  bool need_stub = false;

  //------------- Chip Detect -------------//
  uint32_t regvalue;
  if (!read_reg(CHIP_DETECT_MAGIC_REG_ADDR, &regvalue)) {
    return 0;
  }

  switch (regvalue) {
  case CHIP_DETECT_MAGIC_ESP32:
    // newer module such as esp32 pico need stub to upload
    need_stub = true;
    Serial.println("Found ESP32");
    break;
  case CHIP_DETECT_MAGIC_ESP32S2:
    Serial.println("Found ESP32-S2");
    break;

  case CHIP_DETECT_MAGIC_ESP32S3:
    Serial.println("Found ESP32-S3");
    break;

  default:
    Serial.println("Found unknown ESP");
    break;
  }

#if 0
  uint8_t mac[6];
  read_MAC(mac);
  Serial.printf("MAC addr: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif

  if (need_stub) {
    VERIFY(uploadStub());
  }

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

  // usse default spi connection if no stub
  if (!need_stub) {
    while (!spiAttach()) {
      Serial.println("Failed to attach SPI");
      delay(100);
    }
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
    results[i] = response(ESP_SYNC, 100);
  }

  return (results[0] == 0);
}

int ESP32BootROMClass::changeBaudrate(unsigned long baudrate) {
  const uint32_t data[2] = {baudrate, 0};

  command(ESP_CHANGE_BAUDRATE, data, sizeof(data));
  return (response(ESP_CHANGE_BAUDRATE, 3000) == 0);
}

int ESP32BootROMClass::spiAttach() {
  const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  command(ESP_SPI_ATTACH, data, sizeof(data));

  return (response(ESP_SPI_ATTACH, 3000) == 0);
}

uint32_t ESP32BootROMClass::getFlashWriteSize(void) {
  return _stub_running ? FLASH_WRITE_SIZE_STUB : FLASH_WRITE_SIZE_NOSTUB;
}

int ESP32BootROMClass::beginFlash(uint32_t offset, uint32_t size,
                                  uint32_t chunkSize) {

  const uint32_t data[] = {size, div_ceil(size, chunkSize), chunkSize, offset};
  uint16_t const len = _supports_encrypted_flash ? 20 : 16;

  command(ESP_FLASH_BEGIN, data, len);

  _flashSequenceNumber = 0;
  _chunkSize = chunkSize;

  return (response(ESP_FLASH_BEGIN, 120000) == 0);
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

  return (response(ESP_FLASH_DATA, 3000) == 0);
}

int ESP32BootROMClass::endFlash(uint32_t reboot) {
  const uint32_t data[1] = {reboot};

  command(ESP_FLASH_END, data, sizeof(data));

  return (response(ESP_FLASH_END, 3000) == 0);
}

bool ESP32BootROMClass::md5Flash(uint32_t offset, uint32_t size,
                                 uint8_t *result) {
  const uint32_t data[4] = {offset, size, 0, 0};

  command(ESP_SPI_FLASH_MD5, data, sizeof(data));

  uint8_t resp[32];
  VERIFY(0 == response(ESP_SPI_FLASH_MD5, 3000, resp));

  // Note that the ESP32 ROM loader returns the md5sum as 32 hex encoded ASCII
  // bytes, whereas the stub loader returns the md5sum as 16 raw data bytes of
  // MD5 followed by 2 status bytes.
  if (_stub_running) {
    memcpy(result, resp, 16);
  }else {
    char temp[3] = {0, 0, 0};

    for (int i = 0; i < 16; i++) {
      temp[0] = resp[i * 2];
      temp[1] = resp[i * 2 + 1];

      result[i] = strtoul(temp, NULL, 16);
    }
  }

  return true;
}

bool ESP32BootROMClass::read_reg(uint32_t addr, uint32_t *val,
                                 uint32_t timeout_ms) {
  command(ESP_READ_REG, &addr, 4);

  if (0 == response(ESP_READ_REG, timeout_ms, val, 4)) {
    return true;
  } else {
    return false;
  }
}

uint32_t ESP32BootROMClass::read_chip_detect(void) {
  uint32_t chip_detect;
  if (!read_reg(CHIP_DETECT_MAGIC_REG_ADDR, &chip_detect)) {
    return 0;
  }

  return chip_detect;
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
  memcpy(mac, tmp_mac.bytes + 2, 6);
  return true;
}

bool ESP32BootROMClass::beginMem(uint32_t offset, uint32_t size,
                                 uint32_t chunkSize) {
  const uint32_t data[] = {size, div_ceil(size, chunkSize), chunkSize, offset};
  uint16_t const len = 16;
  command(ESP_MEM_BEGIN, data, len);

  _flashSequenceNumber = 0;
  _chunkSize = chunkSize;

  return (response(ESP_MEM_BEGIN, 120000) == 0);
}

bool ESP32BootROMClass::dataMem(const void *data, uint32_t length) {
  uint32_t cmd_len = 16 + length;
  uint32_t *cmdData = (uint32_t *)malloc(cmd_len);

  VERIFY(cmdData);

  cmdData[0] = length;
  cmdData[1] = _flashSequenceNumber++;
  cmdData[2] = 0;
  cmdData[3] = 0;

  memcpy(&cmdData[4], data, length);
  command(ESP_MEM_DATA, cmdData, cmd_len);

  free(cmdData);

  return (response(ESP_MEM_DATA, 3000) == 0);
}

bool ESP32BootROMClass::endMem(uint32_t entry) {
  uint32_t data[2];
  data[0] = (entry == 0);
  data[1] = entry;

  command(ESP_MEM_END, data, sizeof(data));

  return (response(ESP_MEM_END, 3000) == 0);
}

bool ESP32BootROMClass::syncStub(void) {
  // read OHAI packet
  uint8_t const ohai[6] = {0xc0, 0x4f, 0x48, 0x41, 0x49, 0xc0};
  uint8_t buf[6];
  uint8_t count = 0;

  uint32_t timeout_ms = 3000;
  uint32_t start = millis();
  while ((count < 6) && (millis() - start) < timeout_ms) {
    if (_serial->available()) {
      buf[count] = _serial->read();
      count++;
    }
  }

#if DEBUG
  if (count) {
    Serial.print("<= ");
    for (size_t i = 0; i < count; i++) {
      Serial.printf("%02x ", buf[i]);
    }
    Serial.println();
  }
#endif

  if (count == 6 && 0 == memcmp(ohai, buf, 6)) {
    Serial.println("Stub running...\r\n");
    return true;
  } else {
    Serial.println("Failed to start stub. Unexpected response");
    return false;
  }
}

bool ESP32BootROMClass::uploadStub(void) {
  Serial.println("Uploading stub...");

  uint32_t remain;
  const uint8_t *buf;

  // upload text
  Serial.println("Uploading stub text...");
  VERIFY(
      beginMem(stub_esp32_text_start, stub_esp32_text_length, ESP_RAM_BLOCK));

  remain = stub_esp32_text_length;
  buf = stub_esp32_text;
  while (remain) {
    uint32_t const len = (remain > ESP_RAM_BLOCK) ? ESP_RAM_BLOCK : remain;
    dataMem(buf, len);

    buf += len;
    remain -= len;
  }

  // upload data
  Serial.println("Uploading stub data...");
  VERIFY(
      beginMem(stub_esp32_data_start, stub_esp32_data_length, ESP_RAM_BLOCK));

  remain = stub_esp32_data_length;
  buf = stub_esp32_data;
  while (remain) {
    uint32_t const len = (remain > ESP_RAM_BLOCK) ? ESP_RAM_BLOCK : remain;
    dataMem(buf, len);

    buf += len;
    remain -= len;
  }

  // run stub
  Serial.println("Running stub...");
  VERIFY(endMem(stub_esp32_entry));

  // sync stub
  Serial.println("Syncing stub...");
  VERIFY(syncStub());

  _stub_running = true;

  return true;
}

//--------------------------------------------------------------------+
// Command & Response
//--------------------------------------------------------------------+

void ESP32BootROMClass::command(uint8_t opcode, const void *data,
                                uint16_t length) {
  uint32_t checksum = 0;

  if (opcode == ESP_FLASH_DATA || opcode == ESP_MEM_DATA) {
    checksum = ESP_CHECKSUM_MAGIC; // seed

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

// read until we found SLIP (0xC0) byte
bool ESP32BootROMClass::readSLIP(uint32_t timeout_ms) {
  uint8_t slip = 0;
  uint32_t start_ms = millis();
  uint32_t end_ms = start_ms + timeout_ms;
  while ((slip != 0xc0) && (end_ms - millis())) {
    readBytes(&slip, 1, end_ms - millis());
  }
  return slip == 0xc0;
}

// Read response bytes from ESP32, return number of received bytes
uint16_t ESP32BootROMClass::readBytes(void *buf, uint16_t length,
                                      uint32_t timeout_ms) {
  uint8_t *buf8 = (uint8_t *)buf;
  uint16_t count = 0;
  uint32_t start = millis();

  while ((count < length) && ((millis() - start) < timeout_ms)) {
    if (_serial->available()) {
      uint8_t ch = (uint8_t)_serial->read();

      // escape
      if ( ch == 0xdb ) {
        while(!_serial->available() && ((millis() - start) < timeout_ms) ) { yield(); }
        uint8_t ch2 = (uint8_t) _serial->read();

        if ( ch2 == 0xdb ) {
          ch = 0xdb;
        }else if ( ch2 == 0xdc ) {
          ch = 0xc0;
        }else {
          // should not reach here, must be an error or uart noise
          break;
        }
      }

      buf8[count++] = ch;
    }
  }

  return count;
}

// return response status if success, -1 if failed
int ESP32BootROMClass::response(uint8_t opcode, uint32_t timeout_ms, void *body,
                                uint16_t maxlen) {
  // Response Packet is composed of
  // - 1B: slip start
  // - 8B: fixed response ( struct below )
  // - nB: variable response payload ( if any )
  // - 2B: status, error
  // - 2B: reserved // ROM only, not stub
  // - 1B: slip end
  struct __packed_aligned {
    uint8_t dir; // 0x01 for response
    uint8_t opcode;
    uint16_t length; // at least 2 (or 4) for status bytes
    uint32_t
        reg_value; // Response value used by READ_REG command. Zero otherwise.
  } fixed_resp;
  uint8_t status[4] = { 0 };
  uint8_t const status_len = (_stub_running ? 2 : 4);

#if 1
  uint32_t start = millis();
  uint32_t end_ms = start + timeout_ms;

  readSLIP(timeout_ms);

  // read fixed response first
  if (8 != readBytes(&fixed_resp, 8, end_ms - millis())) {
    return -1; // probably timeout
  }

  // read variable payload if any
  uint16_t const payload_len = fixed_resp.length - status_len;
  uint8_t data[100];

  if (payload_len) {
    if (payload_len != readBytes(data, payload_len, end_ms - millis())) {
      return -1; // probably timeout
    }
  }

  if (body) {
    if (opcode == ESP_READ_REG) {
      memcpy(body, &fixed_resp.reg_value, 4);
    } else {
      memcpy(body, data, payload_len);
    }
  }

  // read status
  if (status_len != readBytes(status, status_len, end_ms - millis())) {
    return -1; // probably timeout
  }

  readSLIP(end_ms - millis());

#if DEBUG
  Serial.printf("<= c0 %02x %02x %04x %08x ", fixed_resp.dir, fixed_resp.opcode,
                fixed_resp.length, fixed_resp.reg_value);
  for (int i = 0; i < payload_len; i++) {
    Serial.printf("%02x ", data[i]);
  }
  for (int i = 0; i < status_len; i++) {
    Serial.printf("%02x ", status[i]);
  }
  Serial.println("c0");
  Serial.flush();
#endif

  // check direction, opcode and status
  if (fixed_resp.dir == 0x01 && fixed_resp.opcode == opcode &&
      status[0] == 0x00 && status[1] == 0x00) {
    return 0; // status[0];
  }

  Serial.printf("response failed status = %02x %02x!\r\n", status[0], status[1]);

  return -1;

#else 0
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
    // Serial.printf("index = %u\r\n", index);
    return -1;
  }

  if (data[0] != 0xc0 || data[1] != 0x01 || data[2] != opcode ||
      data[responseLength + 5] != 0x00 || data[responseLength + 6] != 0x00 ||
      data[responseLength + 9] != 0xc0) {
    Serial.printf("responseLength = %u\r\n", responseLength);
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
#endif
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
      if (print_payload)
        DBG_PRINTF("db db ");
#endif
    } else if (b == 0xc0) {
      _serial->write(0xdb);
      _serial->write(0xdc);

#if DEBUG
      if (print_payload)
        DBG_PRINTF("db dc ");
#endif
    } else {
      _serial->write(b);

#if DEBUG
      if (print_payload)
        DBG_PRINTF("%02x ", b);
#endif
    }
  }
}
