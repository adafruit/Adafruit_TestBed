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

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "ESP32BootROM.h"
#include "stub_esp32.h"

#define DEBUG 0

#if DEBUG
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)

#define DBG_PRINT_BUF(_buf, _len)                                              \
  do {                                                                         \
    for (int _i = 0; _i < _len; _i++)                                          \
      Serial.printf("%02x ", (_buf)[_i]);                                      \
  } while (0)

#else
#define DBG_PRINTF(...)
#define DBG_PRINT_BUF(_buf, _len)
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

  // Some commands supported by ESP32 and later chips ROM bootloader (or -8266
  // with stub)
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

static inline uint32_t div_ceil(uint32_t v, uint32_t d) {
  return (v + d - 1) / d;
}

ESP32BootROMClass::ESP32BootROMClass(HardwareSerial &serial, int gpio0Pin,
                                     int resetnPin)
    : _serial(&serial) {
  _gpio0Pin = gpio0Pin;
  _resetnPin = resetnPin;

  _supports_encrypted_flash = true;
  _stub_running = false;

  _flashSequenceNumber = 0;
}

void ESP32BootROMClass::resetBootloader(void) {
  // Reset Low ( ESP32 in reset)
  digitalWrite(_gpio0Pin, LOW);
  digitalWrite(_resetnPin, LOW);
  delay(100);

  // IO0 Low, Reset HIGH (ESP32 out of reset)
  digitalWrite(_resetnPin, HIGH);

  // Wait for serial, needed if using with SerialHost
  while (!_serial) {
    delay(10);
  }
  delay(50); // additional delay for SerialHost connected

  // IO0 high: done
  digitalWrite(_gpio0Pin, HIGH);
}

uint32_t ESP32BootROMClass::begin(unsigned long baudrate) {
  _serial->begin(ESP_ROM_BAUD);

  pinMode(_gpio0Pin, OUTPUT);
  pinMode(_resetnPin, OUTPUT);

  resetBootloader();

  int synced = 0;

  for (int retries = 0; !synced && (retries < 10); retries++) {
    Serial.println("Trying to sync");
    synced = sync();
  }
  if (!synced) {
    return 0;
  }

  Serial.println("Synced!");

  //------------- Chip Detect -------------//
  uint32_t chip_detect = read_chip_detect();
  if (!chip_detect) {
    return 0;
  }

  const esp32_stub_loader_t *stub = NULL;
  switch (chip_detect) {
  case CHIP_DETECT_MAGIC_ESP32:
    // only ESP32 have SUPPORTS_ENCRYPTED_FLASH = false
    stub = &stub_esp32;
    _supports_encrypted_flash = false;
    Serial.println("Found ESP32");
    break;
  case CHIP_DETECT_MAGIC_ESP32S2:
    stub = &stub_esp32s2;
    Serial.println("Found ESP32-S2");
    break;

  case CHIP_DETECT_MAGIC_ESP32S3:
    stub = &stub_esp32s3;
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

  if (stub) {
    VERIFY(uploadStub(stub));
    VERIFY(syncStub(3000));
  }

  if (baudrate != ESP_ROM_BAUD) {
    if (!changeBaudrate(baudrate)) {
      Serial.print("Failed to change baudrate");
      return 0;
    }
    // _serial->end();
    delay(100);
    Serial.println("Updating local Serial baudrate");
    _serial->begin(baudrate);
  }

  // use default spi connection if no stub
  if (!stub) {
    while (!spiAttach()) {
      Serial.println("Failed to attach SPI");
      delay(100);
    }
  }

  return chip_detect;
}

void ESP32BootROMClass::end() {
  digitalWrite(_gpio0Pin, HIGH);
  digitalWrite(_resetnPin, HIGH);
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

int ESP32BootROMClass::changeBaudrate(uint32_t baudrate) {
  // Two 32-bit words:
  // - new baud rate, and
  // - 0 if we are talking to the ROM loader or the current/old baud rate if we
  // are talking to the stub loader.
  uint32_t data[2] = {baudrate, 0};

  if (_stub_running) {
    data[1] = ESP_ROM_BAUD; // we only changed from 115200 to higher baud
  }

  command(ESP_CHANGE_BAUDRATE, data, sizeof(data));
  return (response(ESP_CHANGE_BAUDRATE, 3000) == 0);
}

int ESP32BootROMClass::spiAttach() {
  const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  command(ESP_SPI_ATTACH, data, sizeof(data));

  return (response(ESP_SPI_ATTACH, 3000) == 0);
}

bool ESP32BootROMClass::isRunningStub(void) { return _stub_running; }

uint32_t ESP32BootROMClass::getFlashWriteSize(void) {
  return _stub_running ? FLASH_WRITE_SIZE_STUB : FLASH_WRITE_SIZE_NOSTUB;
}

//--------------------------------------------------------------------+
// Uncompressed Flashing
//--------------------------------------------------------------------+
int ESP32BootROMClass::beginFlash(uint32_t offset, uint32_t size,
                                  uint32_t chunkSize) {

  const uint32_t data[5] = {size, div_ceil(size, chunkSize), chunkSize, offset,
                            0};
  uint16_t const len = (_supports_encrypted_flash && !_stub_running) ? 20 : 16;

  command(ESP_FLASH_BEGIN, data, len);

  _flashSequenceNumber = 0;

  return (response(ESP_FLASH_BEGIN, 120000) == 0);
}

int ESP32BootROMClass::dataFlash(const void *data, uint32_t length) {
  uint32_t header[4];

  header[0] = length;
  header[1] = _flashSequenceNumber++;
  header[2] = 0;
  header[3] = 0;

  command(ESP_FLASH_DATA, header, sizeof(header), data, length);

  return (response(ESP_FLASH_DATA, 3000) == 0);
}

int ESP32BootROMClass::endFlash(uint32_t reboot) {
  const uint32_t data[1] = {reboot};

  command(ESP_FLASH_END, data, sizeof(data));

  return (response(ESP_FLASH_END, 3000) == 0);
}

//--------------------------------------------------------------------+
// Compressed (Deflated) Flashing
//--------------------------------------------------------------------+

bool ESP32BootROMClass::beginFlashDefl(uint32_t offset, uint32_t size,
                                       uint32_t zip_size) {
  const uint32_t block_size = getFlashWriteSize();
  uint32_t data[5] = {0, div_ceil(zip_size, block_size), block_size, offset, 0};

  if (_stub_running) {
    // stub expects number of bytes here, manages erasing internally
    data[0] = size;
  } else {
    // ROM expects rounded up to erase block size
    data[0] = div_ceil(size, block_size) * block_size;
  }

  uint16_t const len = (_supports_encrypted_flash && !_stub_running) ? 20 : 16;

  command(ESP_FLASH_DEFL_BEGIN, data, len);

  _flashSequenceNumber = 0;

  return (response(ESP_FLASH_DEFL_BEGIN, 3000) == 0);
}

bool ESP32BootROMClass::dataFlashDefl(const void *data, uint32_t len) {
  uint32_t header[4];
  uint32_t stamp = millis();
  header[0] = len;
  header[1] = _flashSequenceNumber++;
  header[2] = 0;
  header[3] = 0;

  command(ESP_FLASH_DEFL_DATA, header, sizeof(header), data, len);
  DBG_PRINTF("FLASH_DEFL_DATA...%d", millis() - stamp);

  bool b = response(ESP_FLASH_DEFL_DATA, 3000);
  DBG_PRINTF(": %d\t", millis() - stamp);

  return (b == 0);
}

bool ESP32BootROMClass::endFlashDefl(uint32_t reboot) {
  const uint32_t data[1] = {reboot};

  command(ESP_FLASH_DEFL_END, data, sizeof(data));

  return (response(ESP_FLASH_DEFL_END, 3000) == 0);
}

bool ESP32BootROMClass::md5Flash(uint32_t offset, uint32_t size,
                                 uint8_t *result) {
  const uint32_t data[4] = {offset, size, 0, 0};

  command(ESP_SPI_FLASH_MD5, data, sizeof(data));

  uint8_t resp[32];
  VERIFY(0 == response(ESP_SPI_FLASH_MD5, 6000, resp));

  // Note that the ESP32 ROM loader returns the md5sum as 32 hex encoded ASCII
  // bytes, whereas the stub loader returns the md5sum as 16 raw data bytes of
  // MD5 followed by 2 status bytes.
  if (_stub_running) {
    memcpy(result, resp, 16);
  } else {
    char temp[3] = {0, 0, 0};

    for (int i = 0; i < 16; i++) {
      temp[0] = resp[i * 2];
      temp[1] = resp[i * 2 + 1];

      result[i] = strtoul(temp, NULL, 16);
    }
  }

  return true;
}

//--------------------------------------------------------------------+
// Read REG
//--------------------------------------------------------------------+

bool ESP32BootROMClass::read_reg(uint32_t addr, uint32_t *val,
                                 uint32_t timeout_ms) {
  command(ESP_READ_REG, &addr, 4);

  if (0 == response(ESP_READ_REG, timeout_ms, val)) {
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

//--------------------------------------------------------------------+
// Stub
//--------------------------------------------------------------------+

bool ESP32BootROMClass::beginMem(uint32_t offset, uint32_t size,
                                 uint32_t chunkSize) {
  const uint32_t data[] = {size, div_ceil(size, chunkSize), chunkSize, offset};
  uint16_t const len = 16;
  command(ESP_MEM_BEGIN, data, len);

  _flashSequenceNumber = 0;

  return (response(ESP_MEM_BEGIN, 120000) == 0);
}

bool ESP32BootROMClass::dataMem(const void *data, uint32_t length) {
  uint32_t header[4];
  header[0] = length;
  header[1] = _flashSequenceNumber++;
  header[2] = 0;
  header[3] = 0;

  command(ESP_MEM_DATA, header, sizeof(header), data, length);

  return (response(ESP_MEM_DATA, 3000) == 0);
}

bool ESP32BootROMClass::endMem(uint32_t entry) {
  uint32_t data[2];
  data[0] = (entry == 0);
  data[1] = entry;

  command(ESP_MEM_END, data, sizeof(data));

  return (response(ESP_MEM_END, 3000) == 0);
}

bool ESP32BootROMClass::syncStub(uint32_t timeout_ms) {
  // read OHAI packet
  uint8_t const ohai[4] = {0x4f, 0x48, 0x41, 0x49};
  uint8_t buf[4];

  Serial.println("Syncing stub...");

  if (!readSLIP(timeout_ms)) {
    return -1;
  }

  if (4 != readBytes(buf, 4, timeout_ms)) {
    return -1;
  }

  if (!readSLIP(timeout_ms)) {
    return -1;
  }

  if (0 == memcmp(ohai, buf, 4)) {
    Serial.println("Stub running...\r\n");
    return true;
  } else {
    Serial.println("Failed to start stub. Unexpected response");
    return false;
  }
}

bool ESP32BootROMClass::uploadStub(const esp32_stub_loader_t *stub) {
  Serial.println("Uploading stub...");

  uint32_t remain;
  const uint8_t *buf;

  // upload text
  Serial.println("Uploading stub text...");
  VERIFY(beginMem(stub->text_start, stub->text_length, ESP_RAM_BLOCK));

  remain = stub->text_length;
  buf = stub->text;
  while (remain) {
    uint32_t const len =
        (remain > ESP_RAM_BLOCK) ? (uint32_t)ESP_RAM_BLOCK : remain;
    dataMem(buf, len);

    buf += len;
    remain -= len;
  }

  // upload data
  Serial.println("Uploading stub data...");
  VERIFY(beginMem(stub->data_start, stub->data_length, ESP_RAM_BLOCK));

  remain = stub->data_length;
  buf = stub->data;
  while (remain) {
    uint32_t const len =
        (remain > ESP_RAM_BLOCK) ? (uint32_t)ESP_RAM_BLOCK : remain;
    dataMem(buf, len);

    buf += len;
    remain -= len;
  }

  // run stub
  Serial.println("Running stub...");
  VERIFY(endMem(stub->entry));

  _stub_running = true;

  return true;
}

//--------------------------------------------------------------------+
// Command & Response
//--------------------------------------------------------------------+

void ESP32BootROMClass::command(uint8_t opcode, const void *data, uint16_t len,
                                const void *data2, uint16_t len2) {
  uint32_t checksum = 0;

  // for FLASH_DATA and MEM_DATA: data is header, data2 is actual payload
  if (opcode == ESP_FLASH_DATA || opcode == ESP_MEM_DATA ||
      opcode == ESP_FLASH_DEFL_DATA) {
    checksum = ESP_CHECKSUM_MAGIC; // seed

    for (uint16_t i = 0; i < len2; i++) {
      checksum ^= ((const uint8_t *)data2)[i];
    }
  }

  uint16_t const total_len = len + len2;

  DBG_PRINTF("=> c0 00 %02x %04x ", opcode, total_len);

  uint8_t const header[3] = {0xc0, 0x00, opcode};

  _serial->write(header, 3);
  _serial->write((uint8_t *)&total_len, 2);

  writeEscapedBytes((uint8_t *)&checksum, sizeof(checksum));
  writeEscapedBytes((uint8_t *)data, len);
  if (data2 && len2) {
    writeEscapedBytes((uint8_t *)data2, len2);
  }

  _serial->write(0xc0);
  _serial->flush();

  DBG_PRINTF("c0\r\n");
}

// read until we found SLIP (0xC0) byte
bool ESP32BootROMClass::readSLIP(uint32_t timeout_ms) {
  uint32_t const end_ms = millis() + timeout_ms;
  while (millis() < end_ms) {
    if (_serial->available()) {
      uint8_t ch = (uint8_t)_serial->read();
      if (ch == 0xc0) {
        return true;
      }
    }
  }
  return false;
}

// Read response bytes from ESP32, return number of received bytes
uint16_t ESP32BootROMClass::readBytes(void *buf, uint16_t length,
                                      uint32_t timeout_ms) {
  uint8_t *buf8 = (uint8_t *)buf;
  uint16_t count = 0;
  uint32_t end_ms = millis() + timeout_ms;

  while ((count < length) && (millis() < end_ms)) {
    if (_serial->available()) {
      uint8_t ch = (uint8_t)_serial->read();

      // escape
      if (ch == 0xdb) {
        while (!_serial->available() && (millis() < end_ms)) {
          yield();
        }
        uint8_t ch2 = (uint8_t)_serial->read();

        if (ch2 == 0xdd) {
          ch = 0xdb;
        } else if (ch2 == 0xdc) {
          ch = 0xc0;
        } else {
          // should not reach here, must be an error or uart noise
          Serial.printf("err %02x ", ch2);
          break;
        }
      }

      buf8[count++] = ch;
    }
  }

  return count;
}

// return response status if success, -1 if failed
int ESP32BootROMClass::response(uint8_t opcode, uint32_t timeout_ms,
                                void *body) {
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
    uint16_t length;    // at least 2 (or 4) for status bytes
    uint32_t reg_value; // READ_REG response. zero otherwise
  } fixed_resp;

  uint8_t status[4] = {0};
  uint8_t const status_len = (_stub_running ? 2 : 4);

  uint32_t end_ms = millis() + timeout_ms;

  if (!readSLIP(timeout_ms)) {
    return -1;
  }

  // read fixed response first
  if (8 != readBytes(&fixed_resp, 8, end_ms - millis())) {
    Serial.printf("line %d\r\n", __LINE__);
    return -1; // probably timeout
  }

  // read variable payload if any
  uint16_t const payload_len = fixed_resp.length - status_len;
  uint8_t data[payload_len];

  if (payload_len) {
    uint16_t rd_len = readBytes(data, payload_len, end_ms - millis());
    if (payload_len != rd_len) {
      Serial.printf("line %d: payload_len = %u, rd_len = %u\r\n", __LINE__,
                    payload_len, rd_len);
      if (rd_len) {
        DBG_PRINT_BUF(data, rd_len);
      }
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
    Serial.printf("line %d\r\n", __LINE__);
    return -1; // probably timeout
  }

  if (!readSLIP(end_ms - millis())) {
    Serial.printf("line %d\r\n", __LINE__);
    return -1;
  }

#if DEBUG
  Serial.printf("<= c0 %02x %02x %04x %08x ", fixed_resp.dir, fixed_resp.opcode,
                fixed_resp.length, fixed_resp.reg_value);
  DBG_PRINT_BUF(data, payload_len);
  DBG_PRINT_BUF(status, status_len);
  Serial.println("c0");
#endif

  // check direction, opcode and status
  if (fixed_resp.dir == 0x01 && fixed_resp.opcode == opcode &&
      status[0] == 0x00 && status[1] == 0x00) {
    return 0; // status[0];
  }

  const char *mess_arr[0x0b + 1] = {
      NULL, NULL, NULL, NULL, NULL, "Received message is invalid",
      "Failed to act on received message", "Invalid CRC in message",
      "Flash write error", //  after writing a block of data to flash, the ROM
                           //  loader reads the value back and the 8-bit CRC is
                           //  compared to the data read from flash. If they
                           //  don’t match, this error is returned.
      "Flash read error", "Flash read length error", "Deflate error"};

  const char *mess =
      (status[1] <= 0x0b) ? mess_arr[status[1]] : "Unknown Error";
  Serial.printf("response failed: status = %02x %02x, %s\r\n", status[0],
                status[1], mess);

  return -1;
}

void ESP32BootROMClass::writeEscapedBytes(const uint8_t *data,
                                          uint16_t length) {
  // skip flashing data since it is a lot to print
  bool const print_payload = (length >= 200) ? false : true;

  uint16_t last_wr = 0;
  for (uint16_t i = 0; i < length; i++) {
    uint8_t b = data[i];

    if (b == 0xdb || b == 0xc0) {
      // write up to i-1
      if (last_wr < i) {
        _serial->write(data + last_wr, i - last_wr);

        if (DEBUG && print_payload) {
          DBG_PRINT_BUF(data + last_wr, i - last_wr);
        }
      }

      uint8_t esc[2] = {0xdb, 0x00};
      esc[1] = (b == 0xdb) ? 0xdd : 0xdc;

      _serial->write(esc, 2);

      // +1 since we already write current index with escape
      last_wr = i + 1;

      if (DEBUG && print_payload) {
        DBG_PRINT_BUF(esc, 2);
      }
    }
  }

  // last chunk without escape
  if (last_wr < length) {
    _serial->write(data + last_wr, length - last_wr);
    if (DEBUG && print_payload) {
      DBG_PRINT_BUF(data + last_wr, length - last_wr);
    }
  }
}

#endif
