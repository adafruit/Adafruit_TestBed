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
  CHIP_DETECT_MAGIC_ESP8266 = 0xFFF0C101,
  CHIP_DETECT_MAGIC_ESP32C6 = 0x2CE0806F,
};

/**************************************************************************/
/*!
    @brief Flasher stub loader image for one ESP chip family, uploaded to RAM
   to speed up flashing
*/
/**************************************************************************/
typedef struct {
  uint32_t chip_detect;  ///< Chip detect magic value this stub is for
  uint32_t entry;        ///< Entry point address of the stub
  uint32_t text_start;   ///< RAM load address of the text segment
  uint32_t text_length;  ///< Length of the text segment in bytes
  const uint8_t *text;   ///< Text segment contents
  uint32_t data_start;   ///< RAM load address of the data segment
  uint32_t data_length;  ///< Length of the data segment in bytes
  const uint8_t *data;   ///< Data segment contents
  const char *chip_name; ///< Human readable chip name
} esp32_stub_loader_t;

/**************************************************************************/
/*!
    @brief Talks to an ESP chip's serial ROM bootloader (or flasher stub) to
   read registers and program flash
*/
/**************************************************************************/
class ESP32BootROMClass {
public:
  /*!
      @brief  Create a ROM loader on a serial port
      @param  hwSerial Serial port connected to the ESP chip
      @param  gpio0Pin Pin connected to the chip's GPIO0, or -1 if not used
      @param  resetnPin Pin connected to the chip's reset, or -1 if not used
  */
  ESP32BootROMClass(HardwareSerial &hwSerial, int gpio0Pin = -1,
                    int resetnPin = -1);

  /*!
      @brief  Reset the chip into its bootloader, sync, upload the flasher stub
              if one is available and switch baudrate
      @param  baudrate Baudrate to use once synced
      @return Chip detect magic on success, otherwise 0
  */
  uint32_t begin(unsigned long baudrate);

  /*!
      @brief  Release the GPIO0 and reset pins
  */
  void end();

  /*!
      @brief  Check if the flasher stub is running instead of the ROM loader
      @return True if the stub is running
  */
  bool isRunningStub(void);

  /*!
      @brief  Get the flash write block size for the running loader
      @return Block size in bytes (larger when the stub is running)
  */
  uint32_t getFlashWriteSize(void);

  // High Level commands

  /*!
      @brief  Read a 32-bit register on the chip
      @param  regAddr Address of the register
      @param  regValue Set to the register value
      @param  timeout_ms How many ms to wait for a response
      @return True on success
  */
  bool read_reg(uint32_t regAddr, uint32_t *regValue,
                uint32_t timeout_ms = ESP32_DEFAULT_TIMEOUT);

  /*!
      @brief  Read the chip's factory MAC address from efuse
      @param  mac Set to the 6-byte MAC address
      @return True on success
  */
  bool read_MAC(uint8_t mac[6]);

  /*!
      @brief  Read the chip detect magic register
      @return Chip detect magic value, or 0 on failure
  */
  uint32_t read_chip_detect(void);

  // uncompressed

  /*!
      @brief  Start an uncompressed flash write
      @param  offset Flash address to start writing at
      @param  size Total number of bytes to write
      @param  chunkSize Size of each block sent with dataFlash()
      @return True on success
  */
  bool beginFlash(uint32_t offset, uint32_t size, uint32_t chunkSize);

  /*!
      @brief  Send one block of an uncompressed flash write
      @param  data Block contents
      @param  length Block length in bytes
      @return True on success
  */
  bool dataFlash(const void *data, uint32_t length);

  /*!
      @brief  Finish an uncompressed flash write
      @param  reboot Flag word for the flash end command: 0 to reboot, 1 to
              run user code
      @return True on success
  */
  bool endFlash(uint32_t reboot);

  // compressed

  /*!
      @brief  Start a deflate-compressed flash write
      @param  offset Flash address to start writing at
      @param  size Uncompressed size of the data in bytes
      @param  zip_size Compressed size of the data in bytes
      @return True on success
  */
  bool beginFlashDefl(uint32_t offset, uint32_t size, uint32_t zip_size);

  /*!
      @brief  Send one block of a compressed flash write
      @param  data Compressed block contents
      @param  len Block length in bytes
      @return True on success
  */
  bool dataFlashDefl(const void *data, uint32_t len);

  /*!
      @brief  Finish a compressed flash write
      @param  reboot Flag word for the flash end command: 0 to reboot, 1 to
              run user code
      @return True on success
  */
  bool endFlashDefl(uint32_t reboot);

  /*!
      @brief  Calculate the MD5 of a region of the chip's flash
      @param  offset Flash address of the region
      @param  size Size of the region in bytes
      @param  result Set to the 16-byte MD5
      @return True on success
  */
  bool md5Flash(uint32_t offset, uint32_t size, uint8_t *result);

private:
  void init();
  void resetBootloader(void);
  bool sync();
  bool uploadStub(const esp32_stub_loader_t *stub);
  bool syncStub(uint32_t timeout_ms);

  // Command and Response
  void command(uint8_t opcode, const void *data, uint16_t length,
               const void *data2 = NULL, uint16_t len2 = 0);
  int response(uint8_t opcode, uint32_t timeout_ms, void *body = NULL);

  bool sendCommandGetResponse(uint8_t opcode, const void *data, uint16_t length,
                              const void *data2 = NULL, uint16_t len2 = 0,
                              uint32_t timeout_ms = ESP32_DEFAULT_TIMEOUT,
                              void *body = NULL);

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
