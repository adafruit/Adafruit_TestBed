/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Adafruit Industries
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

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "SdFat.h"
#include "pio_usb.h"

#include "Adafruit_DAP.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

#define USBHOST_RHPORT 1

Adafruit_TestBed_Brains Brain;

volatile bool LCD_semaphore = false;

// Simple and low code CRC calculation (copied from PicoOTA)
class BrainCRC32 {
public:
  BrainCRC32() { crc = 0xffffffff; }

  ~BrainCRC32() {}

  void add(const void *d, uint32_t len) {
    const uint8_t *data = (const uint8_t *)d;
    for (uint32_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 1) {
          crc = (crc >> 1) ^ 0xedb88320;
        } else {
          crc >>= 1;
        }
      }
    }
  }

  uint32_t get() { return ~crc; }

private:
  uint32_t crc;
};

static inline uint32_t div_ceil(uint32_t v, uint32_t d) {
  return (v + d - 1) / d;
}

/**************************************************************************/
/*!
    @brief  Initializer, sets up the timestamp, neopixels, piezo, led,
            and analog reference. So get all pins assigned before calling
*/
/**************************************************************************/

Adafruit_TestBed_Brains::Adafruit_TestBed_Brains() {
  _inited = false;
  _lcd_line = 0;

  piezoPin = 15; // onboard buzzer
  ledPin = 25;   // green LED on Pico

  targetPowerPin = 6; // VBat switch

  neopixelNum = 1;  // LCD backlight
  neopixelPin = 13; // LCD backlight

  _sd_detect_pin = 14; // SD detect
  _sd_cs_pin = 17;     // SD chip select

  _usbh_dp_pin = 20; // USB Host D+
  _vbus_en_pin = 22; // USB Host VBus enable

  _target_rst = 27;
  _target_swdio = 2;
  _target_swdclk = 3;

  dap = NULL;

  esp32boot = NULL;
  _esp32_flash_defl = false;
  _esp32_chip_detect = 0;
  _esp32s3_in_reset = false;
}

void Adafruit_TestBed_Brains::begin(void) {
  // skip Adafruit_Neopixel by setting neopixelNum = 0 since
  // we will bit-banging due to the pio conflict with usb host
  neopixelNum = 0;
  Adafruit_TestBed::begin();

  neopixelNum = 1;
  pinMode(neopixelPin, OUTPUT);

  pinMode(_target_rst, OUTPUT);
  digitalWrite(_target_rst, HIGH);

  pinMode(_sd_detect_pin, INPUT_PULLUP);
  pinMode(_vbus_en_pin, OUTPUT);
  usbh_setVBus(false); // disabled by default

  analogReadResolution(12);

  // pixels->setBrightness(255); TODO can use variable to take color percentage
  // in setColor()
  setColor(0xFFFFFF);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();
  lcd.noCursor();

  _inited = true;
}

bool Adafruit_TestBed_Brains::inited(void) { return _inited; }

void Adafruit_TestBed_Brains::targetReset(uint32_t reset_ms) {
  digitalWrite(_target_rst, LOW);
  delay(reset_ms);
  digitalWrite(_target_rst, HIGH);

  // Note: S3 has an USB-OTG errata
  // https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
  // which is walkarounded by idf/arduino-esp32 to always mux JTAG to USB for
  // uploading and/or power on. Afterwards USB-OTG will be set up if selected
  // so. However rp2040 USBH is running too fast and can actually retrieve
  // device/configuration descriptor of JTAG before the OTG is fully setup.
  // Mark this for application usage
  if (_esp32_chip_detect == CHIP_DETECT_MAGIC_ESP32S3) {
    _esp32s3_in_reset = true;
  }
}

//--------------------------------------------------------------------+
// RP2040 Target
//--------------------------------------------------------------------+

void Adafruit_TestBed_Brains::rp2040_targetResetBootRom(int bootsel_pin,
                                                        uint32_t reset_ms) {
  pinMode(bootsel_pin, OUTPUT);

  digitalWrite(bootsel_pin, LOW);

  targetReset(reset_ms);
  delay(reset_ms);

  // change bootsel to input since it is muxed with Flash ChipSelect
  digitalWrite(bootsel_pin, HIGH);
  pinMode(bootsel_pin, INPUT);
}

size_t Adafruit_TestBed_Brains::rp2040_programUF2(const char *fpath) {
  File32 fsrc = SD.open(fpath);
  if (!fsrc) {
    Serial.printf("SD: cannot open file: %s\r\n", fpath);
    return 0;
  }

  size_t copied_bytes = 0;

  const char *dst_name = "FIRMWARE.UF2";
  File32 fdst = USBH_FS.open(dst_name, O_WRONLY | O_CREAT);

  if (!fdst) {
    Serial.printf("USBH_FS: cannot create file: %s\r\n", dst_name);
  } else {
    size_t const bufsize = 4096;
    uint8_t *buf = (uint8_t *)malloc(bufsize);
    if (!buf) {
      Serial.println("Not enough memory");
      return 0;
    }

    while (fsrc.available()) {
      memset(buf, 0x00, bufsize); // empty it out

      size_t rd_count = (size_t)fsrc.read(buf, bufsize);
      size_t wr_count = 0;

      setLED(HIGH);
      wr_count = fdst.write(buf, rd_count);
      setLED(LOW);

      copied_bytes += wr_count;

      if (wr_count != rd_count) {
        Serial.println("USBH_FS: Failed to write file");
        break;
      }
    }

    free(buf);
  }

  fsrc.close();
  fdst.close();

  return copied_bytes;
}

//--------------------------------------------------------------------+
// DAP Target
//--------------------------------------------------------------------+
static void dap_err_hanlder(const char *msg) { Brain.LCD_error(msg, NULL); }

bool Adafruit_TestBed_Brains::dap_begin(Adafruit_DAP *dp) {
  if (!dp) {
    return dp;
  }

  pinMode(_target_swdio, OUTPUT);
  digitalWrite(_target_swdio, LOW);

  pinMode(_target_swdclk, OUTPUT);
  digitalWrite(_target_swdclk, LOW);

  dap = dp;

  return dap->begin(_target_swdclk, _target_swdio, _target_rst,
                    dap_err_hanlder);
}

bool Adafruit_TestBed_Brains::dap_connect(void) {
  if (!dap) {
    return false;
  }

  LCD_printf("Connecting...");
  if (!dap->targetConnect()) {
    return false;
  }

  uint32_t dsu_did;
  if (!dap->select(&dsu_did)) {
    setColor(0xFF0000);
    LCD_printf(0, "Unknown MCU");
    LCD_printf(1, "ID = %08X", dsu_did);

    while (1) {
      delay(1);
    }

    return false;
  }

  uint32_t const page_size =
      dap->target_device.n_pages
          ? (dap->target_device.flash_size / dap->target_device.n_pages)
          : 0;

  Serial.printf("Found Target: %s, ID = %08lX\n", dap->target_device.name,
                dsu_did);
  Serial.printf("Flash size: %lu, Page Num: %lu, Page Size: %lu\n",
                dap->target_device.flash_size, dap->target_device.n_pages,
                page_size);

  return true;
}

void Adafruit_TestBed_Brains::dap_disconnect(void) {
  if (!dap) {
    return;
  }
  dap->deselect();
}

bool Adafruit_TestBed_Brains::dap_unprotectBoot(void) {
  if (!dap) {
    return false;
  }

  LCD_printf("Unlock chip...");
  bool ret = dap->unprotectBoot();
  LCD_printf(ret ? "OK" : "Failed");
  return ret;
}

bool Adafruit_TestBed_Brains::dap_protectBoot(void) {
  if (!dap) {
    return false;
  }

  LCD_printf("Lock chip...");
  bool ret = dap->protectBoot();
  LCD_printf(ret ? "OK" : "Failed");
  return ret;
}

bool Adafruit_TestBed_Brains::dap_eraseChip(void) {
  if (!dap) {
    return false;
  }

  uint32_t const dap_typeid = dap->getTypeID();

  // NOTE: STM32 does erase on-the-fly therefore erasing is not needed
  if (dap_typeid == DAP_TYPEID_STM32) {
    LCD_printf("Erasing..skipped");
  } else {
    LCD_printf("Erasing..");
    uint32_t ms = millis();

    dap->erase();

    ms = millis() - ms;
    LCD_printf("done in %.02fs", ms / 1000.0F);
  }

  return true;
}

size_t Adafruit_TestBed_Brains::dap_programFlash(const char *fpath,
                                                 uint32_t addr) {
  if (!dap) {
    return 0;
  }

  File32 fsrc = SD.open(fpath);
  if (!fsrc) {
    Serial.printf("SD: cannot open file: %s\r\n", fpath);
    return 0;
  }
  uint32_t fsize = fsrc.fileSize();

  size_t bufsize;
  uint32_t const dap_typeid = dap->getTypeID();

  switch (dap_typeid) {
  case DAP_TYPEID_SAM:
    bufsize = Adafruit_DAP_SAM::PAGESIZE;
    break;

  case DAP_TYPEID_SAMX5:
    bufsize = Adafruit_DAP_SAMx5::PAGESIZE;
    break;

  case DAP_TYPEID_NRF5X:
    bufsize = 4096;
    break;

  case DAP_TYPEID_STM32:
    bufsize = 4096;
    break;

  default:
    return false;
  }

  uint8_t *buf = (uint8_t *)malloc(bufsize);

  if (!buf) {
    Serial.printf("Not enough memory %u\n", bufsize);
    return 0;
  }

  LCD_printf("Programming..");

  BrainCRC32 crc32;
  dap->program_start(addr, fsize);

  uint32_t addr_tmp = addr;
  while (fsrc.available()) {
    memset(buf, 0xff, bufsize); // empty it out

    uint32_t rd_count = fsrc.read(buf, bufsize);

    setLED(HIGH);
    dap->programBlock(addr_tmp, buf, bufsize);
    crc32.add(buf, rd_count);
    setLED(LOW);

    addr_tmp += bufsize;
  }

  uint32_t target_crc = dap->computeFlashCRC32(addr, fsize);

  if (target_crc != crc32.get()) {
    LCD_printf("CRC Failed");
    Serial.printf("CRC mismtached: %08lX != %08lX\n", crc32.get(), target_crc);
  } else {
    LCD_printf("Done!");
  }

  free(buf);
  fsrc.close();

  return fsize;
}

//--------------------------------------------------------------------+
// ESP32 Target
//--------------------------------------------------------------------+

bool Adafruit_TestBed_Brains::esp32_begin(ESP32BootROMClass *bootrom,
                                          uint32_t baudrate) {
  esp32boot = bootrom;

  LCD_printf("Syncing ESP32");
  _esp32_chip_detect = esp32boot->begin(baudrate);

  if (_esp32_chip_detect) {
    LCD_printf("Synced OK");
    return true;
  } else {
    LCD_printf_error("Sync failed!");
    return false;
  }
}

void Adafruit_TestBed_Brains::esp32_end(bool reset_esp) {
  if (esp32boot->isRunningStub()) {
    // skip sending flash_finish to ROM loader here,
    // as it causes the loader to exit and run user code
    esp32boot->beginFlash(0, 0, esp32boot->getFlashWriteSize());

    if (_esp32_flash_defl) {
      esp32boot->endFlashDefl(reset_esp);
    } else {
      esp32boot->endFlash(reset_esp);
    }
  }

  esp32boot->end();
}

bool Adafruit_TestBed_Brains::esp32_s3_inReset(void) {
  return _esp32s3_in_reset;
}

void Adafruit_TestBed_Brains::esp32_s3_clearReset(void) {
  _esp32s3_in_reset = false;
}

size_t
Adafruit_TestBed_Brains::esp32_programFlashDefl(const esp32_zipfile_t *zfile,
                                                uint32_t addr) {
  if (!esp32boot) {
    return 0;
  }

  // Check if MD5 matches to skip this file
  uint8_t esp_md5[16];

#if 1 // change to 0 to skip pre-flash md5 check for testing
  esp32boot->md5Flash(addr, zfile->uncompressed_len, esp_md5);
  Serial.printf("Flash MD5: ");
  for (size_t i = 0; i < 16; i++) {
    Serial.printf("%02X ", esp_md5[i]);
  }
  Serial.println();
  if (0 == memcmp(zfile->md5, esp_md5, 16)) {
    LCD_printf(2, "MD5 matched");
    return zfile->uncompressed_len;
  }
#endif

  // Write Size is different depending on ROM (1K) or Stub (16KB)
  uint32_t const block_size = esp32boot->getFlashWriteSize();

  Serial.printf("Compressed %lu bytes to %lu\r\n", zfile->uncompressed_len,
                zfile->compressed_len);

  if (!esp32boot->beginFlashDefl(addr, zfile->uncompressed_len,
                                 zfile->compressed_len)) {
    LCD_printf_error("beginFlash failed!");
  } else {
    _esp32_flash_defl = true;

    uint32_t const block_num = div_ceil(zfile->compressed_len, block_size);

    //------------- Flashing  -------------//
    uint8_t const *data = zfile->data;
    uint32_t remain = zfile->compressed_len;

    for (uint32_t i = 0; i < block_num; i++) {
      setLED(HIGH);
      LCD_printf(1, "Pckt %u/%u", i + 1, block_num);

      uint32_t const wr_count = MIN(block_size, remain);

      // Note: flash deflat does not need padding
      if (!esp32boot->dataFlashDefl(data, wr_count)) {
        LCD_printf_error("Failed to flash");
        break;
      }

      setLED(LOW);

      data += wr_count;
      remain -= wr_count;
    }
    Serial.println();

    // Stub only writes each block to flash after 'ack'ing the receive,
    // so do a final dummy operation which will not be 'ack'ed
    // until the last block has actually been written out to flash
    if (esp32boot->isRunningStub()) {
      (void)esp32boot->read_chip_detect();
    }

    //------------- MD5 verification -------------//
    esp32boot->md5Flash(addr, zfile->uncompressed_len, esp_md5);

    if (0 == memcmp(zfile->md5, esp_md5, 16)) {
      LCD_printf(2, "MD5 matched");
    } else {
      LCD_error(NULL, "MD5 mismatched!!");

      Serial.printf("File: ");
      for (size_t i = 0; i < 16; i++) {
        Serial.printf("%02X ", zfile->md5[i]);
      }
      Serial.println();

      Serial.printf("ESP : ");
      for (size_t i = 0; i < 16; i++) {
        Serial.printf("%02X ", esp_md5[i]);
      }
      Serial.println();
    }
  }

  return zfile->uncompressed_len;
}

size_t Adafruit_TestBed_Brains::esp32_programFlash(const char *fpath,
                                                   uint32_t addr) {
  if (!esp32boot) {
    return 0;
  }

  // Write Size is different depending on ROM (1K) or Stub (16KB)
  uint32_t const block_size = esp32boot->getFlashWriteSize();
  uint8_t *buf = (uint8_t *)malloc(block_size);
  if (!buf) {
    LCD_printf_error("No memory %u\n", block_size);
    return 0;
  }

  File32 fsrc = SD.open(fpath);
  if (!fsrc) {
    Serial.printf("SD: cannot open file: %s\r\n", fpath);
    return 0;
  }
  uint32_t fsize = fsrc.fileSize();
  uint32_t total_count = 0;

  Serial.printf("fsize = %lu, block size = %lu\r\n", fsize, block_size);

  if (!esp32boot->beginFlash(addr, fsize, block_size)) {
    LCD_printf_error("beginFlash failed!");
  } else {
    LCD_printf("#Packets %u", div_ceil(fsize, block_size));

    MD5Builder md5;
    md5.begin();

    //------------- Flashing  -------------//
    while (fsrc.available()) {
      memset(buf, 0xff, block_size); // empty it out
      uint32_t const rd_count = fsrc.read(buf, block_size);

      setLED(HIGH);
      Serial.printf("#");

      if (!esp32boot->dataFlash(buf, block_size)) {
        LCD_printf_error("Failed to flash");
        break;
      }

      setLED(LOW);

      md5.add(buf, rd_count);
      total_count += rd_count;
    }
    Serial.println();

    // Stub only writes each block to flash after 'ack'ing the receive,
    // so do a final dummy operation which will not be 'ack'ed
    // until the last block has actually been written out to flash
    if (esp32boot->isRunningStub()) {
      (void)esp32boot->read_chip_detect();
    }

    //------------- MD5 verification -------------//
    md5.calculate();
    Serial.printf("md5 = %s\r\n", md5.toString().c_str());

    uint8_t file_md5[16];
    md5.getBytes(file_md5);

    uint8_t esp_md5[16];
    esp32boot->md5Flash(addr, fsize, esp_md5);

    if (0 == memcmp(file_md5, esp_md5, 16)) {
      LCD_printf("MD5 matched");
    } else {
      LCD_printf_error("MD5 mismatched!!");

      Serial.printf("File: ");
      for (size_t i = 0; i < 16; i++) {
        Serial.printf("%02X ", file_md5[i]);
      }
      Serial.println();

      Serial.printf("ESP : ");
      for (size_t i = 0; i < 16; i++) {
        Serial.printf("%02X ", esp_md5[i]);
      }
      Serial.println();
    }
  }

  free(buf);
  fsrc.close();

  return total_count;
}

//--------------------------------------------------------------------+
// SD Card
//--------------------------------------------------------------------+

bool Adafruit_TestBed_Brains::SD_detected(void) {
  return digitalRead(_sd_detect_pin);
}

bool Adafruit_TestBed_Brains::SD_begin(uint32_t max_clock) {
  if (!SD_detected()) {
    LCD_printf(0, "No SD Card");
    while (!SD_detected()) {
      delay(10);
    }
  }

  if (!SD.begin(_sd_cs_pin, max_clock)) {
    LCD_printf(0, "SD init failed");
    while (1) {
      delay(10);
    }
  }

  LCD_printf(0, "SD mounted");
  return true;
}

//--------------------------------------------------------------------+
// LCD
//--------------------------------------------------------------------+

// 1s = 1.000.000 us --> 120.000.000 nop
// 1 us -> 120 nop
// full frame, 1.25 us -> 150 nop
//  - T1H 0,76 us -> 91 nop
//  - T0H 0,36 us -> 43 nop

void Adafruit_TestBed_Brains::setColor(uint32_t color) {
  uint8_t r = (uint8_t)(color >> 16), g = (uint8_t)(color >> 8),
          b = (uint8_t)color;
  uint8_t buf[3] = {r, g, b};

  uint8_t *ptr, *end, p, bitMask;
  uint32_t const pinMask = 1ul << neopixelPin;

  ptr = buf;
  end = ptr + 3;
  p = *ptr++;
  bitMask = 0x80;

  // 800 KHz
  for (;;) {
    if (p & bitMask) {
      // T1H 0,76 us -> 91 nop
      sio_hw->gpio_set = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop;");

      // 150 - 91 = 59 nop
      sio_hw->gpio_clr = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop;");
    } else {
      // T0H 0,36 us -> 43 nop
      sio_hw->gpio_set = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop;");

      // 150 - 43 = 107 nop
      sio_hw->gpio_clr = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop;");
    }

    // if a full byte is sent, next to another byte
    if (0 == (bitMask >>= 1)) {
      if (ptr >= end)
        break;
      p = *ptr++;
      bitMask = 0x80;
    }
  }
}

void Adafruit_TestBed_Brains::lcd_write(uint8_t linenum, char linebuf[17]) {
  // wait for semaphore to release
  while (LCD_semaphore)
    yield();
  LCD_semaphore = true;
  // fill the rest with spaces
  memset(linebuf + strlen(linebuf), ' ', 16 - strlen(linebuf));
  linebuf[16] = 0;
  lcd.setCursor(0, linenum);
  lcd.write(linebuf);

  Serial.print("LCD: ");
  Serial.println(linebuf);
  Serial.flush();

  _lcd_line = 1 - linenum;
  LCD_semaphore = false;
}

#define _LCD_PRINTF(_line, _format)                                            \
  do {                                                                         \
    char linebuf[17];                                                          \
    va_list ap;                                                                \
    va_start(ap, _format);                                                     \
    vsnprintf(linebuf, sizeof(linebuf), _format, ap);                          \
    va_end(ap);                                                                \
    lcd_write(_line, linebuf);                                                 \
  } while (0)

void Adafruit_TestBed_Brains::LCD_printf(uint8_t linenum, const char format[],
                                         ...) {
  _LCD_PRINTF(linenum, format);
}

void Adafruit_TestBed_Brains::LCD_printf(const char format[], ...) {
  _LCD_PRINTF(_lcd_line, format);
}

void Adafruit_TestBed_Brains::LCD_printf_error(const char format[], ...) {
  setColor(0xFF0000);
  _LCD_PRINTF(_lcd_line, format);
}

void Adafruit_TestBed_Brains::LCD_info(const char *msg1, const char *msg2) {
  setColor(0xFFFFFF);
  LCD_printf(0, msg1);

  if (msg2) {
    LCD_printf(1, msg2);
  }
}

void Adafruit_TestBed_Brains::LCD_error(const char *errmsg1,
                                        const char *errmsg2) {
  setColor(0xFF0000);
  if (errmsg1) {
    LCD_printf(0, errmsg1);
  }
  if (errmsg2) {
    LCD_printf(1, errmsg2);
  }
}

//--------------------------------------------------------------------+
// USB Host
//--------------------------------------------------------------------+

void Adafruit_TestBed_Brains::usbh_setVBus(bool en) {
  digitalWrite(_vbus_en_pin, en ? HIGH : LOW);
}

bool Adafruit_TestBed_Brains::usbh_begin(void) {
  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    while (!Serial) {
      delay(10); // wait for native usb
    }
    Serial.printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be "
                  "multiple of 120 Mhz\r\n",
                  cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU "
                  "Speed \r\n");
    while (1) {
      delay(1);
    }
  }

  // enable vbus
  pinMode(_vbus_en_pin, OUTPUT);
  usbh_setVBus(true);

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = (uint8_t)_usbh_dp_pin;

  USBHost.configure_pio_usb(USBHOST_RHPORT, &pio_cfg);
  if (!USBHost.begin(USBHOST_RHPORT)) {
    Serial.println("usb host begin failed");
    usbh_setVBus(false);
    return false;
  }

  return true;
}

bool Adafruit_TestBed_Brains::usbh_inited(void) { return tuh_inited(); }

bool Adafruit_TestBed_Brains::usbh_mountFS(uint8_t dev_addr) {
  // Initialize block device with MSC device address (only support LUN 0)
  USBH_BlockDev.begin(dev_addr);
  USBH_BlockDev.setActiveLUN(0);

  return USBH_FS.begin(&USBH_BlockDev);
}

bool Adafruit_TestBed_Brains::usbh_umountFS(uint8_t dev_addr) {
  (void)dev_addr;

  // unmount file system
  USBH_FS.end();

  // end block device
  USBH_BlockDev.end();

  return true;
}

#endif
