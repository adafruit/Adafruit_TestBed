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

#ifdef ARDUINO_ARCH_RP2040

#include "SdFat_Adafruit_Fork.h"
#include "pio_usb.h"

#include "Adafruit_DAP.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

#define USBHOST_RHPORT 1

// Workaround force F_CPU to 240MHz for programming RP2350 due to handshake
// timeout
#if F_CPU != 120000000L && F_CPU != 240000000L
#error "F_CPU must be set to either 120Mhz or 240Mhz for pio-usb host"
#endif

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

  targetResetPin = 27;
  _target_swdio = 2;
  _target_swdclk = 3;

  dap = NULL;
}

void Adafruit_TestBed_Brains::begin(void) {
  // skip Adafruit_Neopixel by setting neopixelNum = 0 since
  // we will bit-banging due to the pio conflict with usb host
  neopixelNum = 0;
  Adafruit_TestBed::begin();

  neopixelNum = 1;
  pinMode(neopixelPin, OUTPUT);

  pinMode(targetResetPin, OUTPUT);
  digitalWrite(targetResetPin, HIGH);

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

//--------------------------------------------------------------------+
// RP2040 Target
//--------------------------------------------------------------------+

void Adafruit_TestBed_Brains::rp2_targetResetBootRom(int bootsel_pin,
                                                     uint32_t reset_ms) {
  pinMode(bootsel_pin, OUTPUT);
  digitalWrite(bootsel_pin, LOW);

  targetReset(reset_ms);
  delay(reset_ms);

  // change bootsel to input since it is muxed with Flash ChipSelect
  digitalWrite(bootsel_pin, HIGH);
  pinMode(bootsel_pin, INPUT);
}

size_t Adafruit_TestBed_Brains::rp2_programUF2(const uint8_t *buffer,
                                               size_t bufsize) {
  size_t copied_bytes = 0;
  const char *dst_name = "FIRMWARE.UF2";
  File32 fdst = USBH_FS.open(dst_name, O_WRONLY | O_CREAT);

  if (!fdst) {
    Serial.printf("USBH_FS: cannot create file: %s\r\n", dst_name);
  } else {
    while (copied_bytes < bufsize) {
      size_t count = bufsize - copied_bytes;
      if (count > 4096) {
        count = 4096; // write 1 sector each
      }

      setLED(HIGH);
      size_t wr_count = fdst.write(buffer, count);
      setLED(LOW);

      buffer += wr_count;
      copied_bytes += wr_count;

      if (wr_count != count) {
        Serial.println("USBH_FS: Failed to write file");
        break;
      }
    }
  }

  fdst.close();

  return copied_bytes;
}

size_t Adafruit_TestBed_Brains::rp2_programUF2(const char *fpath) {
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

  return dap->begin(_target_swdclk, _target_swdio, targetResetPin,
                    dap_err_hanlder);
}

bool Adafruit_TestBed_Brains::dap_connect(uint32_t swj_clock) {
  if (!dap) {
    return false;
  }

  LCD_printf("Connecting...");
  if (!dap->targetConnect(swj_clock)) {
    return false;
  }

  uint32_t dsu_did;
  if (!dap->select(&dsu_did)) {
    setColor(0xFF0000);
    LCD_printf(0, "Unknown MCU");
    LCD_printf(1, "ID = %08X", dsu_did);

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

  Serial.println("Unlock chip...");
  bool ret = dap->unprotectBoot();
  Serial.println(ret ? "OK" : "Failed");
  return ret;
}

bool Adafruit_TestBed_Brains::dap_protectBoot(void) {
  if (!dap) {
    return false;
  }

  Serial.println("Lock chip...");
  bool ret = dap->protectBoot();
  Serial.println(ret ? "OK" : "Failed");
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
    LCD_printf("Erased in %.02fs", ms / 1000.0F);
  }

  return true;
}

size_t Adafruit_TestBed_Brains::dap_programFlash(const char *fpath,
                                                 uint32_t addr, bool do_verify,
                                                 bool do_crc32) {
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

  LCD_printf("Programming...");
  uint32_t ms = millis();

  BrainCRC32 crc32;
  dap->program_start(addr, fsize);

  uint32_t addr_tmp = addr;
  while (fsrc.available()) {
    memset(buf, 0xff, bufsize); // empty it out
    uint32_t rd_count = fsrc.read(buf, bufsize);

    setLED(HIGH);

    // don't verify each write if we use crc32
    if (!dap->programFlash(addr_tmp, buf, bufsize, !do_crc32 && do_verify)) {
      Serial.printf("Failed to program block at %08lX\n", addr_tmp);
      free(buf);
      fsrc.close();
      return addr_tmp - addr;
    }

    if (do_crc32) {
      crc32.add(buf, rd_count);
    }

    addr_tmp += bufsize;

    setLED(LOW);
  }

  Serial.printf("Programming end, t = %lu ms\r\n", millis(), millis() - ms);

  if (do_crc32) {
    ms = millis();
    Serial.printf("CRC32 start\r\n", ms);
    uint32_t target_crc = dap->computeFlashCRC32(addr, fsize);
    Serial.printf("CRC32 end, t = %lu ms\r\n", millis(), millis() - ms);

    if (target_crc != crc32.get()) {
      LCD_printf("CRC Failed");
      Serial.printf("CRC mismtached: %08lX != %08lX\n", crc32.get(),
                    target_crc);
    } else {
      LCD_printf("Done!");
    }
  } else {
    LCD_printf("Done!");
  }

  free(buf);
  fsrc.close();

  return fsize;
}

size_t Adafruit_TestBed_Brains::dap_readFlash(const char *fpath, uint32_t addr,
                                              size_t size) {
  if (!dap) {
    return 0;
  }

  size_t bufsize = 4096;
  uint8_t *buf = (uint8_t *)malloc(bufsize);
  if (!buf) {
    return 0;
  }

  File32 fsrc = SD.open(fpath, O_CREAT | O_WRONLY);
  if (!fsrc) {
    Serial.printf("SD: cannot open file: %s\r\n", fpath);
    return 0;
  }

  LCD_printf("Reading...");

  size_t remain = size;
  while (remain) {
    uint32_t count = min(remain, bufsize);

    setLED(HIGH);
    if (!dap->dap_read_block(addr, buf, (int)count)) {
      Serial.printf("Failed to read block at %08lX\n", addr);
      break;
    }
    setLED(LOW);

    fsrc.write(buf, count);
    LCD_printf("%d remaining", remain);
    addr += count;
    remain -= count;
  }

  fsrc.close();
  LCD_printf("Done!");

  return size - remain;
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
#define NOP1 __asm volatile("nop");
#define NOP2 NOP1 NOP1
#define NOP3 NOP2 NOP1
#define NOP4 NOP2 NOP2
#define NOP5 NOP4 NOP1
#define NOP6 NOP4 NOP2
#define NOP7 NOP4 NOP3
#define NOP8 NOP4 NOP4
#define NOP9 NOP8 NOP1
#define NOP10 NOP8 NOP2
#define NOP11 NOP8 NOP3
#define NOP12 NOP8 NOP4
#define NOP13 NOP8 NOP5
#define NOP14 NOP8 NOP6
#define NOP15 NOP8 NOP7
#define NOP16 NOP8 NOP8
#define NOP17 NOP16 NOP1
#define NOP18 NOP16 NOP2
#define NOP19 NOP16 NOP3
#define NOP20 NOP16 NOP4
#define NOP21 NOP16 NOP5
#define NOP22 NOP16 NOP6
#define NOP23 NOP16 NOP7
#define NOP24 NOP16 NOP8
#define NOP25 NOP16 NOP9
#define NOP26 NOP16 NOP10
#define NOP27 NOP16 NOP11
#define NOP28 NOP16 NOP12
#define NOP29 NOP16 NOP13
#define NOP30 NOP16 NOP14
#define NOP31 NOP16 NOP15
#define NOP32 NOP16 NOP16
#define NOP33 NOP32 NOP1
#define NOP34 NOP32 NOP2
#define NOP35 NOP32 NOP3
#define NOP36 NOP32 NOP4
#define NOP37 NOP32 NOP5
#define NOP38 NOP32 NOP6
#define NOP39 NOP32 NOP7
#define NOP40 NOP32 NOP8
#define NOP41 NOP32 NOP9
#define NOP42 NOP32 NOP10
#define NOP43 NOP32 NOP11
#define NOP44 NOP32 NOP12
#define NOP45 NOP32 NOP13
#define NOP46 NOP32 NOP14
#define NOP47 NOP32 NOP15
#define NOP48 NOP32 NOP16
#define NOP49 NOP48 NOP1
#define NOP50 NOP48 NOP2
#define NOP51 NOP48 NOP3
#define NOP52 NOP48 NOP4
#define NOP53 NOP48 NOP5
#define NOP54 NOP48 NOP6
#define NOP55 NOP48 NOP7
#define NOP56 NOP48 NOP8
#define NOP57 NOP48 NOP9
#define NOP58 NOP48 NOP10
#define NOP59 NOP48 NOP11
#define NOP60 NOP48 NOP12
#define NOP61 NOP48 NOP13
#define NOP62 NOP48 NOP14
#define NOP63 NOP48 NOP15
#define NOP64 NOP48 NOP16
#define NOP65 NOP64 NOP1
#define NOP66 NOP64 NOP2
#define NOP67 NOP64 NOP3
#define NOP68 NOP64 NOP4
#define NOP69 NOP64 NOP5
#define NOP70 NOP64 NOP6
#define NOP71 NOP64 NOP7
#define NOP72 NOP64 NOP8
#define NOP73 NOP64 NOP9
#define NOP74 NOP64 NOP10
#define NOP75 NOP64 NOP11
#define NOP76 NOP64 NOP12
#define NOP77 NOP64 NOP13
#define NOP78 NOP64 NOP14
#define NOP79 NOP64 NOP15
#define NOP80 NOP64 NOP16
#define NOP81 NOP80 NOP1
#define NOP82 NOP80 NOP2
#define NOP83 NOP80 NOP3
#define NOP84 NOP80 NOP4
#define NOP85 NOP80 NOP5
#define NOP86 NOP80 NOP6
#define NOP87 NOP80 NOP7
#define NOP88 NOP80 NOP8
#define NOP89 NOP80 NOP9
#define NOP90 NOP80 NOP10
#define NOP91 NOP80 NOP11
#define NOP92 NOP80 NOP12
#define NOP93 NOP80 NOP13
#define NOP94 NOP80 NOP14
#define NOP95 NOP80 NOP15
#define NOP96 NOP80 NOP16
#define NOP97 NOP96 NOP1
#define NOP98 NOP96 NOP2
#define NOP99 NOP96 NOP3
#define NOP100 NOP96 NOP4
#define NOP101 NOP100 NOP1
#define NOP102 NOP100 NOP2
#define NOP103 NOP100 NOP3
#define NOP104 NOP100 NOP4
#define NOP105 NOP100 NOP5
#define NOP106 NOP100 NOP6
#define NOP107 NOP100 NOP7
#define NOP108 NOP100 NOP8
#define NOP109 NOP100 NOP9
#define NOP110 NOP100 NOP10
#define NOP111 NOP100 NOP11
#define NOP112 NOP100 NOP12
#define NOP113 NOP100 NOP13
#define NOP114 NOP100 NOP14
#define NOP115 NOP100 NOP15
#define NOP116 NOP100 NOP16
#define NOP117 NOP116 NOP1
#define NOP118 NOP116 NOP2
#define NOP119 NOP116 NOP3
#define NOP120 NOP116 NOP4
#define NOP121 NOP116 NOP5
#define NOP122 NOP116 NOP6
#define NOP123 NOP116 NOP7
#define NOP124 NOP116 NOP8
#define NOP125 NOP116 NOP9
#define NOP126 NOP116 NOP10
#define NOP127 NOP116 NOP11
#define NOP128 NOP116 NOP12
#define NOP129 NOP128 NOP1
#define NOP130 NOP128 NOP2
#define NOP131 NOP128 NOP3
#define NOP132 NOP128 NOP4
#define NOP133 NOP128 NOP5
#define NOP134 NOP128 NOP6
#define NOP135 NOP128 NOP7
#define NOP136 NOP128 NOP8
#define NOP137 NOP128 NOP9
#define NOP138 NOP128 NOP10
#define NOP139 NOP128 NOP11
#define NOP140 NOP128 NOP12
#define NOP141 NOP128 NOP13
#define NOP142 NOP128 NOP14
#define NOP143 NOP128 NOP15
#define NOP144 NOP128 NOP16
#define NOP145 NOP144 NOP1
#define NOP146 NOP144 NOP2
#define NOP147 NOP144 NOP3
#define NOP148 NOP144 NOP4
#define NOP149 NOP144 NOP5
#define NOP150 NOP144 NOP6
#define NOP151 NOP144 NOP7
#define NOP152 NOP144 NOP8
#define NOP153 NOP144 NOP9
#define NOP154 NOP144 NOP10
#define NOP155 NOP144 NOP11
#define NOP156 NOP144 NOP12
#define NOP157 NOP144 NOP13
#define NOP158 NOP144 NOP14
#define NOP159 NOP144 NOP15
#define NOP160 NOP144 NOP16
#define NOP161 NOP160 NOP1
#define NOP162 NOP160 NOP2
#define NOP163 NOP160 NOP3
#define NOP164 NOP160 NOP4
#define NOP165 NOP160 NOP5
#define NOP166 NOP160 NOP6
#define NOP167 NOP160 NOP7
#define NOP168 NOP160 NOP8
#define NOP169 NOP160 NOP9
#define NOP170 NOP160 NOP10
#define NOP171 NOP160 NOP11
#define NOP172 NOP160 NOP12
#define NOP173 NOP160 NOP13
#define NOP174 NOP160 NOP14
#define NOP175 NOP160 NOP15
#define NOP176 NOP160 NOP16
#define NOP177 NOP176 NOP1
#define NOP178 NOP176 NOP2
#define NOP179 NOP176 NOP3
#define NOP180 NOP176 NOP4
#define NOP181 NOP176 NOP5
#define NOP182 NOP176 NOP6
#define NOP183 NOP176 NOP7
#define NOP184 NOP176 NOP8
#define NOP185 NOP176 NOP9
#define NOP186 NOP176 NOP10
#define NOP187 NOP176 NOP11
#define NOP188 NOP176 NOP12
#define NOP189 NOP176 NOP13
#define NOP190 NOP176 NOP14
#define NOP191 NOP176 NOP15
#define NOP192 NOP176 NOP16
#define NOP193 NOP192 NOP1
#define NOP194 NOP192 NOP2
#define NOP195 NOP192 NOP3
#define NOP196 NOP192 NOP4
#define NOP197 NOP192 NOP5
#define NOP198 NOP192 NOP6
#define NOP199 NOP192 NOP7
#define NOP200 NOP192 NOP8

#define _NOP_COUNT(n) NOP##n
#define NOP_COUNT(n) _NOP_COUNT(n)

void __no_inline_not_in_flash_func(Adafruit_TestBed_Brains::setColor)(
    uint32_t color) {
  static uint32_t end_us = 0;
  static uint32_t last_color = 0x123456;

  if (last_color == color) {
    // no need to update
    return;
  }
  last_color = color;

  uint8_t r = (uint8_t)(color >> 16); // red
  uint8_t g = (uint8_t)(color >> 8);  // green
  uint8_t b = (uint8_t)color;         // blue

  uint8_t buf[3] = {r, g, b};

  uint8_t *ptr, *end, p, bitMask;
  uint32_t const pinMask = 1ul << neopixelPin;

  ptr = buf;
  end = ptr + 3;
  p = *ptr++;
  bitMask = 0x80;

  // wait for previous frame to finish
  enum { FRAME_TIME_US = 300 };

  uint32_t now = end_us;
  while (now - end_us < FRAME_TIME_US) {
    now = micros();
    if (now < end_us) {
      // micros() overflow
      end_us = now;
    }
  }

  uint32_t const isr_context = save_and_disable_interrupts();

  /* Neopixel is 800 KHz, 1T = 1.25 us = 150 nop (120Mhz), 300 nop (240Mhz)
     - T1H = 0.8 us, T1L = 0.45 us
     - T0H = 0.4 us, T0L = 0.85 us
     If F_CPU is 120 MHz: 120 nop = 1us -> 1 nop = 8.33 ns
     - T1H = 0.8 us = 96 nop, T1L = 0.45 us = 54 nop
     - T0H = 0.4 us = 48 nop, T0L = 0.85 us = 102 nop
     If F_CPU is 240 MHz: 240 nop = 1us -> 1 nop = 4.17 ns
     - T1H = 0.8 us = 192 nop, T1L = 0.45 us = 108 nop
     - T0H = 0.4 us = 96 nop, T0L = 0.85 us = 204 nop
     Due to overhead the number of NOP is actually smaller, also M33 run faster
     (higher IPC) therefore these are hand tuned,
  */
#if defined(ARDUINO_RASPBERRY_PI_PICO)
// value for rp2040 at 120MHz
#define T1H_CYCLE 90
#define T1L_CYCLE 39
#define T0H_CYCLE 42
#define T0L_CYCLE 84
#define LOOP_OVERHEAD_CYCLE 10 // overhead for if/else and loop
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2)
// value for rp2350 at 120MHz
#define T1H_CYCLE 190
#define T1L_CYCLE 90
#define T0H_CYCLE 88
#define T0L_CYCLE 180
#define LOOP_OVERHEAD_CYCLE 5
#endif

  while (1) {
    if (p & bitMask) {
      // T1H 0.8 us = 96 nop (without overhead)
      sio_hw->gpio_set = pinMask;
      NOP_COUNT(T1H_CYCLE);
#if F_CPU == 240000000L
      NOP_COUNT(T1H_CYCLE);
#endif

      // T1L 0.45 = 54 - 10 (ifelse) - 5 (overhead) = 44 nop
      sio_hw->gpio_clr = pinMask;
      NOP_COUNT(T1L_CYCLE);
#if F_CPU == 240000000L
      NOP_COUNT(T1L_CYCLE);
#endif

    } else {
      // T0H 0.4 us = 48 cycles
      sio_hw->gpio_set = pinMask;
      NOP_COUNT(T0H_CYCLE);
#if F_CPU == 240000000L
      NOP_COUNT(T0H_CYCLE);
#endif

      // T0L 0.85 us = 102 - 10 (ifelse) - 5 (overhead) = 87 nop
      sio_hw->gpio_clr = pinMask;
      NOP_COUNT(T0L_CYCLE);
#if F_CPU == 240000000L
      NOP_COUNT(T0L_CYCLE);
#endif
    }

    if (bitMask >>= 1) {
      // not full byte, shift to next bit
      NOP_COUNT(LOOP_OVERHEAD_CYCLE);
    } else {
      // probably take 10 nops
      // if a full byte is sent, next to another byte
      if (ptr >= end) {
        break;
      }
      p = *ptr++;
      bitMask = 0x80;
    }
#if F_CPU == 240000000L
    NOP_COUNT(LOOP_OVERHEAD_CYCLE);
#endif
  }

  restore_interrupts(isr_context);

  end_us = micros();
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
  if (cpu_hz % 12000000UL) {
    while (!Serial) {
      delay(10); // wait for native usb
    }
    Serial.printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be "
                  "multiple of 12 Mhz\r\n",
                  cpu_hz);
    Serial.println("Change your CPU Clock to 12*n Mhz in Menu->CPU Speed");
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
