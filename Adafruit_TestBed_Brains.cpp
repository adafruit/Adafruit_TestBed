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

#include "Arduino.h"
#include "pio_usb.h"

#include "Adafruit_TestBed_Brains.h"

#define USBHOST_RHPORT 1

Adafruit_TestBed_Brains Brain;

/**************************************************************************/
/*!
    @brief  Initializer, sets up the timestamp, neopixels, piezo, led,
            and analog reference. So get all pins assigned before calling
*/
/**************************************************************************/

Adafruit_TestBed_Brains::Adafruit_TestBed_Brains() {
  _inited = false;

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
}

void Adafruit_TestBed_Brains::begin(void) {
  // neopixel is already set up
  Adafruit_TestBed::begin();

  pinMode(_target_rst, OUTPUT);
  digitalWrite(_target_rst, HIGH);

  pinMode(_sd_detect_pin, INPUT_PULLUP);
  pinMode(_vbus_en_pin, OUTPUT);
  usbh_setVBus(false); // disabled by default

  analogReadResolution(12);

  pixels->setBrightness(255);
  setColor(0xFFFFFF);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();
  lcd.noCursor();

  //  if (SD_detected()) {
  //    Serial.print("SD inserted...");
  //    if (!SD_begin()) {
  //      Serial.println("Could not init SD!");
  //    } else {
  //      uint32_t SDsize = SD.card()->sectorCount();
  //      if (SDsize == 0) {
  //        Serial.println("Can't determine the card size");
  //      } else {
  //        Serial.printf("Card size = %0.1f GB\n", 0.000000512 *
  //        (float)SDsize); Serial.println("Files found (date time size
  //        name):"); SD.ls(LS_R | LS_DATE | LS_SIZE);
  //      }
  //    }
  //  }
  _inited = true;
}

bool Adafruit_TestBed_Brains::inited(void) { return _inited; }

//--------------------------------------------------------------------+
// Target
//--------------------------------------------------------------------+

void Adafruit_TestBed_Brains::targetReset(uint32_t reset_ms) {
  digitalWrite(_target_rst, LOW);
  delay(reset_ms);
  digitalWrite(_target_rst, HIGH);
}

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
    Serial.flush();
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
      memset(buf, sizeof(buf), 0x00); // empty it out

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
// SD Card
//--------------------------------------------------------------------+

bool Adafruit_TestBed_Brains::SD_detected(void) {
  return digitalRead(_sd_detect_pin);
}

bool Adafruit_TestBed_Brains::SD_begin(uint32_t max_clock) {
  return SD.begin(_sd_cs_pin, max_clock);
}

//--------------------------------------------------------------------+
// LCD
//--------------------------------------------------------------------+

void Adafruit_TestBed_Brains::LCD_printf(bool linenum, const char format[],
                                         ...) {
  char linebuf[17];
  memset(linebuf, 0, sizeof(linebuf));

  va_list ap;
  va_start(ap, format);
  vsnprintf(linebuf, sizeof(linebuf), format, ap);

  // fill the rest with spaces
  memset(linebuf + strlen(linebuf), ' ', 16 - strlen(linebuf));
  linebuf[16] = 0;
  lcd.setCursor(0, linenum);
  lcd.write(linebuf);
  va_end(ap);

  Serial.print("LCD: ");
  Serial.println(linebuf);
}

void Adafruit_TestBed_Brains::LCD_info(const char *msg1, const char *msg2) {
  setColor(0xFFFFFF);
  LCD_printf(0, msg1);
  LCD_printf(1, msg2);
}

void Adafruit_TestBed_Brains::LCD_error(const char *errmsg1,
                                        const char *errmsg2) {
  setColor(0xFF0000);
  LCD_printf(0, errmsg1);
  LCD_printf(1, errmsg2);
  delay(250);
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
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be "
                  "multiple of 120 Mhz\r\n",
                  cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU "
                  "Speed \r\n",
                  cpu_hz);
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
