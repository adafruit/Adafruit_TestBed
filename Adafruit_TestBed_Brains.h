#ifndef ADAFRUIT_TESTBED_BRAINS_H
#define ADAFRUIT_TESTBED_BRAINS_H

#ifdef ARDUINO_ARCH_RP2040

#include "Adafruit_TestBed.h"
#include "Adafruit_TinyUSB.h"
#include <LiquidCrystal.h>
#include <SdFat.h>

// class Adafruit_InternalFlash : public FsBlockDeviceInterface { }

/**************************************************************************/
/*!
    @brief A helper class for making RP2040 "Tester Brains"
*/
/**************************************************************************/
class Adafruit_TestBed_Brains : public Adafruit_TestBed {
public:
  Adafruit_TestBed_Brains(void);
  void begin(void);

  void LCD_printf(bool linenum, const char format[], ...);
  void LCD_error(const char *errmsg1, const char *errmsg2);
  void LCD_info(const char *msg1, const char *msg2);

  bool SD_detected(void);
  bool SD_begin(uint32_t max_clock = SD_SCK_MHZ(16));

  void usbh_setVBus(bool en);
  bool usbh_begin(void);

  LiquidCrystal lcd = LiquidCrystal(7, 8, 9, 10, 11, 12);
  SdFs SD;
  SdSpiConfig SD_CONFIG = SdSpiConfig(17, SHARED_SPI, SD_SCK_MHZ(16));

  Adafruit_USBH_Host USBHost;

private:
  int _sd_detect_pin;
  int _sd_cs_pin;
  int _vbus_en_pin;
  int _usbh_dp_pin;
};

extern Adafruit_TestBed_Brains Brain;

#endif
#endif
