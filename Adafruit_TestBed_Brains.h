#ifndef ADAFRUIT_TESTBED_BRAINS_H
#define ADAFRUIT_TESTBED_BRAINS_H

#ifdef ARDUINO_ARCH_RP2040

#include "Adafruit_TestBed.h"
#include "Adafruit_TinyUSB.h"
#include <LiquidCrystal.h>
#include <SdFat.h>

/**************************************************************************/
/*!
    @brief A helper class for making RP2040 "Tester Brains"
*/
/**************************************************************************/
class Adafruit_TestBed_Brains : public Adafruit_TestBed {
public:
  Adafruit_TestBed_Brains(void);
  void begin(void);
  bool inited(void);

  // LCD
  void LCD_printf(bool linenum, const char format[], ...);
  void LCD_error(const char *errmsg1, const char *errmsg2);
  void LCD_info(const char *msg1, const char *msg2);

  // SD
  bool SD_detected(void);
  bool SD_begin(uint32_t max_clock = SD_SCK_MHZ(16));

  // USB Host
  void usbh_setVBus(bool en);
  bool usbh_begin(void);
  bool usbh_mountFS(uint8_t dev_addr);
  bool usbh_umountFS(uint8_t dev_addr);

  // Target
  void targetReset(uint32_t reset_ms);

  //------------- RP2040 target specific -------------//

  // reset rp2040 target to Boot ROM
  void rp2040_targetResetBootRom(int bootsel_pin = 28, uint32_t reset_ms = 10);

  // program rp2040 target by copying UF2 file from SDCard
  // return number of copied bytes (typically uf2 file size)
  size_t rp2040_programUF2(const char *fpath);

  //------------- Public Variables -------------//
  LiquidCrystal lcd = LiquidCrystal(7, 8, 9, 10, 11, 12);
  SdFat SD;
  SdSpiConfig SD_CONFIG = SdSpiConfig(17, SHARED_SPI, SD_SCK_MHZ(16));

  Adafruit_USBH_Host USBHost;
  FatVolume USBH_FS;
  Adafruit_USBH_MSC_BlockDevice USBH_BlockDev;

private:
  bool _inited;

  int _sd_detect_pin;
  int _sd_cs_pin;
  int _vbus_en_pin;
  int _usbh_dp_pin;

  int _target_rst;
  int _target_swdio;
  int _target_swdclk;
};

extern Adafruit_TestBed_Brains Brain;

#endif
#endif
