#ifndef ADAFRUIT_TESTBED_BRAINS_H
#define ADAFRUIT_TESTBED_BRAINS_H

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "Adafruit_TestBed.h"
#include "SdFat.h"
#include <LiquidCrystal.h>

#include "Adafruit_DAP.h"
#include "Adafruit_TinyUSB.h"

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
  // printf on specified line
  void LCD_printf(uint8_t linenum, const char format[], ...);
  // printf on next line
  void LCD_printf(const char format[], ...);

  void LCD_error(const char *errmsg1, const char *errmsg2);
  void LCD_info(const char *msg1, const char *msg2);

  void setColor(uint32_t color);

  // SD
  bool SD_detected(void);
  bool SD_begin(uint32_t max_clock = SD_SCK_MHZ(16));

  // USB Host
  void usbh_setVBus(bool en);
  bool usbh_begin(void);
  bool usbh_inited(void);
  bool usbh_mountFS(uint8_t dev_addr);
  bool usbh_umountFS(uint8_t dev_addr);

  // Target
  void targetReset(uint32_t reset_ms = 20);

  //------------- RP2040 target -------------//
  // reset rp2040 target to Boot ROM
  void rp2040_targetResetBootRom(int bootsel_pin = 28, uint32_t reset_ms = 20);

  // program rp2040 target by copying UF2 file from SDCard
  // return number of copied bytes (typically uf2 file size)
  size_t rp2040_programUF2(const char *fpath);

  //------------- SAMD21 target -------------//
  bool dap_begin(Adafruit_DAP *dp);
  bool dap_connect(void);
  void dap_disconnect(void);

  bool dap_unprotectBoot(void);
  bool dap_protectBoot(void);

  bool dap_eraseChip(void);

  // program dap target with file from SDCard
  // return number of programmed bytes
  size_t dap_programFlash(const char *fpath, uint32_t addr);

  //------------- Public Variables -------------//
  LiquidCrystal lcd = LiquidCrystal(7, 8, 9, 10, 11, 12);
  SdFat SD;
  SdSpiConfig SD_CONFIG = SdSpiConfig(17, SHARED_SPI, SD_SCK_MHZ(16));

  // USB host
  Adafruit_USBH_Host USBHost;

  // Host MSC
  FatVolume USBH_FS;
  Adafruit_USBH_MSC_BlockDevice USBH_BlockDev;

  Adafruit_DAP *dap;

private:
  bool _inited;
  uint8_t _lcd_line;

  int _sd_detect_pin;
  int _sd_cs_pin;
  int _vbus_en_pin;
  int _usbh_dp_pin;

  int _target_rst;
  int _target_swdio;
  int _target_swdclk;

  void lcd_write(uint8_t linenum, char buf[17]);
};

extern Adafruit_TestBed_Brains Brain;

#endif
#endif
