#ifndef ADAFRUIT_TESTBED_BRAINS_H
#define ADAFRUIT_TESTBED_BRAINS_H

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "Adafruit_TestBed.h"
#include "SdFat.h"
#include <LiquidCrystal.h>

#include "Adafruit_DAP.h"
#include "Adafruit_TinyUSB.h"

#include "ESP32BootROM.h"
#include "MD5Builder.h"

typedef struct {
  const char *name;
  const uint8_t *data;
  const uint32_t compressed_len;
  const uint32_t uncompressed_len;
  const uint8_t md5[16];
} esp32_zipfile_t;

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

  //------------- LCD -------------//

  // printf on specific line
  void LCD_printf(uint8_t linenum, const char format[], ...);

  // printf on next line
  void LCD_printf(const char format[], ...);

  // printf on error (RGB = RED)
  void LCD_printf_error(const char format[], ...);

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

  //--------------------------------------------------------------------+
  // RP2040 Target
  //--------------------------------------------------------------------+
  // reset rp2040 target to Boot ROM
  void rp2040_targetResetBootRom(int bootsel_pin = 28, uint32_t reset_ms = 20);

  // program rp2040 target by copying UF2 file from SDCard
  // return number of copied bytes (typically uf2 file size)
  size_t rp2040_programUF2(const char *fpath);

  //--------------------------------------------------------------------+
  // DAP (samd21/51, nrf5x, stm32f4 etc..) Target
  //--------------------------------------------------------------------+
  bool dap_begin(Adafruit_DAP *dp);
  bool dap_connect(void);
  void dap_disconnect(void);

  bool dap_unprotectBoot(void);
  bool dap_protectBoot(void);

  bool dap_eraseChip(void);

  // program dap target with file from SDCard
  // return number of programmed bytes
  size_t dap_programFlash(const char *fpath, uint32_t addr);

  //--------------------------------------------------------------------+
  // ESP32 Target
  //--------------------------------------------------------------------+

  bool esp32_begin(ESP32BootROMClass *bootrom, uint32_t baudrate);
  void esp32_end(bool reset_esp = false);

  // program esp32 target with file from SDCard
  // return number of programmed bytes
  size_t esp32_programFlash(const char *fpath, uint32_t addr);

  // program flash with compressed using zipfile struct
  size_t esp32_programFlashDefl(const esp32_zipfile_t *zfile, uint32_t addr);

  bool esp32_s3_inReset(void);
  void esp32_s3_clearReset(void);

  //--------------------------------------------------------------------+
  // Public Variables
  //--------------------------------------------------------------------+

  LiquidCrystal lcd = LiquidCrystal(7, 8, 9, 10, 11, 12);
  SdFat SD;
  SdSpiConfig SD_CONFIG = SdSpiConfig(17, SHARED_SPI, SD_SCK_MHZ(16));

  // USB host
  Adafruit_USBH_Host USBHost;

  // Host MSC
  FatVolume USBH_FS;
  Adafruit_USBH_MSC_BlockDevice USBH_BlockDev;

  // Dap
  Adafruit_DAP *dap;

  // ESP32 ROM
  ESP32BootROMClass *esp32boot;

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

  bool _esp32_flash_defl;
  uint32_t _esp32_chip_detect;
  bool _esp32s3_in_reset;

  void lcd_write(uint8_t linenum, char buf[17]);
};

extern Adafruit_TestBed_Brains Brain;

#endif
#endif
