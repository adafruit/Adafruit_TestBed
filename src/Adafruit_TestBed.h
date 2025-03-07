#ifndef ADAFRUIT_TESTBED_H
#define ADAFRUIT_TESTBED_H

#include "Adafruit_NeoPixel.h"
#include "Arduino.h"
#include "Wire.h"

#include "ESP32BootROM.h"
#include "SdFat_Adafruit_Fork.h"

#define RED 0xFF0000
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define TEAL 0x00FFFF
#define PURPLE 0xFF00FF
#define WHITE 0xFFFFFF

typedef struct {
  const char *name;
  const uint8_t *data;
  const uint32_t compressed_len;
  const uint32_t uncompressed_len;
  const uint8_t md5[16];
} esp32_zipfile_t;

/**************************************************************************/
/*!
    @brief A helper class for making test beds and functions. Lots of handy lil
   tools!
*/
/**************************************************************************/
class Adafruit_TestBed {
public:
  Adafruit_TestBed(void);
  void begin(void);

  bool testPullup(uint16_t pin, uint8_t inter_delay = 1);

  void disableI2C(void);
  bool scanI2CBus(byte addr, uint8_t post_delay = 5);
  void printI2CBusScan(void);

  void targetPower(bool on);
  void targetPowerCycle(uint16_t off_time = 10);

  void targetReset(uint32_t reset_ms = 20);

  float readAnalogVoltage(uint16_t pin, float multiplier = 1);
  bool testAnalogVoltage(uint16_t pin, const char *name, float multiplier,
                         float value, uint8_t error = 10);

  bool testpins(uint8_t a, uint8_t b, uint8_t *allpins, uint8_t num_allpins);

  void setColor(uint32_t color);
  uint32_t Wheel(byte WheelPos);

  void setLED(uint8_t state);

  void beep(uint32_t freq, uint32_t duration);
  void beepNblink(void);
  void blink(void);

  uint32_t timestamp(void);
  void printTimeTaken(bool restamp = false);

  //--------------------------------------------------------------------+
  // ESP32 Target
  //--------------------------------------------------------------------+
  bool esp32_begin(ESP32BootROMClass *bootrom, uint32_t baudrate);
  void esp32_end(bool reset_esp = false);

  // program esp32 target with compressed data stored in internal flash
  size_t esp32_programFlashDefl(const esp32_zipfile_t *zfile, uint32_t addr);

  // program esp32 target with compressed file from SDCard
  size_t esp32_programFlashDefl(const esp32_zipfile_t *zfile, uint32_t addr,
                                File32 *fsrc);

  bool esp32_s3_inReset(void);
  void esp32_s3_clearReset(void);

  ESP32BootROMClass *esp32boot; // ESP32 ROM

  //////////////////
  TwoWire *theWire = &Wire;    ///< The I2C port used in scanning
  Stream *theSerial = &Serial; ///< The Serial port used for debugging

  float analogRef = 3.3;      ///< The default analog reference voltage
  uint16_t analogBits = 1024; ///< The default ADC resolution bits

  int16_t targetPowerPin = -1;     ///< Set to a target power pin if used
  bool targetPowerPolarity = HIGH; ///< What to set the power pin to, for ON

  int targetResetPin = -1; ///< Set to target reset pin if used

  int16_t neopixelPin = -1; ///< The neopixel connected pin if any
  uint8_t neopixelNum = 0;  ///< How many neopixels are on board, if any
  Adafruit_NeoPixel *pixels =
      NULL; ///< The strip we will auto-define if pixels exist

  int16_t piezoPin = -1; ///< The pin for a piezo buzzer, if it exists
  int16_t ledPin = -1;   ///< The pin for an LED, if it exists

private:
  uint32_t millis_timestamp = 0; ///< A general purpose timestamp

  bool _esp32_flash_defl;
  uint32_t _esp32_chip_detect;
  bool _esp32s3_in_reset;

  size_t _esp32_programFlashDefl_impl(const esp32_zipfile_t *zfile,
                                      uint32_t addr, File32 *fsrc);
};

extern Adafruit_TestBed TB;

#endif
