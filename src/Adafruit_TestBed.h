#ifndef ADAFRUIT_TESTBED_H
#define ADAFRUIT_TESTBED_H

#include "Adafruit_NeoPixel.h"
#include "Arduino.h"
#include "Wire.h"

#define RED 0xFF0000
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define TEAL 0x00FFFF
#define PURPLE 0xFF00FF
#define WHITE 0xFFFFFF

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

  float readAnalogVoltage(uint16_t pin, float multiplier = 1);
  bool testAnalogVoltage(uint16_t pin, const char *name, float multiplier,
                         float value);

  bool testpins(uint8_t a, uint8_t b, uint8_t *allpins, uint8_t num_allpins);

  void setColor(uint32_t color);
  uint32_t Wheel(byte WheelPos);

  void setLED(uint8_t state);

  void beep(uint32_t freq, uint32_t duration);
  void beepNblink(void);

  uint32_t timestamp(void);
  void printTimeTaken(bool restamp = false);

  //////////////////
  TwoWire *theWire = &Wire;    ///< The I2C port used in scanning
  Stream *theSerial = &Serial; ///< The Serial port used for debugging

  float analogRef = 3.3;      ///< The default analog reference voltage
  uint16_t analogBits = 1024; ///< The default ADC resolution bits

  int16_t targetPowerPin = -1;     ///< Set to a target power pin if used
  bool targetPowerPolarity = HIGH; ///< What to set the power pin to, for ON

  int16_t neopixelPin = -1; ///< The neopixel connected pin if any
  uint8_t neopixelNum = 0;  ///< How many neopixels are on board, if any
  Adafruit_NeoPixel *pixels =
      NULL; ///< The strip we will auto-define if pixels exist

  int16_t piezoPin = -1; ///< The pin for a piezo buzzer, if it exists
  int16_t ledPin = -1;   ///< The pin for an LED, if it exists

private:
  uint32_t millis_timestamp = 0; ///< A general purpose timestamp
};

#endif
