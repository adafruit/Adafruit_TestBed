#ifndef ADAFRUIT_TESTBED_H
#define ADAFRUIT_TESTBED_H

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_NeoPixel.h"


#define RED 0xFF0000
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define TEAL 0x00FFFF
#define PURPLE 0xFF00FF
#define WHITE 0xFFFFFF

class Adafruit_TestBed  {
public:
  Adafruit_TestBed(void);
  void begin(void);

  bool testPullup(uint16_t pin, uint8_t inter_delay=1);

  bool scanI2CBus(byte addr, uint8_t post_delay=5);
  void printI2CBusScan(void);
  
  void targetPower(bool on);
  void targetPowerCycle(uint16_t off_time=10);

  void setColor(uint32_t color);
  uint32_t Wheel(byte WheelPos);

  uint32_t timestamp(void);
  void printTimeTaken(bool restamp=false);


  //////////////////
  TwoWire *theWire = &Wire;
  uint32_t millis_timestamp = 0;

  int16_t targetPowerPin = -1;
  bool targetPowerPolarity = HIGH;

  int16_t neopixelPin = -1;
  uint8_t neopixelNum = 0;
  Adafruit_NeoPixel *pixels = NULL;

  int16_t piezoPin = -1;
  int16_t ledPin = -1;

};

#endif
