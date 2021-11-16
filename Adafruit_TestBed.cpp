#include "Adafruit_TestBed.h"

Adafruit_TestBed::Adafruit_TestBed(void) {
#if defined(ADAFRUIT_METRO_M0_EXPRESS)
  neopixelPin = 40;
  neopixelNum = 1;
#endif
}

void Adafruit_TestBed::begin(void) {
  millis_timestamp = millis();

  targetPower(0);
  if (neopixelNum > 0) {
    pixels = new Adafruit_NeoPixel(neopixelNum, neopixelPin, NEO_GRB + NEO_KHZ800);
    pixels->begin();
    pixels->show(); // Initialize all pixels to 'off'
    pixels->setBrightness(20);
  }

  if (piezoPin >= 0) {
    pinMode(piezoPin, OUTPUT);
  }
  if (ledPin >= 0) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }
}


uint32_t Adafruit_TestBed::timestamp(void) {
  uint32_t delta = millis() - millis_timestamp;
  millis_timestamp = millis();
  return delta;
}

void Adafruit_TestBed::printTimeTaken(bool restamp) {
  Serial.print(F("Took: ")); 
  Serial.print(millis() - millis_timestamp);
  Serial.println(F(" ms"));
  if (restamp) {
    timestamp();
  }
}

bool Adafruit_TestBed::testPullup(uint16_t pin, uint8_t inter_delay) {
  pinMode(pin, OUTPUT); 
  digitalWrite(pin, LOW);
  delay(inter_delay);
  pinMode(pin, INPUT);   
  delay(inter_delay);
  return(digitalRead(pin));
}

bool Adafruit_TestBed::scanI2CBus(byte addr, uint8_t post_delay) 
{
  theWire->begin();
  theWire->beginTransmission(addr);
  bool found = (theWire->endTransmission () == 0);
  delay(post_delay);
  return found;
}

void Adafruit_TestBed::printI2CBusScan(void) 
{
  theWire->begin();
  Serial.print("I2C scan: ");
  for (uint8_t addr=0x10; addr<=0x7F; addr++) {
    theWire->beginTransmission(addr);
    if (theWire->endTransmission () == 0) {
      Serial.print("0x");
      Serial.print(addr, HEX);
      Serial.print(", ");
    }
  }
  Serial.println();
}



void Adafruit_TestBed::targetPower(bool on) {
  if (targetPowerPin < 0) return;

  pinMode(targetPowerPin, OUTPUT);
  digitalWrite(targetPowerPin, on ? targetPowerPolarity : !targetPowerPolarity);
}

void Adafruit_TestBed::targetPowerCycle(uint16_t off_time) {
  targetPower(0);
  delay(off_time);
  targetPower(1);
}


void Adafruit_TestBed::setColor(uint32_t color) {
  if (! pixels) return;

  pixels->fill(color);
  pixels->show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Adafruit_TestBed::Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

Adafruit_TestBed TB;
