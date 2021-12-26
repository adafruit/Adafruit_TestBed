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
    pixels =
        new Adafruit_NeoPixel(neopixelNum, neopixelPin, NEO_GRB + NEO_KHZ800);
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

#if defined(__AVR__)
  analogRef = 5.0;
#elif defined(ARDUINO_ARCH_RP2040)
  analogBits = 4096;
#elif defined(ARDUINO_ARCH_ESP32)
  analogBits = 4096;
  analogReadResolution(12);
  analogRef = 2.4;
#endif
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
  return (digitalRead(pin));
}

bool Adafruit_TestBed::scanI2CBus(byte addr, uint8_t post_delay) {
  theWire->begin();
  theWire->beginTransmission(addr);
  bool found = (theWire->endTransmission() == 0);
  delay(post_delay);
  return found;
}

void Adafruit_TestBed::printI2CBusScan(void) {
  theWire->begin();
  Serial.print("I2C scan: ");
  for (uint8_t addr = 0x00; addr <= 0x7F; addr++) {
    theWire->beginTransmission(addr);
    if (theWire->endTransmission() == 0) {
      Serial.print("0x");
      Serial.print(addr, HEX);
      Serial.print(", ");
    }
  }
  Serial.println();
}

void Adafruit_TestBed::targetPower(bool on) {
  if (targetPowerPin < 0)
    return;

  pinMode(targetPowerPin, OUTPUT);
  digitalWrite(targetPowerPin, on ? targetPowerPolarity : !targetPowerPolarity);
}

void Adafruit_TestBed::targetPowerCycle(uint16_t off_time) {
  targetPower(0);
  delay(off_time);
  targetPower(1);
}

float Adafruit_TestBed::readAnalogVoltage(uint16_t pin, float multiplier) {
  float x = analogRead(pin);
  Serial.println(x);
  x /= analogBits;
  x *= analogRef;
  x *= multiplier;
  return x;
}

bool Adafruit_TestBed::testAnalogVoltage(uint16_t pin, const char *name,
                                         float multiplier, float value) {
  float voltage = readAnalogVoltage(pin, multiplier);
  Serial.print(name);
  Serial.print(F(" output voltage: "));
  Serial.print(voltage);
  Serial.print(F(" V (should be "));
  Serial.print(value);
  Serial.print(" V)...");
  if (abs(voltage - value) > (value / 10.0)) {
    Serial.println("Failed");
    return false;
  }
  Serial.println(F("OK within 10%"));

  return true;
}

bool Adafruit_TestBed::testpins(uint8_t a, uint8_t b, uint8_t *allpins,
                                uint8_t num_allpins) {

  Serial.print(F("\tTesting "));
  Serial.print(a, DEC);
  Serial.print(F(" & "));
  Serial.println(b, DEC);

  // set both to inputs
  pinMode(b, INPUT);
  // turn on 'a' pullup
  pinMode(a, INPUT_PULLUP);
  delay(1);

  // verify neither are grounded
  if (!digitalRead(a) || !digitalRead(b)) {
    Serial.println(F("Ground test 1 fail: both pins should not be grounded"));
    // while (1) yield();
    return false;
  }

  // turn off both pullups
  pinMode(a, INPUT);
  pinMode(b, INPUT);

  // make a an output
  pinMode(a, OUTPUT);
  digitalWrite(a, LOW);
  delay(1);

  int ar = digitalRead(a);
  int br = digitalRead(b);

  // make sure both are low
  if (ar || br) {
    Serial.print(F("Low test fail on "));
    if (br)
      Serial.println(b, DEC);
    if (ar)
      Serial.println(a, DEC);
    return false;
  }

  // a is an input, b is an output
  pinMode(a, INPUT);
  pinMode(b, OUTPUT);
  digitalWrite(b, HIGH);
  delay(10);

  // verify neither are grounded
  if (!digitalRead(a)
#if !defined(ESP32)
      || !digitalRead(b)
#endif
  ) {
    Serial.println(F("Ground test 2 fail: both pins should not be grounded"));
    delay(100);
    return false;
  }

  // make sure no pins are shorted to pin a or b
  for (uint8_t i = 0; i < num_allpins; i++) {
    pinMode(allpins[i], INPUT_PULLUP);
  }

  pinMode(a, OUTPUT);
  digitalWrite(a, LOW);
  pinMode(b, OUTPUT);
  digitalWrite(b, LOW);
  delay(1);

  for (uint8_t i = 0; i < sizeof(allpins); i++) {
    if ((allpins[i] == a) || (allpins[i] == b)) {
      continue;
    }

    // Serial.print("Pin #"); Serial.print(allpins[i]);
    // Serial.print(" -> ");
    // Serial.println(digitalRead(allpins[i]));
    if (!digitalRead(allpins[i])) {
      Serial.print(allpins[i]);
      Serial.println(F(" is shorted?"));

      return false;
    }
  }
  pinMode(a, INPUT);
  pinMode(b, INPUT);

  delay(10);

  return true;
}

void Adafruit_TestBed::setColor(uint32_t color) {
  if (!pixels)
    return;

  pixels->fill(color);
  pixels->show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Adafruit_TestBed::Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void Adafruit_TestBed::disableI2C(void) {
#if defined(__AVR__)
  TWCR = 0;
#elif defined(ESP32) || defined(ESP8266)
  // nothing!
#else
  theWire->end();
#endif
}

void Adafruit_TestBed::beep(uint32_t freq, uint32_t duration) {
  if (piezoPin < 0)
    return;
  pinMode(piezoPin, OUTPUT);
#if !defined(ARDUINO_ARCH_ESP32)
  tone(piezoPin, freq, duration);
#endif
}

void Adafruit_TestBed::beepNblink(void) {
  if (ledPin >= 0) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
  }

  beep(2000, 250);

  delay(500);

  if (ledPin >= 0) {
    digitalWrite(ledPin, LOW);
  }
}

Adafruit_TestBed TB;
