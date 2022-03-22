#include <Adafruit_TestBed.h>
#include <Adafruit_MCP4725.h>

extern Adafruit_TestBed TB;
Adafruit_MCP4725 dac;

#define ADDR A0

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello MCP4725 tester!");
  TB.piezoPin = 5;
  TB.ledPin = 11;

  TB.begin();
}

void loop(void) {
  pinMode(ADDR, INPUT);

  TB.disableI2C();

  if (! TB.testPullup(A5) || ! TB.testPullup(A4)) {
    Serial.println("Waiting for I2C pullups");
    return;
  }

  Serial.println("I2C pullups OK");  // pullups cool - next up!

  if (!TB.scanI2CBus(0x62)) {
    Serial.println("I2C 0x62 not found");
    return;
  }
  Serial.println("I2C 0x62 found");

  delay(10);
  dac.begin(0x62);

  dac.setVoltage(0, false);
  if (! TB.testAnalogVoltage(A1, "Zero DAC", 1, 0.))
    return;
  dac.setVoltage(2048, false);
  if (! TB.testAnalogVoltage(A1, "Half DAC", 1, 2.5))
    return;
  dac.setVoltage(4095, false);
  if (! TB.testAnalogVoltage(A1, "Full DAC", 1, 5))
    return;

  // check addr
  pinMode(ADDR, OUTPUT);
  digitalWrite(ADDR, HIGH);
  delay(10);
  if (!TB.scanI2CBus(0x63)) {
    Serial.println("I2C 0x63 not found");
  }
  Serial.println("I2C 0x63 found");
  
  Serial.println("Test OK!");
  //TB.beepNblink();
  delay(500);
}
