#include <Adafruit_TestBed.h>
extern Adafruit_TestBed TB;

// This is almost always the default I2C port
#define DEFAULT_I2C_PORT &Wire

// Some boards have a second I2C port
#if defined(ARDUINO_ADAFRUIT_KB2040_RP2040)
  #define SECONDARY_I2C_PORT &Wire1
#endif

void setup() {
  Serial.begin(115200);
  
  // Wait for Serial port to open
  while (!Serial) {
    delay(10);
  }
  delay(500);
  Serial.println("Adafruit I2C Scanner");
}

void loop() {
  Serial.println("");
  Serial.println("");
  
  Serial.print("Default port ");
  TB.theWire = DEFAULT_I2C_PORT;
  TB.printI2CBusScan();

#if defined(SECONDARY_I2C_PORT)
  Serial.print("Secondary port ");
  TB.theWire = SECONDARY_I2C_PORT;
  TB.printI2CBusScan();
#endif

  delay(3000); // wait 3 seconds
}