// Testing basic peripherals on Brain

// required for Host MSC block device
#include "SdFat.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Hello world, Tester Brains self test!");

  Brain.begin();
  Brain.LCD_printf(0, "RP2040 Tester Brain");
  Brain.LCD_printf(1,  __DATE__ __TIME__);
}

uint8_t i=0;
void loop() {
  if (i == 0) {
    // approx once every 2 seconds
    Brain.LCD_printf(0, "Testing: %d", millis()/1000);
    Brain.printI2CBusScan();

    if (Brain.SD_detected()) {
      Serial.print("SD inserted...");
      if (! Brain.SD.begin(17, SD_SCK_MHZ(16))) {
        Serial.println("Could not init SD!");
        return;
      }
      uint32_t SDsize = Brain.SD.card()->sectorCount();
      if (SDsize == 0) {
        Serial.println("Can't determine the card size");
        return;
      }
      Serial.printf("Card size = %0.1f GB\n", 0.000000512 * (float)SDsize);
      Serial.println("Files found (date time size name):");
      Brain.SD.ls(LS_R | LS_DATE | LS_SIZE);
    }
  }

  Brain.setColor(Brain.Wheel(i));
  //Brain.beepNblink();

  i++;
  delay(10);
}
