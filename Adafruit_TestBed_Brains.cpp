#include "Adafruit_TestBed_Brains.h"



/**************************************************************************/
/*!
    @brief  Initializer, sets up the timestamp, neopixels, piezo, led,
            and analog reference. So get all pins assigned before calling
*/
/**************************************************************************/

Adafruit_TestBed_Brains::Adafruit_TestBed_Brains() {
  neopixelNum = 1;  // LCD backlight
  neopixelPin = 13; // LCD backlight

  piezoPin = 15; // onboard buzzer
  ledPin = 25;   // green LED on Pico

  targetPowerPin = 6;  // VBat switch
}

void Adafruit_TestBed_Brains::begin(void) {
  Adafruit_TestBed::begin();

  pixels->setBrightness(255);
  setColor(0xFFFFFF);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();
  lcd.noCursor();


  if (SD_detected()) {
    Serial.print("SD inserted...");
    if (! SD.begin(17, SD_SCK_MHZ(4))) {
      Serial.println("Could not init SD!");
    } else {
      uint32_t SDsize = SD.card()->sectorCount();
      if (SDsize == 0) {
        Serial.println("Can't determine the card size");
      } else {
        Serial.printf("Card size = %0.1f GB\n", 0.000000512 * (float)SDsize);
        Serial.println("Files found (date time size name):");
        SD.ls(LS_R | LS_DATE | LS_SIZE);
      }
    }
  }
}

void Adafruit_TestBed_Brains::LCD_printf(bool linenum, const char format[], ...) {
  char linebuf[17];
  memset(linebuf, 0, 17);

  va_list ap;
  va_start(ap, format);
  vsnprintf(linebuf, sizeof(linebuf), format, ap);

  // fill the rest with spaces
  memset(linebuf+strlen(linebuf), ' ', 16 - strlen(linebuf));
  linebuf[16] = 0;
  lcd.setCursor(0, linenum);
  lcd.write(linebuf);
  va_end(ap);
}


bool Adafruit_TestBed_Brains::SD_detected(void) {
  pinMode(14, INPUT_PULLUP);
  return digitalRead(14);
}


Adafruit_TestBed_Brains Brain;
