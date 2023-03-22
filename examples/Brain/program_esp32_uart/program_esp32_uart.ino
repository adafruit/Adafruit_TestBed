// This sketch program ESP32 by flashing bin file from on-flash (with .h header)
// Hardware wiring:
// - Brain GPIO28 <-> ESP32 IO0
// - Brain Reset  <-> ESP32 Enable
// - Brain TX/RX  <-> ESP32 RX/TX

// required for Host MSC block device
#include "SdFat.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

// Change BIN_FILES in esp_binaries.h header to select which binaries to flash
// bin_files[] is defined accordingly
#include "esp_binaries.h"

#define ESP32_RESET     27
#define ESP32_IO0       28

#define ESP32_BAUDRATE  2000000
//#define ESP32_BAUDRATE  1500000
//#define ESP32_BAUDRATE  921600
//#define ESP32_BAUDRATE  115200


// Defined an boot rom object that use UART Serial1
ESP32BootROMClass ESP32BootROM(Serial1, ESP32_IO0, ESP32_RESET);

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void print_speed(size_t count, uint32_t ms) {
  float count_k = count / 1000.0F;
  float sec = ms / 1000.0F;
  float speed = count_k / sec;
  Brain.LCD_printf(0, "%.01fKB %.01fs", count_k, sec);
  Brain.LCD_printf(1, "Spd: %.01f KB/s", speed);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Tester Brains: Programming ESP32 with UART!");

  Brain.begin();
  Brain.usbh_setVBus(true); // enable VBUS for power

  while ( !Brain.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE) ) {
    // retry syncing
    delay(100);
  }

  // Writing bin files
  size_t total_bytes = 0;
  uint32_t ms = millis();
  for(size_t i=0; i<BIN_FILES_COUNT; i++) {
    Brain.LCD_printf(0, "Flashing file %u", i);
    Serial.printf("File: %s\r\n", bin_files[i].zfile->name);
    size_t wr_count = Brain.esp32_programFlashDefl(bin_files[i].zfile, bin_files[i].addr);
    total_bytes += wr_count;
    if (!wr_count) {
      Brain.LCD_printf_error("Failed to flash");
    }
  }
  print_speed(total_bytes, millis() - ms);

  Brain.esp32_end();

  // reset ESP32 to run new firmware
  Brain.targetReset();
}

void loop() {
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

// call usbh_begin() here to make pio usb background task run on core1
// NOTE: Brain.begin() should be called here as well to prevent race condition
void setup1() {

}

// core1's loop: process usb host task on core1
void loop1() {
  yield();
}
