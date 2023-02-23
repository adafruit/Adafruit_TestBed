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

#include "esp_binaries.h"

#define ESP32_RESET     27
#define ESP32_IO0       28

#define ESP32_BAUDRATE  2000000
//#define ESP32_BAUDRATE  921600
//#define ESP32_BAUDRATE  115200

// Bin files header to program
#define BIN_WIFI_AP_SKETCH    0
#define BIN_NINA_1_7_4        1

// select which bins to flash
#define BIN_FILES             BIN_WIFI_AP_SKETCH

// Compressed binaries files generated by tools/esp_compress.py
struct {
  uint32_t addr;
  esp32_zipfile_t const * zfile;
} bin_list[] = {

#if BIN_FILES == BIN_WIFI_AP_SKETCH
  { 0x10000, &WiFiAccessPoint_ino            },
  { 0x1000 , &WiFiAccessPoint_ino_bootloader },
  { 0x8000 , &WiFiAccessPoint_ino_partitions },
  { 0xe000 , &boot_app0                      },
#elif BIN_FILES == BIN_NINA_1_7_4
  { 0x00000, &NINA_W102_1_7_4            },
#endif
};

enum {
  BIN_LIST_COUNT = sizeof(bin_list)/sizeof(bin_list[0])
};


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

  while ( !Brain.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE) ) {
    // retry syncing
    delay(100);
  }

  // Writing bin files
  size_t total_bytes = 0;
  uint32_t ms = millis();
  for(size_t i=0; i<BIN_LIST_COUNT; i++) {
    Brain.LCD_printf(0, "Flashing file %u", i);
    size_t wr_count = Brain.esp32_programFlashDefl(bin_list[i].zfile, bin_list[i].addr);
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

