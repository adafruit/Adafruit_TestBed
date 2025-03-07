/* This sketch run on SAMD21/SAMD51 to program ESP32 by flashing bin file from SD Card
 * Supported/tested ESP MCU are: ESP32, ESP32-S2, ESP32-S3, ESP8266
 * Hardware wiring:
 * - TB D2    <-> ESP32 IO0
 * - TB D3    <-> ESP32 Enable
 * - TB TX/RX <-> ESP32 RX/TX
 *
 * How to run this example:
 * 0. Define ESP32_RESET, ESP32_IO0, SD_CS, SD_DETECT in this sketch according to your hardware setup
 * 1. Generate compressed binary and its metadata (len, md5 etc..) by running:
 *    python3 tools/esp_compress.py --sd <directory_of_bin_files>
 *    For example: python tools/esp_compress.py --sd .
 * 2. .bin.gz (e.g NINA_W102-1.7.5.bin.gz) and esp_binaries.h will be generated in the same directory
 * 3. Copy esp_binaries.h to this example directory
 * 4. Copy .bin.gz files to SD Card within BIN_DIR (defined in this sketch)
 * 5. Insert SD Card to board
 * 6. Upload this sketch to board
 *
 * Note: for convenience, this example included generated 'NINA_W102-1.7.5.bin.gz' and 'esp_binaries.h'
 * from https://github.com/adafruit/nina-fw/releases/tag/1.7.5 in this example directory.
 */

#include "SdFat_Adafruit_Fork.h"

// #include "Adafruit_TinyUSB.h"
#include "Adafruit_TestBed.h"

//--------------------------------------------------------------------+
// Hardware Configuration
//--------------------------------------------------------------------+

// These are defined in airlift-capable board such PyPortal, PyBadge, etc.
#if defined(ESP32_GPIO0) && defined(ESP32_RESETN)
  #define ESP32_RESET  ESP32_RESETN
  #define ESP32_IO0    ESP32_GPIO0
#else
  #define ESP32_RESET  2
  #define ESP32_IO0    3
#endif

#if defined(ARDUINO_PYPORTAL_M4)
  #define SD_CS        32
  #define SD_DETECT    33
#else
  #define SD_CS        4
  #define SD_DETECT    5 // optional
#endif

#define ESP32_BAUDRATE  2000000
//#define ESP32_BAUDRATE  1500000
//#define ESP32_BAUDRATE  921600
//#define ESP32_BAUDRATE  115200

//--------------------------------------------------------------------+
// Binaries and Path
//--------------------------------------------------------------------+
#define BIN_DIR "/" // SD card's directory that contains bin files
#include "esp_binaries.h"

struct {
  uint32_t addr;
  esp32_zipfile_t const *zfile;
} bin_files[] = {
 {0x00000, &NINA_W102_1_7_5}
};

enum { BIN_FILES_COUNT = sizeof(bin_files) / sizeof(bin_files[0]) };

//--------------------------------------------------------------------+
// Defined an boot rom object that use UART Serial1
ESP32BootROMClass ESP32BootROM(Serial1, ESP32_IO0, ESP32_RESET);

SdFat SD;
File32 fbin;

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+

void print_speed(size_t count, uint32_t ms) {
  float count_k = count / 1000.0F;
  float sec = ms / 1000.0F;
  float speed = count_k / sec;

  Serial.print(count_k); Serial.print("KB "); Serial.print(sec); Serial.println("s");
  Serial.print("Spd: "); Serial.print(speed); Serial.println(" KB/s");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TestBed: Programming ESP32 with UART!");

#ifdef SD_DETECT
  pinMode(SD_DETECT, INPUT_PULLUP);
  if (digitalRead(SD_DETECT) == LOW) {
    Serial.println("SD Card not inserted");
    while (1) delay(10);
  }
#endif

  if (!SD.begin(SD_CS, SD_SCK_MHZ(12))) {
    Serial.println("SD Card init failed");
    while (1) delay(10);
  }
  Serial.println("SD Card init OK");

  TB.begin();
  while ( !TB.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE) ) {
    // retry syncing
    delay(100);
  }

  // Writing bin files
  size_t total_bytes = 0;
  uint32_t ms = millis();
  for(size_t i=0; i<BIN_FILES_COUNT; i++) {
    Serial.printf("Flashing file %u\r\n", i);
    Serial.printf("File: %s\r\n", bin_files[i].zfile->name);

    char bin_path[128] = BIN_DIR;
    strcat(bin_path, bin_files[i].zfile->name);

    fbin.open(&SD, bin_path);
    if (!fbin) {
      Serial.printf("Failed to open file %s\r\n", bin_files[i].zfile->name);
      continue;
    }

    size_t wr_count = TB.esp32_programFlashDefl(bin_files[i].zfile, bin_files[i].addr, &fbin);
    total_bytes += wr_count;
    if (!wr_count) {
      Serial.printf("Failed to flash");
    }

    fbin.close();
  }
  print_speed(total_bytes, millis() - ms);

  TB.esp32_end();

  // reset ESP32 to run new firmware
  TB.targetReset();
}
void loop() {
}
