// This sketch program ESP32 by flashing bin file from SD Card
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

#define ESP32_RESET     27
#define ESP32_IO0       28

#define ESP32_BAUDRATE  921600
// #define ESP32_BAUDRATE  115200

// RP2040 Boot VID/PID
#define BOOT_VID   0x2e8a
#define BOOT_PID   0x0003

// Bin file path on SDCard to copy
#define BIN_FILE_PATH   "esp32/NINA_W102-1.7.4.bin"

// Defined an boot rom object that use UART Serial1
ESP32BootROMClass ESP32BootROM(Serial1, ESP32_IO0, ESP32_RESET);

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void prepare_sd(void) {
  if (!Brain.SD_detected()) {
    Brain.LCD_printf(0, "No SD Card");
    while ( !Brain.SD_detected() ) delay(10);
  }

  if ( !Brain.SD_begin(SD_SCK_MHZ(16)) ) {
    Brain.LCD_printf(0, "SD init failed");
    while(1) delay(10);
  }

  Brain.LCD_printf(0, "SD mounted");

  // Print out file on SD if Serial is connected
  if (Serial) {
    Serial.println();
    Serial.println("SD Contents:");
    Serial.printf("Card size = %0.1f GB\n", 0.000000512 * Brain.SD.card()->sectorCount());
    Brain.SD.ls(LS_R | LS_DATE | LS_SIZE);
  }
}

void print_speed(size_t count, uint32_t ms) {
  Brain.LCD_printf(0, "%.01fKB %.01fs", count/1000.0F, ms / 1000.0F);

  Serial.printf("Completed %u bytes in %.02f seconds.\r\n", count, ms / 1000.0F);
  Serial.printf("Speed : %.02f KB/s\r\n", (count / 1000.0F) / (ms / 1000.0F));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Tester Brains: Programming ESP32 with UART!");

  // sync: wait for Brain.begin() called in core1 before accessing SD or other peripherals
  while (!Brain.inited()) delay(10);

  // prepare SD Card
  prepare_sd();

  while ( !Brain.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE) ) {
    // retry syncing
    delay(1000);
  }

  // Writing bin file
  Brain.LCD_printf("Flashing file..");

  uint32_t ms = millis();
  size_t wr_bytes = Brain.essp32_programFlash(BIN_FILE_PATH, 0);

  print_speed(wr_bytes, millis() - ms);
}

void loop() {
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

// call usbh_begin() here to make pio usb background task run on core1
// NOTE: Brain.begin() should be called here as well to prevent race condition
void setup1() {
  Brain.begin();
  Brain.usbh_begin();

  Brain.LCD_printf(1, "No USB Device");
}

// core1's loop: process usb host task on core1
void loop1() {
  Brain.USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
// Note: running in the same core where Brain.USBHost.task() is called
//--------------------------------------------------------------------+
extern "C"  {

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t dev_addr)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if ( !(vid == BOOT_VID && pid == BOOT_PID) ) {
    Brain.LCD_printf(1, "UnkDev %04x:%04x", vid, pid);
  }
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr)
{
  (void) dev_addr;
  Brain.LCD_printf(1, "No USB Device");
}

}
