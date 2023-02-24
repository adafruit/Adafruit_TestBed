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

// Change BIN_FILES in esp_binaries.h header to select which binaries to flash
// bin_files[] is defined accordingly
#include "esp_binaries.h"

#define ESP32_RESET     27
#define ESP32_IO0       28

// Using CDC baudrate does not really matter at all
#define ESP32_BAUDRATE  115200

// CDC Host object
Adafruit_USBH_CDC  SerialHost;

// Defined an boot rom object that use UART Serial1
ESP32BootROMClass ESP32BootROM(SerialHost, ESP32_IO0, ESP32_RESET);

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
  Serial.println("Tester Brains: Programming ESP32 with SerialHost!");

  // sync: wait for Brain.begin() called in core1 before accessing SD or other peripherals
  while (!Brain.inited()) delay(10);

  while ( !Brain.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE) ) {
    delay(100);
  }

  // Writing bin files
  size_t total_bytes = 0;
  uint32_t ms = millis();
  for(size_t i=0; i<BIN_FILES_COUNT; i++) {
    Brain.LCD_printf("Flashing file %u", i);
    Serial.printf("File %s\r\n", bin_files[i].zfile->name);
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
  Brain.begin();
  Brain.usbh_begin();

  // Since we only support 1 CDC interface with Tester (also CFG_TUH_CDC = 1)
  // the index will always be 0 for SerialHost
  // SerialHost.setInterfaceIndex(0);
  SerialHost.begin(115200);

  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");
}

// core1's loop: process usb host task on core1
void loop1() {
  Brain.USBHost.task();

  // periodically flush SerialHost if connected
  if ( SerialHost && SerialHost.connected() ) {
    SerialHost.flush();
  }

  yield();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
// Note: running in the same core where Brain.USBHost.task() is called
//--------------------------------------------------------------------+
extern "C"  {

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t daddr)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(daddr, &vid, &pid);

  Serial.printf("Device attached, address = %d\r\n", daddr);
  Brain.LCD_printf("USBID %04x:%04x", vid, pid);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr)
{
  (void) dev_addr;
  Brain.LCD_printf(1, "No USB Device");
}

}
