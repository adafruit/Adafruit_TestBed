// This sketch program SAMD with bin file from SDCard using Adafruit_DAP
// Hardware wiring:
// - Brain's header Reset <-> Target Reset
// - Brain's header SWDIO <-> Target SWDIO
// - Brain's header SWCLK <-> Target SWCLK
// - Brain's USB host to Target USB

// required for Host MSC block device
#include "SdFat.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

// required for DAP programming
#include "Adafruit_DAP.h"

#include "Adafruit_TestBed_Brains.h"

// RP2040 Boot VID/PID
#define BOOT_VID   0x2e8a
#define BOOT_PID   0x0003

// file path on SDCard to prograom
#define TEST_FILE_PATH "stm32f4/feather_stm32f405/tinyuf2-feather_stm32f405_express.bin"

// DAP interface for nRF5x
Adafruit_DAP_STM32 dap;

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void print_speed(size_t count, uint32_t ms) {
  Brain.LCD_printf(0, "%.01fKB in %.01fs", count/1000.0F, ms / 1000.0F);

  Serial.printf("Completed %u bytes in %.02f seconds.\r\n", count, ms / 1000.0F);
  Serial.printf("Speed : %.02f KB/s\r\n", (count / 1000.0F) / (ms / 1000.0F));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Tester Brains: STM32F405 programming !");

  // sync: wait for Brain.usbh_begin() called in core1 before accessing SD or other peripherals
  while (!Brain.usbh_inited()) delay(10);

  Brain.SD_begin(SD_SCK_MHZ(16));

  // Print out file on SD if Serial is connected
  if (Serial) {
    Serial.println();
    Serial.println("SD Contents:");
    Serial.printf("Card size = %0.1f GB\n", 0.000000512 * Brain.SD.card()->sectorCount());
    Brain.SD.ls(LS_R | LS_SIZE);
  }

  Brain.targetReset();

  Brain.dap_begin(&dap);
  Brain.dap_connect();

  Brain.dap_unprotectBoot();

  // erase chip before programming
  Brain.dap_eraseChip();

  uint32_t ms = millis();
  size_t copied_bytes = Brain.dap_programFlash(TEST_FILE_PATH, 0);
  ms = millis() - ms;
  print_speed(copied_bytes, ms);

  Brain.dap_protectBoot();
  Brain.dap_disconnect();

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
