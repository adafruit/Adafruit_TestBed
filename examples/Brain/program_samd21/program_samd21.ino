// This sketch program rp2040 by copying UF2 file from SDCard to
// rp2040 bootrom
// Hardware wiring:
// - Brain's Target GPIO28 to RP2040 bootsel
// - Brain's Target Reset to RP2040 Reset
// - Brain's USB host to RP2040 USB interface

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
#define TEST_FILE_PATH "samd21/metro/3505test.bin"
//#define TESTFILECRC  0x9709b384

// If USB filesystem is mounted
volatile bool is_usbfs_mounted = false;

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
  Serial.println("Tester Brains: SAMD21 programming !");

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

  uint32_t ms = millis();

  Brain.targetReset();

  Brain.samd21_connectDAP();
  size_t copied_bytes = Brain.samd21_programFlash(TEST_FILE_PATH, 0);
  Brain.samd21_disconnectDAP();

  Brain.targetReset();

  print_speed(copied_bytes, millis() - ms);
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

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if ( vid == BOOT_VID && pid == BOOT_PID ) {
    is_usbfs_mounted = Brain.usbh_mountFS(dev_addr);
    if (is_usbfs_mounted) {
      Brain.LCD_printf(1, "RP2 Boot mounted");
    }
  }
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(uint8_t dev_addr)
{
  is_usbfs_mounted = false;
  Brain.usbh_umountFS(dev_addr);
}

}
