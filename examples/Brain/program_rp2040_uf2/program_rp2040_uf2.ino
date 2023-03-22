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

#include "Adafruit_TestBed_Brains.h"

// RP2040 Boot VID/PID
#define BOOT_VID   0x2e8a
#define BOOT_PID   0x0003

// UF2 file path on SDCard to copy
// #define UF2_FILE_PATH   "pico/adafruit-circuitpython-raspberry_pi_pico-en_US-7.3.3.uf2"
//#define UF2_FILE_PATH   "pico/blink.uf2"
#define UF2_FILE_PATH   "pico/tinyusb_dev_cdc_msc.uf2"

// If USB filesystem is mounted
volatile bool is_usbfs_mounted = false;

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
  // For debugging tinyusb
  if (CFG_TUSB_DEBUG) {
    Serial1.begin(115200);
  }

  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Tester Brains: UF2 copy from SD to USBH FS test!");

  // sync: wait for Brain.begin() called in core1 before accessing SD or other peripherals
  while (!Brain.inited()) delay(10);

  // prepare SD Card
  prepare_sd();

  // wait for USB filesystem is mounted. USB host bit-banging and task is
  // processed on core1
  while (!is_usbfs_mounted) delay(10);

  // Copy UF2 file
  Brain.LCD_printf(0, "Copying UF2 file");
  Serial.println("Copying from SD to USBHFS: " UF2_FILE_PATH);

  uint32_t ms = millis();
  size_t copied_bytes = Brain.rp2040_programUF2(UF2_FILE_PATH);

  print_speed(copied_bytes, millis() - ms);

  // wait for rp2040 boot rom to reset
  // while (is_usbfs_mounted) delay(10);
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

  // reset rp2040 target into Boot Rom, default bootsel = 28, reset duration = 10 ms
  Brain.rp2040_targetResetBootRom();
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
