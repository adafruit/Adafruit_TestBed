// This sketch program rp2040 by copying UF2 file from SDCard to
// rp2040 bootrom
// Hardware wiring:
// - Brain's Target GPIO28 to RP2 bootsel
// - Brain's Target Reset to RP2 Reset
// - Brain's USB host to RP2040 USB interface

// required for Host MSC block device
#include "SdFat_Adafruit_Fork.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

// firmware.h is converted using tools/file2carray.py e.g
//   python tools/file2carray.py cdc_msc.uf2
// above command will generate cdc_msc.uf2.h with bindata, bindata_len
#include "metro_rp2350_cdc_msc.uf2.h"

// RP2040 Boot VID/PID
#define BOOT_VID   0x2e8a
#define BOOT_PID_RP2040   0x0003
#define BOOT_PID_RP2350   0x000f

// If USB filesystem is mounted
volatile bool is_usbfs_mounted = false;

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+
void print_speed(size_t count, uint32_t ms) {
  Brain.LCD_printf(0, "%.01fKB %.01fs", count/1000.0F, ms / 1000.0F);

  Serial.printf("Completed %u bytes in %.02f seconds.\r\n", count, ms / 1000.0F);
  Serial.printf("Speed : %.02f KB/s\r\n", (count / 1000.0F) / (ms / 1000.0F));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Program RP2 by copy UF2 from Internal Flash to Bootloader!");

  // sync: wait for Brain.begin() called in core1 before accessing SD or other peripherals
  while (!Brain.inited()) delay(10);

  // wait for USB filesystem is mounted. USB host bit-banging and task is
  // processed on core1
  while (!is_usbfs_mounted) delay(10);

  // Copy UF2 file
  Brain.LCD_printf(0, "Copying firmware");
  Serial.println("Copying UF2 from Flash to USBHFS");

  uint32_t ms = millis();
  size_t copied_bytes = Brain.rp2_programUF2(bindata, bindata_len);
  print_speed(copied_bytes, millis() - ms);

  // wait for rp2040 boot rom to reset
  // while (is_usbfs_mounted) delay(10);
}

void loop() {
  Serial.flush();
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
  Brain.rp2_targetResetBootRom();
}

// core1's loop: process usb host task on core1
void loop1() {
  Brain.USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
// Note: running in the same core where Brain.USBHost.task() is called
//--------------------------------------------------------------------+
bool is_rp2_bootloader(uint16_t vid, uint16_t pid) {
  return (vid == BOOT_VID && (pid == BOOT_PID_RP2040 || pid == BOOT_PID_RP2350));
}

extern "C"  {

// Invoked when device is mounted (configured)
void tuh_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if (!is_rp2_bootloader(vid, pid)) {
    Brain.LCD_printf(1, "UnkDev %04x:%04x", vid, pid);
  }
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr) {
  (void)dev_addr;
  Brain.LCD_printf(1, "No USB Device");
}

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if (is_rp2_bootloader(vid, pid)) {
    is_usbfs_mounted = Brain.usbh_mountFS(dev_addr);
    if (is_usbfs_mounted) {
      uint16_t rp2variant = (pid == BOOT_PID_RP2040 ? 2040 : 2350);
      Brain.LCD_printf(1, "RP%u Bootldr", rp2variant);
    }
  }
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(uint8_t dev_addr) {
  is_usbfs_mounted = false;
  Brain.usbh_umountFS(dev_addr);
}

}
