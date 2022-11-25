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

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup() {
}

void loop() {
  Serial.flush();
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

// call usbh_begin() hrere to make pio usb background task run on core1
// NOTE: Brain.begin() should be called here as well to prevent race condition
void setup1() {
  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("Hello world, Tester Brains self test!");

  Brain.begin();
  Brain.usbh_begin();

  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");

  // reset rp2040 target into Boot Rom, default bootsel = 28, reset duration = 10 ms
  Brain.rp2040_targetResetBootRom();
}

// core1's loop: process usb host task on core1
void loop1()
{
  Brain.USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C"  {

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t daddr)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(daddr, &vid, &pid);

  Brain.LCD_printf(0, "USBID %04x:%04x", vid, pid);
  if ( vid == BOOT_VID && pid == BOOT_PID ) {
    Brain.LCD_printf(1, "RP2040 Boot ROM");
  }
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");
}

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr)
{
  if ( Brain.usbh_mountFS(dev_addr) ) {
    Brain.USBH_FS.ls(&Serial, LS_SIZE);
  }else
  {
    Serial.println("Failed to mount USB FS");
  }
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(uint8_t dev_addr)
{
  Brain.usbh_umountFS(dev_addr);
}

}
