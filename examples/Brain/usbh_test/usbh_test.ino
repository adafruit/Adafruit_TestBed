// Testing USB host hardware on Brain by printing out VID/PID of attached device
// to LCD. Also determine if device support MSC or HID

// required for Host MSC block device
#include "SdFat.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup() {
}

void loop() {

}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

// call usbh_begin() hrere to make pio usb background task run on core1
// NOTE: Brain.begin() should be called here as well to prevent race condition
void setup1() {
  Serial.begin(115200);
  // while (!Serial) delay(10);
  Serial.println("Tester Brains USB Host test!");

  Brain.begin();
  Brain.usbh_begin();

  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");
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

  Serial.printf("Device attached, address = %d\r\n", daddr);
  Brain.LCD_printf(0, "USBID %04x:%04x", vid, pid);
  Brain.LCD_printf(1, "MSC %u HID %u", tuh_msc_mounted(daddr), tuh_hid_instance_count(daddr));
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  Serial.printf("Device removed, address = %d\r\n", daddr);
  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");

}

}
