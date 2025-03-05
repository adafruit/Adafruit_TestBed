// Testing USB host hardware on Brain by printing out VID/PID of attached device
// to LCD. Also determine if device support MSC or HID

// required for Host MSC block device
#include "SdFat_Adafruit_Fork.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

typedef struct {
  uint16_t vid;
  uint16_t pid;
  bool mounted;
} dev_info_t;

// CFG_TUH_DEVICE_MAX is defined by tusb_config header
dev_info_t dev_info[CFG_TUH_DEVICE_MAX] = { 0 };


//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup() {
}

void loop() {
  // Scan and print VID/PID
  uint8_t dev_count = 0;
  for(uint8_t daddr=1; daddr < CFG_TUH_DEVICE_MAX+1; daddr++) {
    dev_info_t const* dev = &dev_info[daddr-1];

    if ( dev->mounted ) {
      dev_count++;
      Brain.LCD_printf("[%u] %04X:%04X", daddr, dev->vid, dev->pid);
    }

    delay(1000);
  }

  if ( dev_count == 0 ) {
    Brain.LCD_printf(0, "No USB attached");
    Brain.LCD_printf(1, "Plug your device");
  }else {
    Brain.LCD_printf("Devices num: %u", dev_count);
    delay(1000);
  }
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

  // Init Brain peripherals
  Brain.begin();

  // Init Brain USB Host
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
  dev_info_t* dev = &dev_info[daddr-1];
  dev->mounted = true;

  tuh_vid_pid_get(daddr, &dev->vid, &dev->pid);

  Serial.printf("Device attached, address = %d\r\n", daddr);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  dev_info_t* dev = &dev_info[daddr-1];
  dev->mounted = false;

  Serial.printf("Device removed, address = %d\r\n", daddr);
}

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
}

}
