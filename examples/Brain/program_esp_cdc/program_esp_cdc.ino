// This sketch program ESP32 by flashing bin file via Serial Host.
// Hardware wiring is a bit different between S2/S3 and ESP32 + USB-to-UART chip
//   For S2/S3 with native USB
//   - Brain GPIO28   <-> ESP32 IO0
//   - Brain Reset    <-> ESP32 Enable
//   - Brain USB Host <-> ESP32 native usb
//   For ESP32 with USB-to-UART chip
//   - Brain USB Host <-> ESP32 native usb
//   - There is no need to connect IO0/Reset since we will use DTR/RTS to reset ESP32

// required for Host MSC block device
#include "SdFat_Adafruit_Fork.h"

// required for USB host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#include "Adafruit_TestBed_Brains.h"

// Change BIN_FILES in esp_binaries.h header to select which binaries to flash
// bin_files[] is defined accordingly
#include "esp_binaries.h"

// 1 if programming ESP32-S2/S3 via native USB
// 0 if programming ESP32/8266 via USB-to-UART chip such as FTDI/CP210x/CH9102f
#define ESP32_NATIVE_USB 1

#define ESP32_RESET     27
#define ESP32_IO0       28

// Note: baudrate does not matter if programming S2/S3 native USB
// But does matter if programming ESP32/8266 via USB-to-UART chip
#define ESP32_BAUDRATE  (115200*8)

// CDC Host object
Adafruit_USBH_CDC SerialHost;

#if ESP32_NATIVE_USB
  // Declare BootROM with IO0 and Reset will use GPIO for bootloader reset
  // This is typically for programming ESP32-S2/S3 via native USB
  ESP32BootROMClass ESP32BootROM(SerialHost, ESP32_IO0, ESP32_RESET);
#else
  // Defined an boot rom object that use SerialHost
  // Declare BootROM without IO0 and Reset will use SerialHost.setDtrRts() for bootloader reset
  // This is for programming ESP32/8266 via USB-to-UART chip such as FTDI/CP210x/CH9102f
  ESP32BootROMClass ESP32BootROM(SerialHost);
#endif

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

// Reset using DTR/RTS
void reset_with_dtr_rts(uint32_t ms) {
  SerialHost.setDtrRts(false, true);
  delay(ms);
  SerialHost.setDtrRts(false, false);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Tester Brains: Programming ESP32 with SerialHost!");

  // sync: wait for Brain.begin() called in core1 before accessing SD or other peripherals
  while (!Brain.inited()) delay(10);

  while (!Brain.esp32_begin(&ESP32BootROM, ESP32_BAUDRATE)) {
    delay(100);
  }

  // Writing bin files
  size_t total_bytes = 0;
  uint32_t ms = millis();
  for (size_t i = 0; i < BIN_FILES_COUNT; i++) {
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
  Brain.targetReset(); // reset using Reset pin GPIO27

  // Reset using DTR if GPIO27 is not connected
  reset_with_dtr_rts(20);
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
  SerialHost.begin(115200);

  Brain.LCD_printf(0, "No USB attached");
  Brain.LCD_printf(1, "Plug your device");
}

// core1's loop: process usb host task on core1
void loop1() {
  if (Brain.esp32_s3_inReset()) {
    // Note: S3 has an USB-OTG errata
    // https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
    // which is walkarounded by idf/arduino-esp32 to always mux JTAG to USB for
    // uploading and/or power on. Afterwards USB-OTG will be set up if selected
    // so. However rp2040 USBH is running too fast and can actually retrieve
    // device/configuration descriptor of JTAG before the OTG is fully setup.
    // We delay a bit here
    delay(500);

    Brain.esp32_s3_clearReset();
  }

  Brain.USBHost.task();

  // periodically flush SerialHost if connected
  if (SerialHost && SerialHost.connected()) {
    SerialHost.flush();
  }

  yield();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
// Note: running in the same core where Brain.USBHost.task() is called
//--------------------------------------------------------------------+
extern "C" {

// Invoked when device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(daddr, &vid, &pid);

  Serial.printf("Device attached, address = %d\r\n", daddr);
  Brain.LCD_printf("USBID %04x:%04x", vid, pid);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr) {
  (void) dev_addr;
  Brain.LCD_printf(1, "No USB Device");
}

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {
  // bind SerialHost object to this interface index
  SerialHost.mount(idx);
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  // unbind SerialHost if this interface is unmounted
  SerialHost.umount(idx);
}
}
