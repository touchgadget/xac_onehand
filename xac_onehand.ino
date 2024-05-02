/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industriesi
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
MIT License

Copyright (c) 2024 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// TODO Use RGB LED for error conditions such as no DAC and no USB flight stick.

// Set to false to remove the CDC ACM serial port but XAC seems to tolerate it
// so it is on by default. Set to false if it causes problems. Disabling the
// CDC port means you must press button(s) on the RP2040 to put in bootloader
// upload mode before using the IDE to upload.
const bool DEBUG_ON = false;

#define DBG_print(...)    if (DEBUG_ON) Serial.print(__VA_ARGS__)
#define DBG_println(...)  if (DEBUG_ON) Serial.println(__VA_ARGS__)
#define DBG_printf(...)   if (DEBUG_ON) Serial.printf(__VA_ARGS__)

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"
// USB flight stick joystick
#include "flight_stick_tinyusb.h"
// I2C flight stick joystick
#include <Wire.h>

#if defined(ARDUINO_ARCH_RP2040) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) \
    || defined(ARDUINO_SAM_DUE) \
    || defined(ARDUINO_ARCH_RENESAS_UNO)
  #if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST) \
      || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
    #define QWIIC Wire
  #else
    #define QWIIC Wire1
  #endif
#else
  #define QWIIC Wire
#endif

// USB flight stick joystick
Adafruit_USBD_HID G_usb_hid;
FSJoystick FSJoy(&G_usb_hid);

// Thrustmaster T.16000M flight joystick *******************************
const uint16_t T16K_VID = 0x044f;
const uint16_t T16K_PID = 0xb10a;
#define isT16K(vid, pid)  ((vid == T16K_VID) && (pid == T16K_PID))
bool T16K = false;
// Thrustmaster T.16000M Flight sim joystick HID report layout
// Large joystick X, Y, Z (twist) axes
// 8 way hat switch
// 16 buttons (4 on the stick, 12 on the base)
// throttle slider
typedef struct __attribute__ ((packed)) {
  uint16_t	buttons;
  uint8_t		hat;
  uint16_t	x;
  uint16_t	y;
  uint8_t		twist;
  uint8_t		slider;
} T16K_Report_t;

// Logitech Extreme 3D Pro flight joystick *****************************
const uint16_t LE3DP_VID = 0x046d;
const uint16_t LE3DP_PID = 0xc215;
#define isLE3DP(vid, pid)  ((vid == LE3DP_VID) && (pid == LE3DP_PID))
bool LE3DP = false;
// Flight sim joystick HID report layout
// Large joystick X, Y, Z (twist) axes
// 8 way hat switch
// 12 buttons (6 on the stick, 6 on the base)
// throttle slider
typedef struct __attribute__ ((packed)) {
  uint32_t x : 10;      // 0..512..1023
  uint32_t y : 10;      // 0..512..1023
  uint32_t hat : 4;
  uint32_t twist : 8;   // 0..127..255
  uint8_t buttons_a;
  uint8_t slider;       // 0..255
  uint8_t buttons_b;
} LE3DP_Report_t;

// Logitech X52 H.O.T.A.S **********************************************
const uint16_t LX52_VID = 0x06a3;
const uint16_t LX52_PID = 0x075c;
#define isLX52(vid, pid)  ((vid == LX52_VID) && (pid == LX52_PID))
bool LX52 = false;
// Flight sim joystick HID report layout
// Large joystick X, Y, Z (twist) axes
// 3 X 8 way hat switches
// 34 buttons
// throttle slider
typedef struct __attribute__ ((packed)) {
  uint16_t x:11;
  uint16_t y:11;
  uint16_t twist:10;
  uint8_t z;
  uint8_t rx;
  uint8_t ry;
  uint8_t slider;
  uint64_t buttons:34;
  uint8_t filler:2;
  uint8_t dpad:4;
  uint8_t dpad2:4;
  uint8_t dpad3:4;
} LX52_Report_t;

const uint16_t XAC_MAX = 1023;
const uint16_t XAC_MID = 511;
const uint16_t XAC_MIN = 0;

const uint16_t HAT2XAC[16] = {
  XAC_MIN,  // North
  XAC_MIN,  // North East
  XAC_MID,  // East
  XAC_MAX,  // South East
  XAC_MAX,  // South
  XAC_MAX,  // South West
  XAC_MID,  // West
  XAC_MIN,  // North West
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID,  // No direction
  XAC_MID   // No direction
};

void xac_center_all(void) {
  FSJoystick_Report_t FSJoy_right;

  // USB left joystick
  FSJoy.xAxis(XAC_MID);
  FSJoy.yAxis(XAC_MID);
  FSJoy.twist(127);
  if (FSJoy.ready()) FSJoy.loop();

  // I2C right joystick
  memset(&FSJoy_right, 0, sizeof(FSJoy_right));
  FSJoy_right.x = XAC_MID;
  FSJoy_right.y = XAC_MID;
  FSJoy_right.twist = 127;
  QWIIC.beginTransmission(0x30);
  QWIIC.write((uint8_t *)&FSJoy_right, sizeof(FSJoy_right));
  QWIIC.endTransmission();
}


//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
  QWIIC.setClock(400000);
  QWIIC.begin();
  
  if (DEBUG_ON) {
    FSJoy.begin();
    // wait until device mounted
    Serial.begin(115200);
    while (!Serial && (millis() < 4000)) delay(10);   // wait for native usb
    Serial.println("Flight stick/gamepad to two USB joysticks");
  } else {
    Serial.end();   // Remove CDC ACM USB Serial console port
    FSJoy.begin();
  }

#if !defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
  USBHost.begin(1);
#endif  // !defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
  while( !FSJoy.ready() ) delay(1);
  FSJoy.write();
  DBG_println("USB joystick ready");

  xac_center_all();
}

typedef struct {
  uint8_t report[64];
  uint32_t report_count = 0;
  uint32_t available_count = 0;
  uint32_t last_millis = 0;
  uint8_t len;
  bool available = false;
  bool skip_report_id;
  bool hid_joystick;
} HID_state_t;

volatile HID_state_t HID_Report;

void loop()
{
  FSJoystick_Report_t FSJoy_right;
  memset(&FSJoy_right, 0, sizeof(FSJoy_right));

#if !defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
  USBHost.task();
#endif  // !defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
  if (HID_Report.available) {
    if (HID_Report.len > 2) {
      uint16_t xac_x, xac_y, xac_right_x, xac_right_y;
      uint8_t  xac_buttons, xac_right_buttons;
      if (LE3DP && (HID_Report.len == sizeof(LE3DP_Report_t))) {
        LE3DP_Report_t *rpt = (LE3DP_Report_t *)HID_Report.report;
        xac_x = map(rpt->x, 0, 1023, XAC_MIN, XAC_MAX);
        xac_y = map(rpt->y, 0, 1023, XAC_MIN, XAC_MAX);
        xac_buttons = rpt->buttons_a;
        xac_right_x = map(rpt->twist, 0, 255, XAC_MIN, XAC_MAX);
        xac_right_y = HAT2XAC[rpt->hat];
        xac_right_buttons = rpt->buttons_b;
#if 0
        DBG_printf("LE3DP x: %d xac x: %d y: %d xac y: %d "
            "LE3DP twist: %d xac_right_x: %d LE3DP hat: %d xac_right_y: %d\n",
            rpt->x, xac_x, rpt->y, xac_y, rpt->twist, xac_right_x, rpt->hat,
            xac_right_y);
#endif
      } if (LX52 && (HID_Report.len == sizeof(LX52_Report_t))) {
        LX52_Report_t *rpt = (LX52_Report_t *)HID_Report.report;
        xac_x = map(rpt->x, 0, 2047, XAC_MIN, XAC_MAX);
        xac_y = map(rpt->y, 0, 2047, XAC_MIN, XAC_MAX);
        xac_buttons = rpt->buttons & 0xFF;
        xac_right_x = map(rpt->twist, 0, 1023, XAC_MIN, XAC_MAX);
        xac_right_y = (rpt->dpad == 0) ? XAC_MID : HAT2XAC[rpt->dpad - 1];
        xac_right_buttons = (rpt->buttons >> 8) & 0xFF;
#if 0
        DBG_printf("LX52 x: %d xac x: %d y: %d xac y: %d "
            "LX52 twist: %d xac_right_x: %d LX52 hat: %d xac_right_y: %d\n",
            rpt->x, xac_x, rpt->y, xac_y, rpt->twist, xac_right_x, rpt->dpad,
            xac_right_y);
#endif
      } if (T16K && (HID_Report.len == sizeof(T16K_Report_t))) {
        T16K_Report_t *rpt = (T16K_Report_t *)HID_Report.report;
        xac_x = map(rpt->x, 0, 16383, XAC_MIN, XAC_MAX);
        xac_y = map(rpt->y, 0, 16383, XAC_MIN, XAC_MAX);
        xac_buttons = rpt->buttons & 0xFF;
        xac_right_x = map(rpt->twist, 0, 255, XAC_MIN, XAC_MAX);
        xac_right_y = HAT2XAC[rpt->hat];
        xac_right_buttons = (rpt->buttons >> 8) & 0xFF;
#if 0
        DBG_printf("T16K x: %d xac x: %d y: %d xac y: %d "
            "T16K twist: %d xac_right_x: %d T16K hat: %d xac_right_y: %d\n",
            rpt->x, xac_x, rpt->y, xac_y, rpt->twist, xac_right_x, rpt->hat,
            xac_right_y);
#endif
      }
      // USB left joystick
      FSJoy.xAxis(xac_x);
      FSJoy.yAxis(xac_y);
      FSJoy.buttons(xac_buttons);
      if (FSJoy.ready()) FSJoy.loop();

      // I2C right joystick
      FSJoy_right.x = xac_right_x;
      FSJoy_right.y = xac_right_y;
      FSJoy_right.buttons_a = xac_right_buttons;
      QWIIC.beginTransmission(0x30);
      QWIIC.write((uint8_t *)&FSJoy_right, sizeof(FSJoy_right));
      QWIIC.endTransmission();
    }
    HID_Report.available = false;
  }
}

#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST)
//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+
void setup1() {
  if (DEBUG_ON) {
    while (!Serial) delay(10);
  }
  DBG_println("setup1 enter");
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
  DBG_println("setup1 exit");
}

void loop1()
{
  USBHost.task();
}
#endif  // ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST

extern "C" {

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  LE3DP = isLE3DP(vid, pid);
  LX52 = isLX52(vid, pid);
  T16K = isT16K(vid, pid);
  uint8_t const protocol_mode = tuh_hid_get_protocol(dev_addr, instance);
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  DBG_printf("protocol_mode=%d,itf_protocol=%d\r\n",
      protocol_mode, itf_protocol);

  const size_t REPORT_INFO_MAX = 8;
  tuh_hid_report_info_t report_info[REPORT_INFO_MAX];
  uint8_t report_num = tuh_hid_parse_report_descriptor(report_info,
      REPORT_INFO_MAX, desc_report, desc_len);
  DBG_printf("HID descriptor reports:%d\r\n", report_num);
  for (size_t i = 0; i < report_num; i++) {
    DBG_printf("%d,%d,%d\r\n", report_info[i].report_id, report_info[i].usage,
        report_info[i].usage_page);
    HID_Report.skip_report_id = false;
    HID_Report.hid_joystick = false;
    if ((report_info[i].usage_page == 1) && (report_info[i].usage == 0)) {
      HID_Report.hid_joystick = true;
    }
  }

  if (desc_report && desc_len) {
    for (size_t i = 0; i < desc_len; i++) {
      DBG_printf("%x,", desc_report[i]);
      if ((i & 0x0F) == 0x0F) DBG_println();
    }
    DBG_println();
  }

  HID_Report.report_count = 0;
  HID_Report.available_count = 0;
  DBG_printf("Start polling instance %d\n", instance);
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_println("Error: cannot request to receive report");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
    uint8_t const *report, uint16_t len) {
  HID_Report.report_count++;
  if (HID_Report.available) {
    static uint32_t dropped = 0;
    DBG_printf("drops=%lu\r\n", ++dropped);
  } else {
    if (HID_Report.skip_report_id) {
      // Skip first byte which is report ID.
      report++;
      len--;
    }
    memcpy((void *)HID_Report.report, report, min(len, sizeof(HID_Report.report)));
    HID_Report.len = len;
    HID_Report.available = true;
    HID_Report.available_count++;
    HID_Report.last_millis = millis();
  }
  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_println("Error: cannot request to receive report");
  }
}

} /* extern "C" */
