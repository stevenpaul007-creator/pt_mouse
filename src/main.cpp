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
 * Stripped down version of hid_mouse_tremor_filter.ino that removes
 * everything except for a USB mouse pass through. This can be used
 * as a foundation for autoclickers and jigglers.
 */

/*
MIT License

Copyright (c) 2023 touchgadgetdev@gmail.com

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

/* This example demonstrates use of both device and host, where
 * - Device run on native usb controller (controller0)
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library (controller1)
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set to 0 to remove the CDC ACM serial port. Set to 0 if it causes problems.
// Disabling the CDC port means you must press button(s) on the RP2040 to put
// in bootloader upload mode before using the IDE to upload.
// WARNING: The program stops working if USB_DEBUG is set to 0. Leave it on
// until this is solved.

#include <Arduino.h>

#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "kvm.h"
#include "CH9329.h"
CH9329 *CH9329Client;

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"

#define USB_DEBUG 1
#define USB_DEBUG_SENSITIVE 0

struct CH9329CFG ch9329cfgs[CH9329COUNT] = {
    {
        .led_pin = 2,
        .rx_pin = 8,
        .tx_pin = 9,
        .cfg1_pin = 10,
        .mode1_pin = 11,
        .addr = 0x30,
        .baud = 115200,
        .CFG1 = HIGH,
        .MODE1 = HIGH // LOW: 注：Linux/Android/macOS等操作系统下，建议使用该模式。
    },
    {
        .led_pin = 28,
        .rx_pin = 8,
        .tx_pin = 9,
        .cfg1_pin = 27,
        .mode1_pin = 26,
        .addr = 0x01,
        .baud = 115200,
        .CFG1 = HIGH,
        .MODE1 = HIGH // LOW: 注：Linux/Android/macOS等操作系统下，建议使用该模式。
    }};

// 定义通信结构体
typedef struct TU_ATTR_PACKED
{
  uint8_t type;    // 0: mouse, 1: keyboard, 2:usb mount or unmount
  uint8_t data[8]; // HID report data (根据需要调整大小)
} hid_report_t;

// 使用队列进行安全通信
// 鼠标/键盘报告从 Core 1 发送到 Core 0
queue_t report_queue;

// USB Host object
Adafruit_USBH_Host USBHost;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis0 = 0;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long currentMillis = 0;
static unsigned long led_blink_interval = 1000;
uint8_t led_blink_count = 0;

static void process_usb_mount_report(hid_report_t *report)
{
  for (size_t i = 0; i < CH9329COUNT; i++)
  {
    CH9329Client->releaseAll(i);
    CH9329Client->mouseRelease(i);
  }

  led_blink_interval = 200;
}

static void process_kbd_report(hid_keyboard_report_t const *report)
{
  CH9329Client->press(current_status.active_screen,
                      report->modifier,
                      report->keycode[0],
                      report->keycode[1],
                      report->keycode[2],
                      report->keycode[3],
                      report->keycode[4],
                      report->keycode[5]
  );
}

static void process_mouse_report(hid_mouse_report_t const *report)
{
  judge_kvm_mode(CH9329Client->isUSBConnected(0),
                 CH9329Client->isUSBConnected(1),
                 report->x, report->y);
  switch (current_status.mode)
  {
  case KVM_MODE_LEFT_ONLY:
  case KVM_MODE_RIGHT_ONLY:
    CH9329Client->mouseMove(current_status.active_screen, report->x, report->y, report->buttons, report->wheel);
    break;
  case KVM_MODE_ON_LEFT:
  case KVM_MODE_ON_RIGHT:
    CH9329Client->mouseMoveAbs(current_status.active_screen,
                               convert_to_abs_pos_x(current_status.current_mouse_x),
                               convert_to_abs_pos_y(current_status.current_mouse_y),
                               report->buttons,
                               report->wheel);
    // CH9329 click and wheel are not working under linux and android.
    CH9329Client->mouseMove(current_status.active_screen,
                            0,
                            0,
                            report->buttons,
                            report->wheel);
    break;
  default:
    break;
  }
#if USB_DEBUG && 0
  // Mouse position.
  DBG_printf("Mouse: (%d %d %d)", report->x, report->y, report->wheel);

  // Button state.
  uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
  if (button_changed_mask & report->buttons)
  {
    DBG_printf(" %c%c%c",
               report->buttons & MOUSE_BUTTON_LEFT ? 'L' : '-',
               report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-',
               report->buttons & MOUSE_BUTTON_RIGHT ? 'R' : '-');
  }

  DBG_printf("\n");
#endif
}
// tuh_hid_mount_cb is executed when a new device is mounted.
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  if (itf_protocol == HID_ITF_PROTOCOL_NONE)
  {
    DBG_printf("Device with address %d, instance %d is not a keyboard or mouse.\r\n", dev_addr, instance);
    return;
  }
  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD)
  {
    addKeyboard(dev_addr, instance);
  }

  const char *protocol_str[] = {"None", "Keyboard", "Mouse"};
  DBG_printf("Device with address %d, instance %d is a %s.\r\n", dev_addr, instance, protocol_str[itf_protocol]);

  // notify core0 to blink
  hid_report_t new_report;
  new_report.type = 3;
  queue_try_add(&report_queue, &new_report);

  // request to receive report
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// tuh_hid_report_received_cb is executed when data is received from the keyboard or mouse.
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  uint8_t type = tuh_hid_interface_protocol(dev_addr, instance);

#if USB_DEBUG && USB_DEBUG_SENSITIVE
  // 打印接收到的数据长度和类型
  DBG_printf("Received Report: Type=%02X, Length=%u\r\n", type, len);

  // 2. 打印原始报告数据的前几个字节，排除 memcpy 的错误
  DBG_printf("Data: ");
  for (uint16_t i = 0; i < len && i < 16; i++)
  { // 限制最多打印16个字节
    DBG_printf("%02X ", report[i]);
  }
  DBG_printf("\r\n");
#endif

  hid_report_t new_report;
  new_report.type = type;
  memcpy(new_report.data, report, len);

  queue_try_add(&report_queue, &new_report);

  // request to receive next report
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// tuh_hid_umount_cb is executed when a device is unmounted.
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  removeKeyboard(dev_addr, instance);
  // notify core0 to blink
  hid_report_t new_report;
  new_report.type = 3;
  queue_try_add(&report_queue, &new_report);

  DBG_printf("Device with address %d, instance %d was unmounted.\r\n", dev_addr, instance);
}

void kvm_change_keyboard_leds_cb(uint8_t index, uint8_t leds)
{
  static uint8_t new_led_value;
  new_led_value = leds;
  if (current_status.active_screen == index)
  {
    for (size_t i = 0; i < current_status.keyboardCount; i++)
    {
      tuh_hid_set_report(current_status.keyboardlist[i].dev_addr,
                         current_status.keyboardlist[i].instance,
                         0,
                         HID_REPORT_TYPE_OUTPUT,
                         &new_led_value,
                         sizeof(leds));
    }
  }
}

void setBoardLEDs_cb(KVM_MODE mode)
{
  switch (mode)
  {
  case KVM_MODE_NONE:
    CH9329Client->turnOffLed(0);
    CH9329Client->turnOffLed(1);
    break;
  default:
    CH9329Client->turnOnLed(current_status.active_screen);
    CH9329Client->turnOffLed(!current_status.active_screen);
    break;
  }
}

void mouseFromLeftToRight_cb()
{
  CH9329Client->mouseMoveAbs(SCREEN_LEFT,
                             convert_to_abs_pos_x(MAX_SCREEN_WIDTH) - 1,
                             convert_to_abs_pos_y(current_status.current_mouse_y),
                             0, 0);
}

void mouseFromRightToLeft_cb()
{
  CH9329Client->mouseMoveAbs(SCREEN_RIGHT,
                             0,
                             convert_to_abs_pos_y(current_status.current_mouse_y),
                             0, 0);
}

// 新增任务：LED 闪烁任务
void ledTask()
{
  if (led_blink_interval < 1000)
  {
    led_blink_count++;
    if (led_blink_count >= 10)
    {
      led_blink_count = 0;
      led_blink_interval = 1000;
    }
  }
  gpio_xor_mask(1U << LED_BUILTIN); // LED 开关
}
/**
 * 任务函数: 按下BootSel按钮
 */
void BootSelTask()
{
  if (BOOTSEL)
  {
    rp2040.rebootToBootloader();
  }
}
void resetCH9329Task()
{
  for (int i = 0; i < CH9329COUNT; i++)
  {
    CH9329Client->cmdReset(i);
    delay(10);
    DBG_printf("CH9329[%d] inited.\r\n", i);
  }
}
/**
 * 任务函数: 每500ms向 Serial2 发送获取状态
 */
void Serial2SendGetInfoTask()
{
  for (int i = 0; i < CH9329COUNT; i++)
  {
    CH9329Client->cmdGetInfo(i);
    delay(20);
  }
}
/**
 * 任务函数: Serial2读取
 */
void ReadSerialTask()
{
  CH9329Client->readUart();
}
/**
 * 任务函数: Serial2写入
 */
void Core0SendSerialTask()
{
  hid_report_t report;
  if (queue_try_remove(&report_queue, &report))
  {
    switch (report.type)
    {
    case HID_ITF_PROTOCOL_KEYBOARD:
      process_kbd_report((hid_keyboard_report_t const *)report.data);
      break;
    case HID_ITF_PROTOCOL_MOUSE:
      process_mouse_report((hid_mouse_report_t const *)report.data);
      break;
    case 3:
      process_usb_mount_report(&report);
      break;
    }
  }
}
void CH9329ClientStatusTask()
{
  judge_kvm_mode(CH9329Client->isUSBConnected(0),
                 CH9329Client->isUSBConnected(1),
                 0, 0);
}

/**
 * 任务函数: Core1USBInitTask
 */
void setup1()
{
  delay(3000);
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");
  Serial.flush();

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP2;
  pio_cfg.pinout = PIO_USB_PINOUT_DPDM;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // --- 配置第二个 Host 端口 (RHPort 2, GPIO 14/15) ---
  // 配置 USB 主机库的根端口 2
  // pio_usb_configuration_t pio_cfg_2 = PIO_USB_DEFAULT_CONFIG;
  // pio_cfg_2.pin_dp = PIN_PIO_USB_HOST_DP2;
  // USBHost.configure_pio_usb(2, &pio_cfg_2);

  // pio_usb_host_add_port(PIN_PIO_USB_HOST_DP2, PIO_USB_PINOUT_DPDM);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
  DBG_println("USB1 begin.");

  // See https://github.com/sekigon-gonnoc/Pico-PIO-USB/discussions/72#discussioncomment-9456017
  /**
   * there is a function called pio_usb_host_add_port which adds a new host port.
   * This can even be extendet to more ports if the value PIO_USB_ROOT_PORT_CNT (in file pio_usb_configuration.h) is changed (from 2 to 4 for example).
   * pio_usb_host_add_port needs only 2 parameters (PIO_USB2_DP_PIN, PIO_USB_PINOUT_DPDM) and should be called after   tuh_init(BOARD_TUH_RHPORT).
   * By the way, this has got nothing to do with further PIOs and state machines.
   * Its internally done by changing pin settings on the fly (jmp pin, in pin, set pins, sideset pins).
   * But this works only for host ports (for now).
   */
  pio_usb_host_add_port(PIN_PIO_USB_HOST_DP, PIO_USB_PINOUT_DPDM);
  DBG_println("USB2 begin.");
  Serial.flush();
}

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+
void setup()
{
  Serial.begin(115200);
  DBG_println("Serial inited.");
  Serial.flush();

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL)
  {
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while (1)
      delay(1);
  }

  gpio_init(LED_BUILTIN);
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);

  // 初始化队列
  queue_init(&report_queue, sizeof(hid_report_t), 100); // 100项容量
  DBG_println("Queue inited.");
  Serial.flush();

  CH9329Client = new CH9329(&Serial2, ch9329cfgs);
  DBG_println("CH9329Client inited.");
  Serial.flush();

  resetCH9329Task();
  DBG_println("CH9329Client reseted.");
  Serial.flush();

  // wait until device mounted
  // while (!USBDevice.mounted())
  //   delay(1);
}

void loop()
{
  currentMillis = millis();
  Core0SendSerialTask();

  if (currentMillis - previousMillis1 >= 50)
  {
    ReadSerialTask();
    previousMillis1 = currentMillis;
  }
  if (currentMillis - previousMillis0 >= 500)
  {
    Serial2SendGetInfoTask();
    previousMillis0 = currentMillis;
  }
  if (currentMillis - previousMillis2 >= 1000)
  {
    BootSelTask();
    CH9329ClientStatusTask();
    previousMillis2 = currentMillis;
  }
  if (currentMillis - previousMillis3 >= led_blink_interval)
  {
    ledTask();
    previousMillis3 = currentMillis;
  }
}
void loop1()
{
  USBHost.task();
}
