#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "CH9329.h"

#define MAX_SCREEN_WIDTH 1366
#define MAX_SCREEN_HEIGHT 768

#define MAX_DEVICE_COUNT 3

enum KVM_MODE
{
    KVM_MODE_NONE = 0,
    KVM_MODE_LEFT_ONLY,
    KVM_MODE_RIGHT_ONLY,
    KVM_MODE_ON_LEFT,
    KVM_MODE_ON_RIGHT
};

enum SCREEN
{
    SCREEN_LEFT = 0,
    SCREEN_RIGHT
};

struct current_status
{
    SCREEN active_screen = SCREEN_LEFT;
    KVM_MODE mode = KVM_MODE_NONE;
    uint16_t max_height = MAX_SCREEN_HEIGHT;
    uint16_t max_width = MAX_SCREEN_WIDTH;
    int16_t current_mouse_x_in_dual_screen = 0;
    int16_t current_mouse_x = 0;
    int16_t current_mouse_y = 0;
    uint8_t keyboardCount = 0;
    bool prevLeftUSBConnected = false;
    bool prevRightUSBConnected = false;
    struct keyboardlist
    {
        uint8_t dev_addr;
        uint8_t instance;
    } keyboardlist[MAX_DEVICE_COUNT];
} current_status;

/**
 * set kvm board leds
 */
__attribute__((weak)) void setBoardLEDs_cb(KVM_MODE mode);

/**
 * when mouse cursor run from left screen to right screen
 */
__attribute__((weak)) void mouseFromLeftToRight_cb();
/**
 * when mouse cursor run from right screen to left screen
 */
__attribute__((weak)) void mouseFromRightToLeft_cb();

static void checkUSBConnected(bool leftUSBConnected, bool rightUSBConnected)
{
    // left connected.
    if (!current_status.prevLeftUSBConnected && leftUSBConnected)
    {
        // and mouse is on right
        if (current_status.active_screen == SCREEN_RIGHT)
        {
            // should keep on right screen
            current_status.current_mouse_x_in_dual_screen = current_status.current_mouse_x_in_dual_screen + MAX_SCREEN_WIDTH;
            current_status.current_mouse_x = current_status.current_mouse_x + MAX_SCREEN_WIDTH;
        }
    }
    // right disconnected.
    if (current_status.prevRightUSBConnected && !rightUSBConnected)
    {
        // and mouse is on right
        if (current_status.active_screen == SCREEN_RIGHT)
        {
            // should keep on left screen and cursor is on the right edge
            current_status.current_mouse_x_in_dual_screen = MAX_SCREEN_WIDTH;
            current_status.current_mouse_x = MAX_SCREEN_WIDTH;
        }
    }

    current_status.prevLeftUSBConnected = leftUSBConnected;
    current_status.prevRightUSBConnected = rightUSBConnected;
}

static void judge_kvm_mode(bool leftUSBConnected,
                           bool rightUSBConnected,
                           uint8_t mouseRelX,
                           uint8_t mouseRelY)
{
    // DBG_printf("leftUSBConnected = %02X, rightUSBConnected= %02X\r\n",
    //            leftUSBConnected, rightUSBConnected);
    checkUSBConnected(leftUSBConnected, rightUSBConnected);
    if (!leftUSBConnected && !rightUSBConnected)
    {
        current_status.mode = KVM_MODE_NONE;
    }
    else if (!leftUSBConnected && rightUSBConnected)
    {
        current_status.active_screen = SCREEN_RIGHT;
        current_status.mode = KVM_MODE_RIGHT_ONLY;
    }
    else if (leftUSBConnected && !rightUSBConnected)
    {
        current_status.active_screen = SCREEN_LEFT;
        current_status.mode = KVM_MODE_LEFT_ONLY;
    }
    else if (leftUSBConnected && rightUSBConnected)
    {
        DBG_printf("leftUSBConnected = %02X, rightUSBConnected= %02X mouseRelX= %02X mouseRelY= %02X\r\n",
                   leftUSBConnected, rightUSBConnected, mouseRelX, mouseRelY);
        if (mouseRelX >= 0x80)
        {
            current_status.current_mouse_x_in_dual_screen -= (uint8_t)(0 - mouseRelX);
        }
        else
        {
            current_status.current_mouse_x_in_dual_screen += mouseRelX;
        }
        if (mouseRelY >= 0x80)
        {
            current_status.current_mouse_y -= (uint8_t)(0 - mouseRelY);
        }
        else
        {
            current_status.current_mouse_y += mouseRelY;
        }

        if (current_status.current_mouse_x_in_dual_screen <= 0)
        {
            current_status.current_mouse_x_in_dual_screen = 0;
        }
        if (current_status.current_mouse_y <= 0)
        {
            current_status.current_mouse_y = 0;
        }
        if (current_status.current_mouse_y >= MAX_SCREEN_HEIGHT)
        {
            current_status.current_mouse_y = MAX_SCREEN_HEIGHT;
        }
        if (current_status.current_mouse_x_in_dual_screen > MAX_SCREEN_WIDTH * 2)
        {
            current_status.current_mouse_x_in_dual_screen = MAX_SCREEN_WIDTH * 2;
        }

        // cursor is on right screen
        if (current_status.current_mouse_x_in_dual_screen > MAX_SCREEN_WIDTH)
        {
            current_status.current_mouse_x = current_status.current_mouse_x_in_dual_screen - MAX_SCREEN_WIDTH;
            // judge cross screens:
            if (current_status.active_screen == SCREEN_LEFT)
            {
                current_status.active_screen = SCREEN_RIGHT;
                current_status.mode = KVM_MODE_ON_RIGHT;
                mouseFromLeftToRight_cb();
            }
        }
        else
        {
            current_status.current_mouse_x = current_status.current_mouse_x_in_dual_screen;

            if (current_status.active_screen == SCREEN_RIGHT)
            {
                current_status.active_screen = SCREEN_LEFT;
                current_status.mode = KVM_MODE_ON_LEFT;
                mouseFromRightToLeft_cb();
            }
        }
    }

    setBoardLEDs_cb(current_status.mode);
    // DBG_printf("current_mouse_x = %d, current_mouse_y= %d active_screen=%02X, mode=%02X \r\n",
    //            current_status.current_mouse_x, current_status.current_mouse_y, current_status.active_screen, current_status.mode);
}

static uint16_t convert_to_abs_pos_x(uint16_t x)
{
    uint16_t ret = 4096.0f * x / MAX_SCREEN_WIDTH;
    DBG_printf("convert_to_abs_pos_x(%d)=%d\r\n", x, ret);
    return ret;
}
static uint16_t convert_to_abs_pos_y(uint16_t y)
{
    uint16_t ret = 4096.0f * y / MAX_SCREEN_HEIGHT;
    DBG_printf("convert_to_abs_pos_y(%d)=%d\r\n", y, ret);
    return ret;
}

static void addKeyboard(uint8_t dev_addr, uint8_t instance)
{
    DBG_printf("addKeyboard dev_addr= %d, instance=%02x\r\n", dev_addr, instance);
    for (size_t i = 0; i < MAX_DEVICE_COUNT; i++)
    {
        if (current_status.keyboardlist[i].dev_addr == 0)
        {
            current_status.keyboardlist[i].dev_addr = dev_addr;
            current_status.keyboardlist[i].instance = instance;
            current_status.keyboardCount++;
            current_status.keyboardCount = min(current_status.keyboardCount, MAX_DEVICE_COUNT);
            return;
        }
    }
}

static void removeKeyboard(uint8_t dev_addr, uint8_t instance)
{
    DBG_printf("removeKeyboard dev_addr= %d, instance=%02x\r\n", dev_addr, instance);
    for (size_t i = 0; i < MAX_DEVICE_COUNT; i++)
    {
        if (current_status.keyboardlist[i].dev_addr == dev_addr && current_status.keyboardlist[i].instance == instance)
        {
            current_status.keyboardlist[i].dev_addr = 0;
            current_status.keyboardlist[i].instance = 0;
            current_status.keyboardlist[i].dev_addr = 0;
            current_status.keyboardlist[i].instance = 0;
            current_status.keyboardCount--;
            current_status.keyboardCount = max(current_status.keyboardCount, 0);
            return;
        }
    }
}