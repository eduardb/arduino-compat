/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_arduino.h"

#include "mgos_app.h"

#include <Arduino.h>

#include "common/cs_dbg.h"
#include "mongoose/mongoose.h"

#include "mgos_gpio.h"
#include "mgos_hal.h"
#include "mgos_init.h"
#include "mgos_timers.h"

#include "esp8266_peri.h"

uint8_t esp8266_gpioToFn[16] = {0x34, 0x18, 0x38, 0x14, 0x3C, 0x40, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x04, 0x08, 0x0C, 0x10};

#ifndef IRAM
#define IRAM
#endif

IRAM void pinMode(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case INPUT:
    case INPUT_PULLUP:
      mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_INPUT);
      mgos_gpio_set_pull(pin, (mode == INPUT_PULLUP ? MGOS_GPIO_PULL_UP
                                                    : MGOS_GPIO_PULL_NONE));
      break;
    case OUTPUT:
      mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_OUTPUT);
      break;
    case SPECIAL:
      GPC(pin) = (GPC(pin) & (0xF << GPCI)); //SOURCE(GPIO) | DRIVER(NORMAL) | INT_TYPE(UNCHANGED) | WAKEUP_ENABLE(DISABLED)
      GPEC = (1 << pin); //Disable
      GPF(pin) = GPFFS(GPFFS_BUS(pin));//Set mode to BUS (RX0, TX0, TX1, SPI, HSPI or CLK depending in the pin)
      if(pin == 3) GPF(pin) |= (1 << GPFPU);//enable pullup on RX
      break;
    default:
      LOG(LL_ERROR, ("Unsupported mode: %u", mode));
  }
}

IRAM int digitalRead(uint8_t pin) {
  return mgos_gpio_read(pin);
}

IRAM void digitalWrite(uint8_t pin, uint8_t val) {
  mgos_gpio_write(pin, val);
}

void delay(unsigned int ms) {
  mgos_usleep(ms * 1000);
}

void delayMicroseconds(unsigned int us) {
  mgos_usleep(us);
}

unsigned long millis(void) {
  return mgos_uptime() * 1000;
}

unsigned long micros(void) {
  return mgos_uptime() * 1000000;
}

void interrupts(void) {
  mgos_ints_enable();
}

void noInterrupts(void) {
  mgos_ints_disable();
}

extern "C" {
static mgos_timer_id s_loop_timer;
}

void setup(void) __attribute__((weak));
void setup(void) {
}

void loop(void) __attribute__((weak));
void loop(void) {
  // It's a dummy loop, no need to invoke it.
  mgos_clear_timer(s_loop_timer);
}

extern "C" {
void loop_cb(void *arg) {
  loop();
  (void) arg;
}

bool mgos_arduino_compat_init(void) {
  setup();
  s_loop_timer = mgos_set_timer(0, true /* repeat */, loop_cb, NULL);
  return true;
}

}  // extern "C"
