/*
 * LCD Example for Sipeed Longan Nano a GigaBit RISC-V GD32VF103CBT6 based board.
 *
 * Shows Sipeed logo and text output on on-board LCD
 * 
 * Using Sipeed init_uart0() procedure to send printf output to the serial port
 * http://longan.sipeed.com/zh/examples/printf.html (thank you to Google for the translation)
 *
 * Based on the example nanoblink3 by Michel Deslierres <https://sigmdel.ca/michel>
 * 2020-06-27 Claus Kühnel (info@ckuehnel.ch)
 *
 * This example code is in the public domain.
 */ 

#include <Arduino.h>

extern "C" {
#include "lcd/lcd.h"
}
#include <stdio.h>
#include <inttypes.h>

const char *s = "Sipeed Logan Nano";
uint32_t value = 0;

static void longan_oled_init(void)
{
    Lcd_Init();
    LCD_Clear(BLACK);
    BACK_COLOR = BLACK;
}

void setup()
{
  longan_oled_init();
  LCD_ShowLogo();
  delay_1ms(2000);
  LCD_Clear(BLACK);
  LCD_ShowString(0, 0, (const u8 *) s, GBLUE);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop()
{
  char buf[12];
  sprintf(buf, "Count: %ld", ++value);
  LCD_ShowString(0, 32, (u8 const *) buf, GBLUE);

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(900);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  // wait for a second
  delay(100);
}
