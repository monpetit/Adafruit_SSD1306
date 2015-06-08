/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

#include <avr/pgmspace.h>
#ifndef __SAM3X8E__
// #include <util/delay.h>
#endif
#include <stdlib.h>

#include <Wire.h>

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"


#if defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__) || defined(__TM4C1294NCPDT__)

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#define cs_output()		{ \
	MAP_GPIOPadConfigSet((uint32_t)csport, cspinmask, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)csport, cspinmask, GPIO_DIR_MODE_OUT); \
}
#define dc_output()		{ \
	MAP_GPIOPadConfigSet((uint32_t)dcport, dcpinmask, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)dcport, dcpinmask, GPIO_DIR_MODE_OUT); \
}
#define clk_output()	{ \
	MAP_GPIOPadConfigSet((uint32_t)clkport, clkpinmask, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)clkport, clkpinmask, GPIO_DIR_MODE_OUT); \
}
#define mosi_output()	{ \
	MAP_GPIOPadConfigSet((uint32_t)mosiport, mosipinmask, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)mosiport, mosipinmask, GPIO_DIR_MODE_OUT); \
}

#define cs_high()		MAP_GPIOPinWrite((uint32_t)csport, cspinmask, cspinmask)
#define cs_low()		MAP_GPIOPinWrite((uint32_t)csport, cspinmask, 0)
#define dc_high()		MAP_GPIOPinWrite((uint32_t)dcport, dcpinmask, dcpinmask)
#define dc_low()		MAP_GPIOPinWrite((uint32_t)dcport, dcpinmask, 0)
#define clk_high()		MAP_GPIOPinWrite((uint32_t)clkport, clkpinmask, clkpinmask)
#define clk_low()		MAP_GPIOPinWrite((uint32_t)clkport, clkpinmask, 0)
#define mosi_high()		MAP_GPIOPinWrite((uint32_t)mosiport, mosipinmask, mosipinmask)
#define mosi_low()		MAP_GPIOPinWrite((uint32_t)mosiport, mosipinmask, 0)

#elif defined(TARGET_IS_MSP432P4XX)

#define HWREG8(x)                                          (*((volatile uint8_t *)(x)))

#define P1OUT                                              (HWREG8(0x40004C02))  /* Port 1 Output */
#define P2OUT                                              (HWREG8(0x40004C03))  /* Port 2 Output */
#define P3OUT                                              (HWREG8(0x40004C22))  /* Port 3 Output */
#define P4OUT                                              (HWREG8(0x40004C23))  /* Port 4 Output */
#define P5OUT                                              (HWREG8(0x40004C42))  /* Port 5 Output */
#define P6OUT                                              (HWREG8(0x40004C43))  /* Port 6 Output */

#define BIT0                                               (0x0001u)
#define BIT1                                               (0x0002u)
#define BIT2                                               (0x0004u)
#define BIT3                                               (0x0008u)
#define BIT4                                               (0x0010u)
#define BIT5                                               (0x0020u)
#define BIT6                                               (0x0040u)
#define BIT7                                               (0x0080u)
#define BIT8                                               (0x0100u)
#define BIT9                                               (0x0200u)
#define BITA                                               (0x0400u)
#define BITB                                               (0x0800u)
#define BITC                                               (0x1000u)
#define BITD                                               (0x2000u)
#define BITE                                               (0x4000u)
#define BITF                                               (0x8000u)
#define BIT(x)                                             (1 << (x))


volatile uint8_t* output_registers[] = {
    0,            // 0
    0,            // 1 3.3v
    &P6OUT,        // 2 p6.0
    &P3OUT,        // 3 p3.2
    &P3OUT,        // 4 p3.3
    &P4OUT,        // 5 p4.1
    &P4OUT,        // 6 p4.3
    &P1OUT,        // 7 p1.5
    &P4OUT,        // 8 p4.6
    &P6OUT,        // 9 p6.5
    &P6OUT,        // 10 p6.4
    
    &P3OUT,        // 11 p3.6
    &P5OUT,        // 12 p5.2
    &P5OUT,        // 13 p5.0
    &P1OUT,        // 14 p1.7
    &P1OUT,        // 15 p1.6
    0,            // 16 RST
    &P5OUT,        // 17 p5.7
    &P3OUT,        // 18 p3.0
    &P2OUT,        // 19 p2.5
    0,            // 20 GND
    
    0,            // 21 5v
    0,            // 22 GND
    &P6OUT,        // 23 p6.1
    &P4OUT,        // 24 p4.0
    &P4OUT,        // 25 p4.2
    &P4OUT,        // 26 p4.4
    &P4OUT,        // 27 p4.5
    &P4OUT,        // 28 p4.7
    &P5OUT,        // 29 p5.4
    &P5OUT,        // 30 p5.5
    
    &P3OUT,        // 31 P3.7
    &P3OUT,        // 32 P3.5
    &P5OUT,        // 33 P5.1
    &P2OUT,        // 34 P2.3
    &P6OUT,        // 35 P6.7
    &P6OUT,        // 36 P6.6
    &P5OUT,        // 37 P5.6
    &P2OUT,        // 38 P2.4
    &P2OUT,        // 39 P2.6
    &P2OUT,        // 40 P2.7
};


const uint8_t pin_bitmasks[] = {
    0,            // 0
    0,            // 1 3.3v
    BIT0,        // 2 p6.0
    BIT2,        // 3 p3.2
    BIT3,        // 4 p3.3
    BIT1,        // 5 p4.1
    BIT3,        // 6 p4.3
    BIT5,        // 7 p1.5
    BIT6,        // 8 p4.6
    BIT5,        // 9 p6.5
    BIT4,        // 10 p6.4
    
    BIT6,        // 11 p3.6
    BIT2,        // 12 p5.2
    BIT0,        // 13 p5.0
    BIT7,        // 14 p1.7
    BIT6,        // 15 p1.6
    0,        // 16 RST
    BIT7,        // 17 p5.7
    BIT0,        // 18 p3.0
    BIT5,        // 19 p2.5
    0,            // 20 GND
    
    0,            // 21 5v
    0,            // 22 GND
    BIT1,        // 23 p6.1
    BIT0,        // 24 p4.0
    BIT2,        // 25 p4.2
    BIT4,        // 26 p4.4
    BIT5,        // 27 p4.5
    BIT7,        // 28 p4.7
    BIT4,        // 29 p5.4
    BIT5,        // 30 p5.5
    
    BIT7,        // 31 P3.7
    BIT5,        // 32 P3.5
    BIT1,        // 33 P5.1
    BIT3,        // 34 P2.3
    BIT7,        // 35 P6.7
    BIT6,        // 36 P6.6
    BIT6,        // 37 P5.6
    BIT4,        // 38 P2.4
    BIT6,        // 39 P2.6
    BIT7,        // 40 P2.7
};

volatile uint8_t* out_register(int pin)
{
	return output_registers[pin];
}

uint8_t pin_bitmask(int pin)
{
	return pin_bitmasks[pin];
}

#define cs_high()		*csport |= cspinmask			// P5OUT |= BIT2		// 12 (P5.2)
#define cs_low()		*csport &= ~cspinmask		// P5OUT &= ~BIT2
#define dc_high()		*dcport |= dcpinmask			// P3OUT |= BIT6		// 11 (P3.6)
#define dc_low()		*dcport &= ~dcpinmask		// P3OUT &= ~BIT6
#define clk_high()		*clkport |= clkpinmask		// P6OUT |= BIT4		// 10 (P6.4)
#define clk_low()		*clkport &= ~clkpinmask		// P6OUT &= ~BIT4
#define mosi_high()		*mosiport |= mosipinmask		// P6OUT |= BIT5		// 9 (P.6.5)
#define mosi_low()		*mosiport &= ~mosipinmask	// P6OUT &= ~BIT5

#elif defined(__MSP430MCU__)

#define cs_high()		*csport |= cspinmask
#define cs_low()		*csport &= ~cspinmask
#define dc_high()		*dcport |= dcpinmask
#define dc_low()		*dcport &= ~dcpinmask
#define clk_high()		*clkport |= clkpinmask
#define clk_low()		*clkport &= ~clkpinmask
#define mosi_high()		*mosiport |= mosipinmask
#define mosi_low()		*mosiport &= ~mosipinmask

#elif defined(__CC3200R1M1RGC__)

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_apps_rcm.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin.h"
#include "driverlib/gpio.h"
#include "driverlib/prcm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#define cs_output()		{ \
    uint16_t pin_num = digitalPinToPinNum(cs); \
	MAP_PinConfigSet(pin_num, PIN_STRENGTH_2MA | PIN_STRENGTH_4MA | PIN_STRENGTH_6MA, PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)csport, cspinmask, GPIO_DIR_MODE_OUT); \
}
#define dc_output()		{ \
    uint16_t pin_num = digitalPinToPinNum(dc); \
	MAP_PinConfigSet(pin_num, PIN_STRENGTH_2MA | PIN_STRENGTH_4MA | PIN_STRENGTH_6MA, PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)dcport, dcpinmask, GPIO_DIR_MODE_OUT); \
}
#define clk_output()	{ \
    uint16_t pin_num = digitalPinToPinNum(sclk); \
	MAP_PinConfigSet(pin_num, PIN_STRENGTH_2MA | PIN_STRENGTH_4MA | PIN_STRENGTH_6MA, PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)clkport, clkpinmask, GPIO_DIR_MODE_OUT); \
}
#define mosi_output()	{ \
    uint16_t pin_num = digitalPinToPinNum(sid); \
	MAP_PinConfigSet(pin_num, PIN_STRENGTH_2MA | PIN_STRENGTH_4MA | PIN_STRENGTH_6MA, PIN_TYPE_STD); \
    MAP_GPIODirModeSet((uint32_t)mosiport, mosipinmask, GPIO_DIR_MODE_OUT); \
}

#define cs_high()		MAP_GPIOPinWrite((uint32_t)csport, cspinmask, cspinmask)
#define cs_low()		MAP_GPIOPinWrite((uint32_t)csport, cspinmask, 0)
#define dc_high()		MAP_GPIOPinWrite((uint32_t)dcport, dcpinmask, dcpinmask)
#define dc_low()		MAP_GPIOPinWrite((uint32_t)dcport, dcpinmask, 0)
#define clk_high()		MAP_GPIOPinWrite((uint32_t)clkport, clkpinmask, clkpinmask)
#define clk_low()		MAP_GPIOPinWrite((uint32_t)clkport, clkpinmask, 0)
#define mosi_high()		MAP_GPIOPinWrite((uint32_t)mosiport, mosipinmask, mosipinmask)
#define mosi_low()		MAP_GPIOPinWrite((uint32_t)mosiport, mosipinmask, 0)

#else
#define cs_high()		digitalWrite(cs, HIGH) // MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
#define cs_low()		digitalWrite(cs, LOW) // MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
#define dc_high()		digitalWrite(dc, HIGH) // MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
#define dc_low()		digitalWrite(dc, LOW) // MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
#define clk_high()		digitalWrite(sclk, HIGH)
#define clk_low()		digitalWrite(sclk, LOW)
#define mosi_high()		digitalWrite(sid, HIGH)
#define mosi_low()		digitalWrite(sid, LOW)
#endif


// the memory buffer for the LCD

static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
	0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xE0,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
	0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0,
	0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0xF8, 0xFC,
	0xFC, 0xFC, 0xF8, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xE0, 0xE0, 0xC0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xBC, 0x78, 0xF8, 0xF0, 0xE0, 0xF8, 0xF4, 0xFE,
	0xCE, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1F, 0x3F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F,
	0x3F, 0x1F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x1F, 0x1F,
	0x1F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
	0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x3E, 0x3E, 0x3E,
	0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x07, 0x03, 0x07, 0x03, 0x1F, 0x3F, 0x57, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0,
	0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x03, 0x01, 0x00, 0x00,
	0x00, 0x80, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x06, 0xFE, 0xFE, 0x00, 0x00, 0x06, 0xFE, 0xFC, 0x0C, 0x04, 0x06, 0x06,
	0x06, 0xFC, 0xF8, 0x00, 0x06, 0x0E, 0x3C, 0xE0, 0x00, 0x00, 0xE0, 0x78, 0x1C, 0x06, 0x20, 0xF8,
	0x3C, 0x26, 0x22, 0x22, 0x26, 0x3C, 0x38, 0x00, 0x06, 0xFE, 0xFC, 0x0C, 0x04, 0x04, 0x06, 0x06,
	0xFC, 0xF8, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06, 0x06, 0x00, 0x06, 0x7E, 0xFE, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x18, 0x3C, 0x36, 0x62, 0x62, 0xCE, 0xCC, 0x00, 0x00, 0x00,
#if (SSD1306_LCDHEIGHT == 64)
	0x00, 0x00, 0x00, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x03, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xC1, 0xC1, 0xC1, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x03, 0x03, 0x02, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x03, 0x03, 0x02, 0x02, 0x02, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x03, 0x02, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x03, 0x02, 0x02,
	0x02, 0x01, 0x03, 0x03, 0x02, 0x00, 0x01, 0x03, 0x02, 0x02, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0x30, 0x30, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0xE0,
	0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xE2, 0xF3, 0x99, 0x19, 0x89, 0xDB,
	0xFF, 0xFC, 0x00, 0x00, 0x3C, 0xFE, 0xC3, 0x81, 0x01, 0x01, 0x81, 0xC3, 0x80, 0x01, 0x01, 0xFF,
	0xFF, 0x81, 0x81, 0x00, 0x3C, 0x7E, 0xC3, 0x81, 0x01, 0x01, 0x81, 0xC3, 0x7E, 0x3C, 0x00, 0x01,
	0xFF, 0xFF, 0x06, 0x03, 0x01, 0x00, 0x01, 0x07, 0x1E, 0x78, 0xC0, 0xC0, 0xF8, 0x1F, 0x07, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x38, 0x30, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x0C, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
};


// the most basic function, set a single pixel
void Adafruit_SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
        return;

    // check rotation, move pixel around if necessary
    switch (getRotation()) {
    case 1:
        swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    // x is which column
    switch (color) {
    case WHITE:
        buffer[x+ (y/8)*SSD1306_LCDWIDTH] |= (1 << (y&7));
        break;
    case BLACK:
        buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << (y&7));
        break;
    case INVERSE:
        buffer[x+ (y/8)*SSD1306_LCDWIDTH] ^= (1 << (y&7));
        break;
    }

}

Adafruit_SSD1306::Adafruit_SSD1306(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS) : Adafruit_GFX(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT)
{
    cs = CS;
    rst = RST;
    dc = DC;
    sclk = SCLK;
    sid = SID;
    hwSPI = false;
}

// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset
Adafruit_SSD1306::Adafruit_SSD1306(int8_t DC, int8_t RST, int8_t CS) : Adafruit_GFX(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT)
{
    dc = DC;
    rst = RST;
    cs = CS;
    hwSPI = true;
}

// initializer for I2C - we only indicate the reset pin!
Adafruit_SSD1306::Adafruit_SSD1306(int8_t reset) :
    Adafruit_GFX(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT)
{
    sclk = dc = cs = sid = -1;
    rst = reset;
}


void Adafruit_SSD1306::begin(uint8_t vccstate, uint8_t i2caddr, bool reset)
{
    _vccstate = vccstate;
    _i2caddr = i2caddr;

    // set pin directions
    if (sid != -1) {
        pinMode(dc, OUTPUT);
        pinMode(cs, OUTPUT);
#if defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__) || defined(__TM4C1294NCPDT__) ||  defined(__CC3200R1M1RGC__)
        csport      = portBASERegister(digitalPinToPort(cs));
        cspinmask   = digitalPinToBitMask(cs);
        dcport      = portBASERegister(digitalPinToPort(dc));
        dcpinmask   = digitalPinToBitMask(dc);
		cs_output();
		dc_output();
#elif defined(TARGET_IS_MSP432P4XX)
        csport      = out_register(cs);
        cspinmask   = pin_bitmask(cs);
        dcport      = out_register(dc);
        dcpinmask   = pin_bitmask(dc);
#elif defined(__MSP430MCU__)
        csport      = portOutputRegister(digitalPinToPort(cs));
        cspinmask   = digitalPinToBitMask(cs);
        dcport      = portOutputRegister(digitalPinToPort(dc));
        dcpinmask   = digitalPinToBitMask(dc);
#endif
        // csport      = portOutputRegister(digitalPinToPort(cs));
        // cspinmask   = digitalPinToBitMask(cs);
        // dcport      = portOutputRegister(digitalPinToPort(dc));
        // dcpinmask   = digitalPinToBitMask(dc);
        if (!hwSPI) {
            // set pins for software-SPI
            pinMode(sid, OUTPUT);
            pinMode(sclk, OUTPUT);
#if defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__) || defined(__TM4C1294NCPDT__) ||  defined(__CC3200R1M1RGC__)
            clkport     = portBASERegister(digitalPinToPort(sclk));
            clkpinmask  = digitalPinToBitMask(sclk);
            mosiport    = portBASERegister(digitalPinToPort(sid));
            mosipinmask = digitalPinToBitMask(sid);
			clk_output();
			mosi_output();
#elif defined(TARGET_IS_MSP432P4XX)
            clkport     = out_register(sclk);
            clkpinmask  = pin_bitmask(sclk);
            mosiport    = out_register(sid);
            mosipinmask = pin_bitmask(sid);
#elif defined(__MSP430MCU__)
            clkport     = portOutputRegister(digitalPinToPort(sclk));
            clkpinmask  = digitalPinToBitMask(sclk);
            mosiport    = portOutputRegister(digitalPinToPort(sid));
            mosipinmask = digitalPinToBitMask(sid);
#endif
            // clkport     = portOutputRegister(digitalPinToPort(sclk));
            // clkpinmask  = digitalPinToBitMask(sclk);
            // mosiport    = portOutputRegister(digitalPinToPort(sid));
            // mosipinmask = digitalPinToBitMask(sid);
        }
        if (hwSPI) {
            SPI.begin();
			// SPI.setDataMode(1);
#ifdef __SAM3X8E__
            SPI.setClockDivider(9);  // 9.3 MHz
#else
            // SPI.setClockDivider(SPI_CLOCK_DIV2);  // 8 MHz
#endif
        }
    }
    else {
        // I2C Init
        Wire.begin();
#ifdef __SAM3X8E__
        // Force 400 KHz I2C, rawr! (Uses pins 20, 21 for SDA, SCL)
        TWI1->TWI_CWGR = 0;
        TWI1->TWI_CWGR = ((VARIANT_MCK / (2 * 400000)) - 4) * 0x101;
#endif
    }

    if (reset) {
        // Setup reset pin direction (used by both SPI and I2C)
        pinMode(rst, OUTPUT);
#if defined(__CC3200R1M1RGC__) 
        uint16_t pin_num = digitalPinToPinNum(rst);
	    MAP_PinConfigSet(pin_num, PIN_STRENGTH_4MA | PIN_STRENGTH_6MA, PIN_TYPE_STD);
        MAP_GPIODirModeSet((uint32_t)portBASERegister(digitalPinToPort(rst)), digitalPinToBitMask(rst), GPIO_DIR_MODE_OUT);
#endif
        digitalWrite(rst, HIGH);
        // VDD (3.3V) goes high at start, lets just chill for a ms
        delay(1);
        // bring reset low
        digitalWrite(rst, LOW);
        // wait 10ms
        delay(10);
        // bring out of reset
        digitalWrite(rst, HIGH);
        // turn on VCC (9V?)
    }

#if defined SSD1306_128_32
    // Init sequence for 128x32 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x1F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x10);
    }
    else {
        ssd1306_command(0x14);
    }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x22);
    }
    else {
        ssd1306_command(0xF1);
    }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

#if defined SSD1306_128_64
    // Init sequence for 128x64 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x3F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x10);
    }
    else {
        ssd1306_command(0x14);
    }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x9F);
    }
    else {
        ssd1306_command(0xCF);
    }
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x22);
    }
    else {
        ssd1306_command(0xF1);
    }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

#if defined SSD1306_96_16
    // Init sequence for 96x16 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x0F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x00);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x10);
    }
    else {
        ssd1306_command(0x14);
    }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x2);	//ada x12
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x10);
    }
    else {
        ssd1306_command(0xAF);
    }
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x22);
    }
    else {
        ssd1306_command(0xF1);
    }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

    ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}


void Adafruit_SSD1306::invertDisplay(uint8_t i)
{
    if (i) {
        ssd1306_command(SSD1306_INVERTDISPLAY);
    }
    else {
        ssd1306_command(SSD1306_NORMALDISPLAY);
    }
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c)
{
    if (sid != -1) {
        // SPI
        cs_high(); // digitalWrite(cs, HIGH);
        // *csport |= cspinmask;
        dc_low(); // digitalWrite(dc, LOW);
        // *dcport &= ~dcpinmask;
        cs_low(); // digitalWrite(cs, LOW);
        // *csport &= ~cspinmask;
        fastSPIwrite(c);
        cs_high(); // digitalWrite(cs, HIGH);
        // *csport |= cspinmask;
    }
    else {
        // I2C
        uint8_t control = 0x00;   // Co = 0, D/C = 0
        Wire.beginTransmission(_i2caddr);
        WIRE_WRITE(control);
        WIRE_WRITE(c);
        Wire.endTransmission();
    }
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrollright(uint8_t start, uint8_t stop)
{
    ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrollleft(uint8_t start, uint8_t stop)
{
    ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrolldiagright(uint8_t start, uint8_t stop)
{
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop)
{
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void Adafruit_SSD1306::stopscroll(void)
{
    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void Adafruit_SSD1306::dim(boolean dim)
{
    uint8_t contrast;

    if (dim) {
        contrast = 0; // Dimmed display
    }
    else {
        if (_vccstate == SSD1306_EXTERNALVCC) {
            contrast = 0x9F;
        }
        else {
            contrast = 0xCF;
        }
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(contrast);
}

void Adafruit_SSD1306::ssd1306_data(uint8_t c)
{
    if (sid != -1) {
        // SPI
        cs_high(); // digitalWrite(cs, HIGH);
        // *csport |= cspinmask;
        dc_high(); // digitalWrite(dc, HIGH);
        // *dcport |= dcpinmask;
        cs_low(); // digitalWrite(cs, LOW);
        // *csport &= ~cspinmask;
        fastSPIwrite(c);
        cs_high(); // digitalWrite(cs, HIGH);
        // *csport |= cspinmask;
    }
    else {
        // I2C
        uint8_t control = 0x40;   // Co = 0, D/C = 1
        Wire.beginTransmission(_i2caddr);
        WIRE_WRITE(control);
        WIRE_WRITE(c);
        Wire.endTransmission();
    }
}

void Adafruit_SSD1306::display(void)
{
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);   // Column start address (0 = reset)
    ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0); // Page start address (0 = reset)
#if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
#endif

    if (sid != -1) {
        // SPI
        cs_high(); // digitalWrite(cs, HIGH);// *csport |= cspinmask;
        dc_high(); // digitalWrite(dc, HIGH); // *dcport |= dcpinmask;
        cs_low(); // digitalWrite(cs, LOW); // *csport &= ~cspinmask;

        for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
            fastSPIwrite(buffer[i]);
            //ssd1306_data(buffer[i]);
        }
        cs_high(); // digitalWrite(cs, HIGH); // *csport |= cspinmask;
    }
    else {
        // save I2C bitrate
#if 0
#ifndef __SAM3X8E__
        uint8_t twbrbackup = TWBR;
        TWBR = 12; // upgrade to 400KHz!
#endif
#endif
        //Serial.println(TWBR, DEC);
        //Serial.println(TWSR & 0x3, DEC);

        // I2C
        for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
            // send a bunch of data in one xmission
            Wire.beginTransmission(_i2caddr);
            WIRE_WRITE(0x40);
            for (uint8_t x=0; x<16; x++) {
                WIRE_WRITE(buffer[i]);
                i++;
            }
            i--;
            Wire.endTransmission();
        }
#if 0
#ifndef __SAM3X8E__
        TWBR = twbrbackup;
#endif
#endif
    }
}

// clear everything
void Adafruit_SSD1306::clearDisplay(void)
{
    memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}


inline void Adafruit_SSD1306::fastSPIwrite(uint8_t d)
{

    if (hwSPI) {
        (void)SPI.transfer(d);
    }
    else {
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            clk_low(); // digitalWrite(sclk, LOW); // *clkport &= ~clkpinmask;
            if (d & bit) mosi_high(); // digitalWrite(sid, HIGH); // *mosiport |=  mosipinmask;
            else        mosi_low(); // digitalWrite(sid, LOW); // *mosiport &= ~mosipinmask;
            clk_high(); // digitalWrite(sclk, HIGH); // *clkport |=  clkpinmask;
        }
    }
    // cs_high(); //*csport |= cspinmask;
}

void Adafruit_SSD1306::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    boolean bSwap = false;
    switch (rotation) {
    case 0:
        // 0 degree rotation, do nothing
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x
        bSwap = true;
        swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        x -= (w-1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
        bSwap = true;
        swap(x, y);
        y = HEIGHT - y - 1;
        y -= (w-1);
        break;
    }

    if (bSwap) {
        drawFastVLineInternal(x, y, w, color);
    }
    else {
        drawFastHLineInternal(x, y, w, color);
    }
}

void Adafruit_SSD1306::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    // Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) {
        return;
    }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ((x + w) > WIDTH) {
        w = (WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) {
        return;
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y&7);

    switch (color) {
    case WHITE:
        while (w--) {
            *pBuf++ |= mask;
        };
        break;
    case BLACK:
        mask = ~mask;
        while (w--) {
            *pBuf++ &= mask;
        };
        break;
    case INVERSE:
        while (w--) {
            *pBuf++ ^= mask;
        };
        break;
    }
}

void Adafruit_SSD1306::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    bool bSwap = false;
    switch (rotation) {
    case 0:
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        bSwap = true;
        swap(x, y);
        x = WIDTH - x - 1;
        x -= (h-1);
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h-1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y
        bSwap = true;
        swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if (bSwap) {
        drawFastHLineInternal(x, y, h, color);
    }
    else {
        drawFastVLineInternal(x, y, h, color);
    }
}


void Adafruit_SSD1306::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color)
{

    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) {
        return;
    }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if ((__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;


    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y&7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8-mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if (h < mod) {
            mask &= (0XFF >> (mod-h));
        }

        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }

        // fast exit if we're done here!
        if (h<mod) {
            return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
                *pBuf=~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do  {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }
    }
}
