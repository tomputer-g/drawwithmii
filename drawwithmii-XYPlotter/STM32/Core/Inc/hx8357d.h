/*
 * hx8357d.h
 *
 *  Created on: Mar 15, 2022
 *      Author: gzm20
 */

#ifndef INC_HX8357D_H_
#define INC_HX8357D_H_

/* Acknowledgement:
 * This code is created for the project by referencing other existing libraries for the HX8357D chip.
 * These include:
 * https://github.com/JCardoen/stm32_hx8357D_lib
 * https://github.com/adafruit/Adafruit_CircuitPython_HX8357
 * https://github.com/adafruit/Adafruit_HX8357_Library
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "string.h"
//----------------------------reg addr etc definitions-----------------------
//LCD dimensions defines
#define HX8357_TFTWIDTH            320  ///< 320 pixels wide
#define HX8357_TFTHEIGHT           480  ///< 480 pixels tall

#define LCD_PIXEL_COUNT	HX8357_TFTWIDTH*HX8357_TFTHEIGHT

#define HX8357_NOP                0x00  ///< No op
#define HX8357_SWRESET            0x01  ///< software reset
#define HX8357_RDDID              0x04  ///< Read ID
#define HX8357_RDDST              0x09  ///< (unknown)

#define HX8357_RDPOWMODE          0x0A  ///< Read power mode Read power mode
#define HX8357_RDMADCTL           0x0B  ///< Read MADCTL
#define HX8357_RDCOLMOD           0x0C  ///< Column entry mode
#define HX8357_RDDIM              0x0D  ///< Read display image mode
#define HX8357_RDDSDR             0x0F  ///< Read dosplay signal mode

#define HX8357_SLPIN              0x10  ///< Enter sleep mode
#define HX8357_SLPOUT             0x11  ///< Exit sleep mode

#define HX8357_INVOFF             0x20  ///< Turn off invert
#define HX8357_INVON              0x21  ///< Turn on invert
#define HX8357_DISPOFF            0x28  ///< Display on
#define HX8357_DISPON             0x29  ///< Display off

#define HX8357_CASET              0x2A  ///< Column addr set
#define HX8357_PASET              0x2B  ///< Page addr set
#define HX8357_RAMWR              0x2C  ///< Write VRAM
#define HX8357_RAMRD              0x2E  ///< Read VRAm

#define HX8357_TEON               0x35  ///< Tear enable on
#define HX8357_TEARLINE           0x44  ///< (unknown)
#define HX8357_MADCTL             0x36  ///< Memory access control
#define HX8357_COLMOD             0x3A  ///< Color mode

#define HX8357_SETOSC             0xB0  ///< Set oscillator
#define HX8357_SETPWR1            0xB1  ///< Set power control
#define HX8357_SETRGB             0xB3  ///< Set RGB interface
#define HX8357D_SETCOM            0xB6  ///< Set VCOM voltage

#define HX8357D_SETCYC            0xB4  ///< Set display cycle reg
#define HX8357D_SETC              0xB9  ///< Enable extension command

#define HX8357D_SETSTBA           0xC0  ///< Set source option
#define HX8357_SETPANEL           0xCC  ///< Set Panel

#define HX8357D_SETGAMMA          0xE0  ///< Set Gamma

#define MADCTL_MY  0x80 ///< Bottom to top
#define MADCTL_MX  0x40 ///< Right to left
#define MADCTL_MV  0x20 ///< Reverse Mode
#define MADCTL_ML  0x10 ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04 ///< LCD refresh right to left

//Color Definitions
#define	HX8357_BLACK   0x0000 ///< BLACK color for drawing graphics
#define	HX8357_BLUE    0x001F ///< BLUE color for drawing graphics
#define	HX8357_RED     0xF800 ///< RED color for drawing graphics
#define	HX8357_GREEN   0x07E0 ///< GREEN color for drawing graphics
#define HX8357_CYAN    0x07FF ///< CYAN color for drawing graphics
#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
#define HX8357_YELLOW  0xFFE0 ///< YELLOW color for drawing graphics
#define HX8357_WHITE   0xFFFF ///< WHITE color for drawing graphics

//Function prototypes - basic level
void LCD_sendCommand(uint8_t command);
void LCD_sendData(uint8_t data);
void LCD_sendDataMulti(uint16_t colorData, uint32_t size);
void LCD_sendCommand_NoCS(uint8_t com);
void LCD_sendData_NoCS(uint8_t data);

void LCD_init(SPI_HandleTypeDef *spiLcdHandle, GPIO_TypeDef *csPORT, uint16_t csPIN, GPIO_TypeDef *dcPORT, uint16_t dcPIN, GPIO_TypeDef *resetPORT, uint16_t resetPIN);
//Graphics function prototypes
void LCD_setCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_drawPixel(uint16_t x, uint16_t y, uint16_t color);
//TODO more functions needed?
void LCD_fill(uint16_t color);
void LCD_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
#endif /* INC_HX8357D_H_ */
