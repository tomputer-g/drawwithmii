/*
 * hx8357d.c
 *
 *  Created on: Mar 15, 2022
 *      Author: gzm20
 */

#include "hx8357d.h"
//---------GPIO definitions
static SPI_HandleTypeDef lcdSPIhandle;
//Chip Select pin
static GPIO_TypeDef  *tftCS_GPIO;
static uint16_t tftCS_PIN;
//Data Command pin
static GPIO_TypeDef  *tftDC_GPIO;
static uint16_t tftDC_PIN;
//Reset pin
static GPIO_TypeDef  *tftRESET_GPIO;
static uint16_t tftRESET_PIN;

void LCD_sendCommand(uint8_t com){
    uint8_t tmpCmd = com;
    HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 0);
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 0);
	HAL_SPI_Transmit(&lcdSPIhandle, &tmpCmd, 1, 1);
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
}

void LCD_sendData(uint8_t data){
	uint8_t tmpDat = data;
	HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 1);
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 0);
	HAL_SPI_Transmit(&lcdSPIhandle, &tmpDat, 1, 1);
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
}

void LCD_sendCommand_NoCS(uint8_t com){
    uint8_t tmpCmd = com;
    HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 0);
	HAL_SPI_Transmit(&lcdSPIhandle, &tmpCmd, 1, 1);
}

void LCD_sendData_NoCS(uint8_t data){
	uint8_t tmpDat = data;
	HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 1);
	HAL_SPI_Transmit(&lcdSPIhandle, &tmpDat, 1, 1);
}

void LCD_sendCommandArg(uint8_t command, uint8_t *dataBytes, uint8_t dataLen){
    HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 0);
	HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 0); //0 for command
    uint8_t buf = command;
    HAL_SPI_Transmit(&lcdSPIhandle, &buf, 1, 1);
    HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 1); //start writing args
    for(int i = 0; i < dataLen; ++i){
        HAL_SPI_Transmit(&lcdSPIhandle, dataBytes, 1, 1);
        dataBytes++;
    }  
    HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
}

void LCD_init(SPI_HandleTypeDef *spiLcdHandle, GPIO_TypeDef *csPORT, uint16_t csPIN, GPIO_TypeDef *dcPORT, uint16_t dcPIN, GPIO_TypeDef *resetPORT, uint16_t resetPIN){
    //Copy SPI settings (...?)
    memcpy(&lcdSPIhandle, spiLcdHandle, sizeof(*spiLcdHandle));
    //set pins and ports
    //CS pin
    tftCS_GPIO = csPORT;
    tftCS_PIN = csPIN;
    HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
    //DC pin
    tftDC_GPIO = dcPORT;
    tftDC_PIN = dcPIN;
    HAL_GPIO_WritePin(tftDC_GPIO, tftDC_PIN, 1);
    //RESET pin
    tftRESET_GPIO = resetPORT;
    tftRESET_PIN = resetPIN;
    HAL_GPIO_WritePin(tftRESET_GPIO, tftRESET_PIN, 1);
    HAL_Delay(10);
    //init commands

    LCD_sendCommand(HX8357_SWRESET);
    HAL_Delay(10);

    uint8_t setC[] = {0xFF, 0x83, 0x57};
    LCD_sendCommandArg(HX8357D_SETC, setC, 3);
    HAL_Delay(500);
    uint8_t setRGB[] = {0x80, 0x00, 0x06, 0x06};
    LCD_sendCommandArg(HX8357_SETRGB, setRGB, 4);
    uint8_t setCOM = 0x25;
    LCD_sendCommandArg(HX8357D_SETCOM, &setCOM, 1);
    uint8_t setOSC = 0x68;
    LCD_sendCommandArg(HX8357_SETOSC, &setOSC, 1);
    uint8_t setPANEL = 0x05;
    LCD_sendCommandArg(HX8357_SETPANEL, &setPANEL, 1);
    uint8_t setPWR1[] = {0x00, 0x15, 0x1C, 0x1C, 0x83, 0xAA};
    LCD_sendCommandArg(HX8357_SETPWR1, setPWR1, 6);
    uint8_t setSTBA[] = {0x50, 0x50, 0x01, 0x3C, 0x1E, 0x08};
    LCD_sendCommandArg(HX8357D_SETSTBA, setSTBA, 6);
    uint8_t setCYC[] = {0x02, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x78};
    LCD_sendCommandArg(HX8357D_SETCYC, setCYC, 7);
    uint8_t setGAMMA[] = {0x02, 0x0A, 0x11, 0x1D, 0x23, 0x35, 0x41, 0x4B,
                        0x4B, 0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03,
                        0x02, 0x0A, 0x11, 0x1D, 0x23, 0x35, 0x41, 0x4B,
                        0x4B, 0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03,
                        0x00, 0x01};
    LCD_sendCommandArg(HX8357D_SETGAMMA, setGAMMA, 34);
    uint8_t setCOLMOD = 0x55;
    LCD_sendCommandArg(HX8357_COLMOD, &setCOLMOD, 1);
    uint8_t setMADCTL = 0xC0;
    LCD_sendCommandArg(HX8357_MADCTL, &setMADCTL, 1);
    uint8_t setTEON = 0x00;
    LCD_sendCommandArg(HX8357_TEON, &setTEON, 1);
    uint8_t setTEARLINE[] = {0x00, 0x02};
    LCD_sendCommandArg(HX8357_TEARLINE, setTEARLINE, 2);

    LCD_sendCommand(HX8357_SLPOUT);
    HAL_Delay(150);
    LCD_sendCommand(HX8357_DISPON);
    HAL_Delay(50);
}

//Graphics function prototypes
void LCD_setCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  uint8_t setCASET[] = {x1 >> 8, x1 & 0xFF, x2 >> 8, x2 & 0xFF};
  LCD_sendCommandArg(HX8357_CASET, setCASET, 4);
  uint8_t setPASET[] = {y1 >> 8, y1 & 0xFF, y2 >> 8, y2 & 0xFF};
  LCD_sendCommandArg(HX8357_PASET, setPASET, 4);
  LCD_sendCommand(HX8357_RAMWR);
  //TODO needs write immediately after (RAMWR)
}
//5. Write data to a single pixel
void LCD_drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  LCD_setCursorPosition(x, y, x, y);
  uint8_t setRAMWR[] = {color >> 8, color & 0xFF};
  LCD_sendCommandArg(HX8357_RAMWR, setRAMWR, 2);
}

void LCD_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color){
	uint32_t n = (x2 - x1) * (y2 - y1);
	LCD_setCursorPosition(x1, y1, x2, y2);

	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 0);
	while (n) {
		n--;
		LCD_sendData_NoCS(color>>8);
		LCD_sendData_NoCS(color&0xff);
	}
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
}
void LCD_fill(uint16_t color){
	uint32_t n = LCD_PIXEL_COUNT;
	LCD_setCursorPosition(0, 0, HX8357_TFTWIDTH-1, HX8357_TFTHEIGHT-1);

	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 0);
	while (n) {
		n--;
		LCD_sendData_NoCS(color>>8);
		LCD_sendData_NoCS(color&0xff);
	}
	HAL_GPIO_WritePin(tftCS_GPIO, tftCS_PIN, 1);
}
