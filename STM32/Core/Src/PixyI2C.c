/*
 * PixyI2C.c
 *
 *  Created on: Apr 14, 2022
 *      Author: davidsun
 */


#include "PixyI2C.h"
#include "main.h"
// I2C Address = 0x54
#define I2C_WRITE 0xA8
#define I2C_READ 0xA9

//Pixy
#define SYNC_START 0xaa55
#define SYNC_START_CC 0xaa56
#define SYNC_X 0x55aa

//extern uint8_t send(uint8_t addr, uint8_t *data, uint8_t len);
//extern uint8_t recv(uint8_t addr, uint8_t *data, uint8_t len);

uint8_t send(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *data, uint8_t len)
{
	uint32_t ret = HAL_I2C_Master_Transmit(i2c, addr, data, len, 1000);
	return ret;
}
uint8_t recv(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *data, uint8_t len)
{
	uint32_t ret = HAL_I2C_Master_Receive(i2c, addr, data, len, 1000);
	return ret;
}

uint16_t getWord(I2C_HandleTypeDef *i2c)
{
	  uint8_t buf[2];
	  uint16_t r;
	  recv(i2c, I2C_READ, &buf[0], 1);
	  recv(i2c, I2C_READ, &buf[1], 1);
	  r = buf[1] << 8;
	  r |= buf[0];
	  return r;
}
enum BlockType getStart(I2C_HandleTypeDef *i2c)
{
	uint16_t w, lw = 0xffff;
	while(1)
	{
		w = getWord(i2c);
		if(w == 0 && lw == 0)
		{
			return NOTFOUND;
		}
		if(w == 0 && lw == 0) // No data found
		{
			return NOTFOUND;
		}
	    else if (w == SYNC_START && lw == SYNC_START)
	    {
	    	return NORMAL;
	    }
	    else if (w == SYNC_START_CC && lw == SYNC_START)
	    {
	    	return COLOR; // code found!
	    }
	    else if (w == SYNC_X) // this is important, we might be juxtaposed
	      recv(i2c, I2C_READ, NULL, 1);

	    lw = w; // save
	}
}

void getBlocks(I2C_HandleTypeDef *i2c, int* buf) //X is 0, Y is 2, none others
{
	// Pixy
	  while(getStart(i2c) == NOTFOUND); //Finds start of frame
	  getWord(i2c); // Checksum word
	  uint16_t signature = getWord(i2c);
	  uint16_t x_center = getWord(i2c);
	  uint16_t y_center = getWord(i2c);
	  // Width/height words come here if needed
	  buf[0] = x_center;
	  buf[1] = y_center;
}
