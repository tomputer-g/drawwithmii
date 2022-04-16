/*
 * PixyI2C.h
 *
 *  Created on: Apr 14, 2022
 *      Author: davidsun
 *  Implementation Details:
 *  	Add PixyI2C.h, PixyI2C.c to /core/src
 *  	Setup I2C in .ioc file
 *  	#include "PixyI2C.h" where needed
 *  	Call getBlocks() with address of I2C (typically &hi2c1 if only one i2c device) and uint8_t buffer array
 */

#ifndef INC_PIXYI2C_H_
#define INC_PIXYI2C_H_

#include "main.h"

enum BlockType {NORMAL, COLOR, NOTFOUND};

uint8_t send(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *data, uint8_t len);
uint8_t recv(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *data, uint8_t len);

uint16_t getWord(I2C_HandleTypeDef *i2c);
enum BlockType getStart(I2C_HandleTypeDef *i2c);

void getBlocks(I2C_HandleTypeDef *i2c, int* buf);


#endif /* INC_PIXYI2C_H_ */
