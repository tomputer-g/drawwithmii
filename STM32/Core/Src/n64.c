//Driver code for the N64.

#include "n64.h"

static TIM_HandleTypeDef* countTim;


//??
void delay_us (uint16_t us)//blocking delay
{
	__HAL_TIM_SET_COUNTER(countTim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(countTim) < us);  // wait for the counter to reach the us input in the parameter
}

void writeOne() {
	uint32_t* writeAdd = (uint32_t*)(GPIOC_ADDR + ODR_OFFSET);
	*writeAdd &= ~(1 << 6);
	delay_us(5);
	*writeAdd |= (1 << 6);
	delay_us(15);
}

void writeZero() {
	uint32_t* writeAdd = (uint32_t*)(GPIOC_ADDR + ODR_OFFSET);
	*writeAdd &= ~(1 << 6);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	delay_us(15); //3.125
	*writeAdd |= (1 << 6);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	delay_us(5); //1.375
}

//ret >> 31:            buttonval (?)
//(ret >> 8) & 0xff:    XVal (signed char)
//ret & 0xff:           YVal (signed char)
uint32_t pollRead() {
	//poll
	volatile uint32_t* readAdd = (uint32_t*)(GPIOC_ADDR + IDR_OFFSET);
	volatile uint32_t* writeAdd = (uint32_t*)(GPIOC_ADDR + ODR_OFFSET);
	uint32_t buttonVals = 0;
	writeZero();
	writeZero();
	writeZero();
	writeZero();
	writeZero();
	writeZero();
	writeZero();
	writeOne();
	*writeAdd &= ~(1 << 6);
	delay_us(5);
	*writeAdd |= (1 << 6);
	delay_us(9);

	//read
	for(int i = 0; i < 31; ++i) {
	delay_us(5);//5
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	buttonVals |= (*readAdd >> 6) & 1; //0.5u
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	buttonVals = buttonVals << 1;
	delay_us(8);
//	 if(i % 2 == 0) {
//		delay_us(2);
//	}
//	 if (i==15) {
//		 delay_us(4);
//	 }
	}
	delay_us(5);
	buttonVals |= (*readAdd >> 6) & 1;
	delay_us(15);
	return buttonVals;
}

void N64_init(TIM_HandleTypeDef* countTimer){
	countTim = countTimer;
	HAL_TIM_Base_Start(countTim);
	//TODO future: add pin/gpio of all pins involved
}
