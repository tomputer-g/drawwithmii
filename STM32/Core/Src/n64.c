// Driver code for the N64.

#include "n64.h"

// Definitions
static TIM_HandleTypeDef *countTim;
static TIM_HandleTypeDef *watchdogTim;
// Data pin
static GPIO_TypeDef *n64_GPIO;
static uint16_t n64_PIN;
// Debug pin
static GPIO_TypeDef *n64_DEBUG_GPIO;
static uint16_t n64_DEBUG_PIN;
// Interrupt timing pin
static GPIO_TypeDef *n64_INT_GPIO;
static uint16_t n64_INT_PIN;

//For interrupts
static uint32_t currentRead;
static uint8_t numRead;
static int shouldRead = 0;
void delay_us(uint16_t us) // blocking delay
{
	__HAL_TIM_SET_COUNTER(countTim, 0); // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(countTim) < us); // wait for the counter to reach the us input in the parameter
}

void writeOne()
{
	uint32_t *writeAdd = (uint32_t *)(GPIOC_ADDR + ODR_OFFSET);
	*writeAdd &= ~(1 << 6);
	delay_us(5);
	*writeAdd |= (1 << 6);
	delay_us(15);
}

void writeZero()
{
	uint32_t *writeAdd = (uint32_t *)(GPIOC_ADDR + ODR_OFFSET);
	*writeAdd &= ~(1 << 6);
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	delay_us(15); // 3.125
	*writeAdd |= (1 << 6);
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	delay_us(5); // 1.375
}

// ret >> 31:            buttonval (?)
//(ret >> 8) & 0xff:    XVal (signed char)
// ret & 0xff:           YVal (signed char)

uint32_t pollRead()
{ // takes ~150us total.
	// poll
	volatile uint32_t *readAdd = (uint32_t *)(GPIOC_ADDR + IDR_OFFSET);
	volatile uint32_t *writeAdd = (uint32_t *)(GPIOC_ADDR + ODR_OFFSET);
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
	delay_us(10);

	// read
	for (int i = 0; i < 31; ++i)
	{
		delay_us(4); // 5
		HAL_GPIO_WritePin(n64_DEBUG_GPIO, n64_DEBUG_PIN, 1);
		buttonVals |= (*readAdd >> 6) & 1; // 0.5u
		HAL_GPIO_WritePin(n64_DEBUG_GPIO, n64_DEBUG_PIN, 0);
		buttonVals = buttonVals << 1;
		delay_us(4);
		if (i % 4)
		{
			delay_us(1);
		}
		//	 if (i==15) {
		//		 delay_us(4);
		//	 }
	}
	delay_us(5);
	buttonVals |= (*readAdd >> 6) & 1;
	delay_us(15);
	return buttonVals;
}

uint32_t intRead(){
	volatile uint32_t *writeAdd = (uint32_t *)(GPIOC_ADDR + ODR_OFFSET);
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
	delay_us(10);
	shouldRead = 1;
	__HAL_TIM_SET_COUNTER(watchdogTim, 0); // 1MHz timer
	while (numRead < 32 && __HAL_TIM_GET_COUNTER(watchdogTim) < 130); //watchDog timeout
	if(numRead < 32){
		currentRead = currentRead << (32 - numRead); //attempt to fix the transmission
	}
	numRead = 0;
	shouldRead = 0;
	return currentRead;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//used by n64 intRead()
	//Falling edge trigger. delay 1.7us then read into static var
	  volatile uint32_t *readAdd = (uint32_t *)(GPIOC_ADDR + IDR_OFFSET);

	if(shouldRead){
	  delay_us(3); //??
	  numRead++;
	  HAL_GPIO_WritePin(n64_DEBUG_GPIO, n64_DEBUG_PIN, 1);
	  currentRead = (currentRead << 1) | ((*readAdd >> 6) & 1);
	  HAL_GPIO_WritePin(n64_DEBUG_GPIO, n64_DEBUG_PIN, 0);
	}
}

void N64_init(TIM_HandleTypeDef *countTimer, TIM_HandleTypeDef *watchdogTimer, GPIO_TypeDef *Data_GPIO, uint16_t Data_PIN, GPIO_TypeDef *Debug_GPIO, uint16_t Debug_PIN, GPIO_TypeDef *Int_GPIO, uint16_t Int_PIN)
{
	countTim = countTimer;
	watchdogTim = watchdogTimer;
	n64_GPIO = Data_GPIO; // due to nature of making C6 inout by registers, this pin assignment does not matter
	n64_PIN = Data_PIN;
	n64_DEBUG_GPIO = Debug_GPIO;
	n64_DEBUG_PIN = Debug_PIN;
	n64_INT_GPIO = Int_GPIO;
	n64_INT_PIN = Int_PIN;
	HAL_TIM_Base_Start(countTim);
	HAL_TIM_Base_Start(watchdogTim);
}
