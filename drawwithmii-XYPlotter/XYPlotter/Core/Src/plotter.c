#include "plotter.h"

//The timer used for countTim must have a resolution of 1 milisecond!!!!

static TIM_HandleTypeDef* countTim;

unsigned long step_delay = 0;
int direction = 0;
int number_of_steps = 200;
int pin_count = 4;
int step_number = 0;
unsigned long last_step_time = 0;

//for step mode let a value of 0 represent half step and
//a value of 1 represent full step mode
int stepMode = 1;

void step_init(int pin_num, TIM_HandleTypeDef* countTimer, int mode) {
    if(pin_num != 4) {
        exit(1);
    }
countTim = countTimer;
stepMode = mode;
 HAL_TIM_Base_Start(countTimer);
}

void setSpeed(long whatSpeed) {
	step_delay = (long)60 * (long)1000 / number_of_steps / whatSpeed;
}

void switchMode(int mode) {
	stepMode = mode;
}


void step(int steps_to_move, int axis) {
	int steps_left = abs(steps_to_move);
	if(steps_to_move > 0) {
		direction = 1;
		}
	if(steps_to_move < 0) {
		direction = 0;
		}

	while(steps_left > 0) {
		unsigned long now = __HAL_TIM_GET_COUNTER(countTim);
		if(now - last_step_time >= step_delay) {
			last_step_time = now;
			if(direction == 1) {
				step_number++;
				if(step_number == number_of_steps) {
					step_number = 0;
				}
			}
			else {
				if(step_number == 0) {
					step_number = number_of_steps;
				}
				step_number--;
			}
			steps_left--;
			if(stepMode) {
				stepMotor(step_number % 4, axis);
			}
			else {
				stepMotor(step_number % 7, axis);
			}
	}
	}
}

void stepMotor(int thisStep, int axis) {
	if(!axis) {
		if(stepMode) {
			//lettering is notationaly consistant with http://www.idc-online.com/technical_references/pdfs/electrical_engineering/Step_Sequence_of_Stepper_Motor.pdf
			switch(thisStep) {
			case 0:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //c
				break;

			case 1:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //d
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //b
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //c
				break;

			case 2:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //d
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //b
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0); //a
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1); //c
				break;

			case 3:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0); //a
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1); //c
				break;
				}
			}
		else {
			switch(thisStep) {
			case 0:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //c
				break;

			case 1:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				break;

			case 2:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				break;

			case 3:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				break;
				}
		}
	}
	else {
		if(stepMode) {
			switch(thisStep) {
				//lettering is notationaly consistant with http://www.idc-online.com/technical_references/pdfs/electrical_engineering/Step_Sequence_of_Stepper_Motor.pdf
			case 0:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0); //c
				break;

			case 1:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //d
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); //b
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0); //c
				break;

			case 2:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //d
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); //b
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); //a
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //c
				break;

			case 3:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); //a
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //c
				break;
				}
			}
		else {
			switch(thisStep) {
			case 0:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //d
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //b
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); //a
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0); //c
				break;

			case 1:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
				break;

			case 2:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
				break;

			case 3:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
				break;
				}
		}
	}

	}

 
