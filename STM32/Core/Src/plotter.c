#include "plotter.h"

// The timer used for countTim must have a resolution of 1 milisecond!!!!

static TIM_HandleTypeDef *countTim;

unsigned long step_delay = 0;
int direction = 0;
int number_of_steps = 200;
int step_number = 0;
unsigned long last_step_time = 0;
int xdirection = 0;
int ydirection = 0;
int x_step_number = 0;
int y_step_number = 0;

// Pin definitions
static GPIO_TypeDef *A0_GPIO;
static uint16_t A0_PIN;
static GPIO_TypeDef *A1_GPIO;
static uint16_t A1_PIN;
static GPIO_TypeDef *A2_GPIO;
static uint16_t A2_PIN;
static GPIO_TypeDef *A3_GPIO;
static uint16_t A3_PIN;
static GPIO_TypeDef *B0_GPIO;
static uint16_t B0_PIN;
static GPIO_TypeDef *B1_GPIO;
static uint16_t B1_PIN;
static GPIO_TypeDef *B2_GPIO;
static uint16_t B2_PIN;
static GPIO_TypeDef *B3_GPIO;
static uint16_t B3_PIN;

// for step mode let a value of 0 represent half step and
// a value of 1 represent full step mode
int stepMode = 1;

void step_init(TIM_HandleTypeDef *countTimer, int mode, 
GPIO_TypeDef *A0GPIO, uint16_t A0PIN,GPIO_TypeDef *A1GPIO, uint16_t A1PIN,
GPIO_TypeDef *A2GPIO, uint16_t A2PIN,GPIO_TypeDef *A3GPIO, uint16_t A3PIN,
GPIO_TypeDef *B0GPIO, uint16_t B0PIN,GPIO_TypeDef *B1GPIO, uint16_t B1PIN,
GPIO_TypeDef *B2GPIO, uint16_t B2PIN,GPIO_TypeDef *B3GPIO, uint16_t B3PIN)
{
	countTim = countTimer;
	stepMode = mode;
	A0_GPIO = A0GPIO;
	A0_PIN = A0PIN;
	A1_GPIO = A1GPIO;
	A1_PIN = A1PIN;
	A2_GPIO = A2GPIO;
	A2_PIN = A2PIN;
	A3_GPIO = A3GPIO;
	A3_PIN = A3PIN;
	B0_GPIO = B0GPIO;
	B0_PIN = B0PIN;
	B1_GPIO = B1GPIO;
	B1_PIN = B1PIN;
	B2_GPIO = B2GPIO;
	B2_PIN = B2PIN;
	B3_GPIO = B3GPIO;
	B3_PIN = B3PIN;
	
	HAL_TIM_Base_Start(countTimer);
}

void setSpeed(long whatSpeed)
{
	step_delay = (long)60 * 2 * (long)1000 / number_of_steps / whatSpeed;
}

void switchMode(int mode)
{
	stepMode = mode;
}

void stopStep(int axis)
{
	if (!axis)
	{
		HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
		HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
		HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
		HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
	}
	else
	{
		HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
		HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
		HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
		HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
	}
}

void stepDiag(int x_steps_to_move, int y_steps_to_move)
{
	int lastMov = 0;

	int x_steps_left = abs(x_steps_to_move);
	int y_steps_left = abs(y_steps_to_move);
	if (x_steps_to_move > 0)
	{
		xdirection = 1;
	}
	if (x_steps_to_move < 0)
	{
		xdirection = 0;
	}
	if (y_steps_to_move > 0)
	{
		ydirection = 1;
	}
	if (y_steps_to_move < 0)
	{
		ydirection = 0;
	}
	while (x_steps_left > 0 || y_steps_left > 0)
	{
		unsigned long now = __HAL_TIM_GET_COUNTER(countTim);
		if (now - last_step_time >= step_delay)
		{
			last_step_time = now;
			if (!lastMov || y_steps_left == 0)
			{
				lastMov = 1;
				if (xdirection == 1)
				{
					x_step_number++;
					if (x_step_number == number_of_steps)
					{
						x_step_number = 0;
					}
				}
				else
				{
					if (x_step_number == 0)
					{
						x_step_number = number_of_steps;
					}
					x_step_number--;
				}
				x_steps_left--;
				if (stepMode)
				{
					stepMotor(x_step_number % 4, 0);
				}
				else
				{
					stepMotor(x_step_number % 8, 0);
				}
			}
			if (lastMov || x_steps_left == 0)
			{
				lastMov = 0;
				if (ydirection == 1)
				{
					y_step_number++;
					if (y_step_number == number_of_steps)
					{
						y_step_number = 0;
					}
				}
				else
				{
					if (y_step_number == 0)
					{
						y_step_number = number_of_steps;
					}
					y_step_number--;
				}
				y_steps_left--;
				if (stepMode)
				{
					stepMotor(y_step_number % 4, 1);
				}
				else
				{
					stepMotor(y_step_number % 8, 1);
				}
			}
			if (x_steps_left == 0)
			{
				stopStep(0);
			}
			if (y_steps_left == 0)
			{
				stopStep(1);
			}
		}
	}
}

void step(int steps_to_move, int axis)
{
	int steps_left = abs(steps_to_move);
	if (steps_to_move > 0)
	{
		direction = 1;
	}
	else if (steps_to_move < 0)
	{
		direction = 0;
	}

	while (steps_left > 0)
	{
		unsigned long now = __HAL_TIM_GET_COUNTER(countTim);
		if (now - last_step_time >= step_delay)
		{
			last_step_time = now;
			if (direction == 1)
			{
				step_number++;
				if (step_number == number_of_steps)
				{
					step_number = 0;
				}
			}
			else
			{
				if (step_number == 0)
				{
					step_number = number_of_steps;
				}
				step_number--;
			}
			steps_left--;
			if (stepMode)
			{
				stepMotor(step_number % 4, axis);
			}
			else
			{
				stepMotor(step_number % 8, axis); // was 7
			}
		}
	}
	// why can't I put stopStep() here??
}

void stepStop(int steps, int axis)
{
	step(steps, axis); // doesn't work either
	stopStep(axis);
}

void stepMotor(int thisStep, int axis)
{
	if (!axis)
	{
		if (stepMode)
		{
			// lettering is notationaly consistant with http://www.idc-online.com/technical_references/pdfs/electrical_engineering/Step_Sequence_of_Stepper_Motor.pdf
			switch (thisStep)
			{
			case 0:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 1); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 1); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;

			case 1:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 1); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 1); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;

			case 2:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 1); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 1); // c
				break;

			case 3:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 1); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 1); // c
				break;
			}
		}
		else
		{
			switch (thisStep)
			{
			case 0:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 1); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 1); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;

			case 1:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 1); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;

			case 2:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 1); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 1); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;

			case 3:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 1); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;
			case 4:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 1); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 1); // c
				break;
			case 5:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 0); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 1); // c
				break;
			case 6:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 1); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 1); // c
				break;
			case 7:
				HAL_GPIO_WritePin(A0_GPIO, A0_PIN, 1); // d
				HAL_GPIO_WritePin(A1_GPIO, A1_PIN, 0); // b
				HAL_GPIO_WritePin(A2_GPIO, A2_PIN, 0); // a
				HAL_GPIO_WritePin(A3_GPIO, A3_PIN, 0); // c
				break;
			}
		}
	}
	else
	{
		if (stepMode)
		{
			switch (thisStep)
			{
				// lettering is notationaly consistant with http://www.idc-online.com/technical_references/pdfs/electrical_engineering/Step_Sequence_of_Stepper_Motor.pdf
			case 0:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 1); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 1); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;

			case 1:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 1); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 1); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;

			case 2:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 1); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 1); // c
				break;

			case 3:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 1); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 1); // c
				break;
			}
		}
		else
		{
			switch (thisStep)
			{
			case 0:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 1); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 1); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;

			case 1:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 1); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;

			case 2:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 1); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 1); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;

			case 3:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 1); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;
			case 4:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 1); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 1); // c
				break;
			case 5:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 0); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 1); // c
				break;
			case 6:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 1); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 1); // c
				break;
			case 7:
				HAL_GPIO_WritePin(B0_GPIO, B0_PIN, 1); // d
				HAL_GPIO_WritePin(B1_GPIO, B1_PIN, 0); // b
				HAL_GPIO_WritePin(B2_GPIO, B2_PIN, 0); // a
				HAL_GPIO_WritePin(B3_GPIO, B3_PIN, 0); // c
				break;
			}
		}
	}
}
