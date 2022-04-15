//header for plotter.c
#ifndef INC_plotter_H_
#define INC_plotter_H_

#include <stdlib.h>
#include "stm32l4xx_hal.h"

//let ports A0-A3 represent the X axis and ports B0-B3 represent the Y axis
//for simplicity's sake assume both axes must be run in the same mode
void step_init(TIM_HandleTypeDef *countTimer, int mode, 
GPIO_TypeDef *A0GPIO, uint16_t A0PIN,GPIO_TypeDef *A1GPIO, uint16_t A1PIN,
GPIO_TypeDef *A2GPIO, uint16_t A2PIN,GPIO_TypeDef *A3GPIO, uint16_t A3PIN,
GPIO_TypeDef *B0GPIO, uint16_t B0PIN,GPIO_TypeDef *B1GPIO, uint16_t B1PIN,
GPIO_TypeDef *B2GPIO, uint16_t B2PIN,GPIO_TypeDef *B3GPIO, uint16_t B3PIN);

//shouldn't be called by a user, only for the step function
void stepMotor(int thisStep, int axis);

void stepDiag(int x_steps_to_move, int y_steps_to_move);
//sets rotation speed in RPM, saves the value as a milisecond delay
//both axes will run at the same RPM to avoid desyncs
void setSpeed(long whatSpeed);

/*give a number of steps, with 200 representing a full revolution, and which axis to move
0 represents the X axis, 1 represents the Y axis */
void stopStep(int axis);

void step(int steps_to_move, int axis);
void stepStop(int steps, int axis);

//if you want to change modes when the stepper is near the desired location use this
void switchMode(int mode);

#endif
