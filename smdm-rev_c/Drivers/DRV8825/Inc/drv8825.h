/*************************************************************************
Title:    drv8825.h - 
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     drv8825.h
Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
Hardware: STM32Fxxx
License:  The MIT License (MIT)

DESCRIPTION:


USAGE:

NOTES:

TO-DO:
    Is it feasible to use the same timer for PWM and accelerations?

LICENSE:
    Copyright (C) 2018 Pathogen Systems, Inc. dba Crystal Diagnostics

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*************************************************************************/
#ifndef DRV8825_INC_DRV8825_H_
#define DRV8825_INC_DRV8825_H_
/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include "stm32f0xx_hal.h"
#include "tpl0401.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define DRV8825_DEBUG 0

#define DRV8825_DRIVER_DRV8825 0
#define DRV8825_MICROSTEP_FULL 1
#define DRV8825_MICROSTEP_HALF 2
#define DRV8825_MICROSTEP_QUARTER 4
#define DRV8825_MICROSTEP_EIGHTH 8
#define DRV8825_MICROSTEP_SIXTEENTH 16
#define DRV8825_MICROSTEP_THIRTYSECOND 32

#define DRV8825_DIR_CW 1
#define DRV8825_DIR_CCW 2

#define DRV8825_DEFAULT_VELOCITY 20 /*Full steps/second*/

#define DRV8825_MOVE_CONTINUOUS 1
#define DRV8825_MOVE_FINITE 2

#define DRV8825_PWM_TIMER_FREQUENCY  (unsigned long)500000
#define DRV8825_ISENSE_RESISTOR_VALUE 0.2f
#define DRV8825_MAX_WINDING_CURRENT 2.5f
#define DRV8825_HOLD_CURRENT 0.1f
#define DRV8825_INITIAL_MOVE_CURRENT 0.1f

#define DRV8825_DECAY_MODE_SLOW  0
#define DRV8825_DECAY_MODE_FAST  1
#define DRV8825_DECAY_MODE_MIXED 2
#define DRV8825_VEL_MAX ((uint32_t)250000)

typedef struct{
	TIM_HandleTypeDef *PWMTimerHandle;
	TIM_HandleTypeDef *AccelTimerHandle;
	I2C_HandleTypeDef *I2CHandle;
	uint32_t PWMTimerChannel;
	uint32_t AccelTimerChannel;
	GPIO_TypeDef* DecayPort;
	GPIO_TypeDef* DirPort;
	GPIO_TypeDef* EnablePort;
	GPIO_TypeDef* StepPort;
	GPIO_TypeDef* Mode0Port;
	GPIO_TypeDef* Mode1Port;
	GPIO_TypeDef* Mode2Port;
	uint16_t DecayPin;
	uint16_t DirPin;
	uint16_t EnablePin;
	uint16_t StepPin;
	uint16_t Mode0Pin;
	uint16_t Mode1Pin;
	uint16_t Mode2Pin;
	uint16_t MicrostepResolution;
	uint16_t StepsPerRevolution;
	double Acceleration;
	/* PRIVATE */
	uint16_t microstepsPerRevolution;
	uint16_t currentRotationDirection;
	volatile float currentVelocity;
	volatile float targetVelocity;
	volatile uint32_t stepsRemaining;
	volatile uint16_t moveMode;
	TPL0401_TypeDef *tpl0401;
}DRV8825_TypeDef;
/***********************    FUNCTION PROTOTYPES    ***********************/
extern void _Error_Handler(char *, int);
HAL_StatusTypeDef DRV8825_Init(DRV8825_TypeDef *d);
HAL_StatusTypeDef DRV8825_SetRotationDirection(DRV8825_TypeDef *const d, const uint16_t dir);
HAL_StatusTypeDef DRV8825_SetMicrostepResolution(DRV8825_TypeDef *const d, const uint16_t res);
HAL_StatusTypeDef DRV8825_SetVelocity(DRV8825_TypeDef *const d, uint16_t vel);
HAL_StatusTypeDef DRV8825_MoveSteps(DRV8825_TypeDef *const d, const uint8_t dir, const uint16_t vel, const uint32_t steps);
HAL_StatusTypeDef DRV8825_MoveContinuous(DRV8825_TypeDef *const d, const uint8_t dir, const uint16_t vel);
HAL_StatusTypeDef DRV8825_Start(DRV8825_TypeDef *const d);
HAL_StatusTypeDef DRV8825_Stop(DRV8825_TypeDef *const d);
HAL_StatusTypeDef DRV8825_SetCurrent(DRV8825_TypeDef *const d, float current);
HAL_StatusTypeDef DRV8825_SetDecayMode(DRV8825_TypeDef *const d, uint8_t mode);
void DRV8825_SetAcceleration(DRV8825_TypeDef * const d, float accel);

void DRV8825_PWMInterruptHandler(TIM_HandleTypeDef *htim);
void DRV8825_AccelInterruptHandler(TIM_HandleTypeDef *htim);
#endif /* DRV8825_INC_DRV8825_H_ */
