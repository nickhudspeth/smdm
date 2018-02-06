/*************************************************************************
Title:    tpl0401.h - Driver for Texas Instruments TPL0401x-10-Q1 128-Tap
					  Single-Channel Digital Potentiometer with I2C Interface.
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     tpl0401.h
Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
Hardware: STM32Fxxx
License:  The MIT License (MIT)

DESCRIPTION:

USAGE:
	// Instantiate and initialize a TPL0401 structure
	TPL0401_TypeDef *t =  TPL0401Init(&hi2c1, 3.3f);

	// Configure non-default options, if required.
	t->low_terminal_voltage = 0.9f;

    // Set the desired wiper voltage.
    TPL0401SetWiperVoltage(t, 1.6f);

NOTES:

TO-DO:


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
#ifndef TPL0401_INC_TPL0401_H_
#define TPL0401_INC_TPL0401_H_
/**********************    INCLUDE DIRECTIVES    ***********************/
#include "stm32f0xx_hal.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define TPL0401_DEBUG 1
//#define TPL0401_HIGH_TERMINAL_VOLTAGE 5.0f
#define TPL0401_DEFAULT_LOW_TERMINAL_VOLTAGE 0.0f
#define TPL0401_I2C_SLAVE_ADDRESS 0x2E

typedef struct{
	I2C_HandleTypeDef *hi2c;
	float high_terminal_voltage;
	float low_terminal_voltage;
}TPL0401_TypeDef;

/***********************    FUNCTION PROTOTYPES    ***********************/

TPL0401_TypeDef *TPL0401Init(I2C_HandleTypeDef *hi2c, float htv);
HAL_StatusTypeDef TPL0401SetWiperVoltage(TPL0401_TypeDef *t, float voltage);
#endif /* TPL0401_INC_TPL0401_H_ */
