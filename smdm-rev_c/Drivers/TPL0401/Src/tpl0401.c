/************************************************************************
 Title:	  tpl0401.c - Driver for Texas Instruments TPL0401x-10-Q1 128-Tap
 Single-Channel Digital Potentiometer with I2C Interface.
 Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
 File:     tpl0401.c
 Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
 Hardware: STM32Fxxx
 License:  The MIT License (MIT)
 Usage:    Refer to the header file tpl0401.h for a description of the routines.
 See also example test_tpl0401.c, if available.
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
 ************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/

#include "stdlib.h"
#include "tpl0401.h"
/**********************    GLOBAL VARIABLES    ***********************/

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

TPL0401_TypeDef *TPL0401Init(I2C_HandleTypeDef *hi2c, float htv) {
	TPL0401_TypeDef *t = (TPL0401_TypeDef *) calloc(1, sizeof(TPL0401_TypeDef));
	t->hi2c = hi2c;
	t->high_terminal_voltage = htv;
	t->low_terminal_voltage = TPL0401_DEFAULT_LOW_TERMINAL_VOLTAGE;
	TPL0401SetWiperVoltage(t, 0.0f);
	return t;
}

HAL_StatusTypeDef TPL0401SetWiperVoltage(TPL0401_TypeDef *t, float voltage) {
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t tstatus = 0;
	if ((voltage > t->high_terminal_voltage) || (voltage < t->low_terminal_voltage)) {
		/* Bounds check  on wiper terminal voltage. */
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
		return status;
	}
	/* Calculate the tap setting from the desired wiper voltage and terminal configuration. */
	uint8_t step = (uint8_t) ((voltage * 128.0f)
			/ (t->high_terminal_voltage - t->low_terminal_voltage));
#if TPL0401_DEBUG
	printf("Setting wiper tap to %d.\n", step);
#endif
	if ((tstatus = HAL_I2C_Mem_Write(t->hi2c, TPL0401_I2C_SLAVE_ADDRESS << 1,
			0x00, I2C_MEMADD_SIZE_8BIT, &step, 1, 100)) != HAL_OK) {
#if TPL0401_DEBUG
		printf("I2C transfer returned code %ld\n.", tstatus);
#endif
		status = HAL_ERROR;
		printf("Error setting current limit.\n");
		//_Error_Handler(__FILE__, __LINE__);
	}

	return status;
}
