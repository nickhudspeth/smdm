/************************************************************************
 Title:	  drv8825.c -
 Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
 File:     drv8825.c
 Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
 Hardware: STM32Fxxx
 License:  The MIT License (MIT)
 Usage:    Refer to the header file drv8825.h for a description of the routines.
 See also example test_drv8825.c, if available.
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

#include "drv8825.h"

/**********************    GLOBAL VARIABLES    ***********************/
DRV8825_TypeDef **DRV8825List; /*List of allocated DRV8825 objects*/
uint16_t DRV8825ListLen = 0;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

HAL_StatusTypeDef DRV8825_Init(DRV8825_TypeDef *d)
{

	/* Check to make sure required fields are properly initialized here. */
	/* Ensure GPIOs are properly set up here. */

	/* Make sure driver is disabled. */
	HAL_GPIO_WritePin(d->EnablePort, d->EnablePin, GPIO_PIN_SET);
	d->currentVelocity = 0;
	d->targetVelocity = 0;
	d->stepsRemaining = 0;
	DRV8825_SetMicrostepResolution(d, d->MicrostepResolution);
	DRV8825_SetDecayMode(d, DRV8825_DECAY_MODE_SLOW);
	/* Add DRV8825 instance to DRV8825List */
	DRV8825List = (DRV8825_TypeDef **)realloc(DRV8825List,
											  (++DRV8825ListLen) * sizeof(DRV8825_TypeDef *));
	DRV8825List[DRV8825ListLen - 1] = d;
	/* Configure digital potentiometer */
	d->tpl0401 = TPL0401Init(d->I2CHandle, 3.3f);
	DRV8825_SetCurrent(d, 0.0f);
	return HAL_OK;
}

HAL_StatusTypeDef DRV8825_SetRotationDirection(DRV8825_TypeDef *const d,
											   const uint16_t dir)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	switch (dir)
	{
	case DRV8825_DIR_CW:
		HAL_GPIO_WritePin(d->DirPort, d->DirPin, GPIO_PIN_RESET);
		status = HAL_OK;
		break;
	case DRV8825_DIR_CCW:
		HAL_GPIO_WritePin(d->DirPort, d->DirPin, GPIO_PIN_SET);
		status = HAL_OK;
		break;
	default:
		/* DRV8825 uninitialized or unknown rotation direction specified. */
		_Error_Handler(__FILE__, __LINE__);
		break;
	}
	return status;
}

HAL_StatusTypeDef DRV8825_SetMicrostepResolution(DRV8825_TypeDef *const d,
												 const uint16_t res)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	switch (res)
	{
	case DRV8825_MICROSTEP_FULL:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_RESET);
		status = HAL_OK;
		break;
	case DRV8825_MICROSTEP_HALF:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_SET);
		status = HAL_OK;
		break;
	case DRV8825_MICROSTEP_QUARTER:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_RESET);
		status = HAL_OK;
		break;
	case DRV8825_MICROSTEP_EIGHTH:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_SET);
		status = HAL_OK;
		break;
	case DRV8825_MICROSTEP_SIXTEENTH:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_RESET);
		status = HAL_OK;
		break;
	case DRV8825_MICROSTEP_THIRTYSECOND:
		HAL_GPIO_WritePin(d->Mode2Port, d->Mode2Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(d->Mode1Port, d->Mode1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(d->Mode0Port, d->Mode0Pin, GPIO_PIN_SET);
		status = HAL_OK;
		break;
	default:
		/* DRV8825 instance uninitialized or unknown microstep resolution specified. */
		_Error_Handler(__FILE__, __LINE__);
		break;
	}
	d->MicrostepResolution = res;
	d->microstepsPerRevolution = d->StepsPerRevolution * res;
	return status;
}

HAL_StatusTypeDef DRV8825_SetVelocity(DRV8825_TypeDef *const d, uint16_t vel)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t vel_max = 250000;
	/* Set timer interrupt frequency to match desired motor velocity in steps/second */
	if (vel != 0)
	{
		vel *= d->MicrostepResolution;
		if (vel > vel_max)
		{
			vel = vel_max;
		}
		d->targetVelocity = vel;

		status = HAL_OK;
	}
	else
	{
		/* DRV8825 instance uninitialized or zero velocity specified. */
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	return status;
}

HAL_StatusTypeDef DRV8825_MoveSteps(DRV8825_TypeDef *const d,
									const uint8_t dir, const uint16_t vel, const uint32_t steps)
{
	HAL_StatusTypeDef status = HAL_OK;
	d->moveMode = DRV8825_MOVE_FINITE;
	d->stepsRemaining = steps;
	if (DRV8825_SetVelocity(d, vel) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (DRV8825_SetRotationDirection(d, dir) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (DRV8825_Start(d) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	return status;
}

HAL_StatusTypeDef DRV8825_MoveContinuous(DRV8825_TypeDef *const d,
										 const uint8_t dir, const uint16_t vel)
{
	HAL_StatusTypeDef status = HAL_OK;
	d->moveMode = DRV8825_MOVE_CONTINUOUS;
	if (DRV8825_SetVelocity(d, vel) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (DRV8825_SetRotationDirection(d, dir) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (DRV8825_Start(d) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	return status;
}

HAL_StatusTypeDef DRV8825_Start(DRV8825_TypeDef *const d)
{
	HAL_StatusTypeDef status = HAL_OK;

	/* Enable driver */
	HAL_GPIO_WritePin(d->EnablePort, d->EnablePin, GPIO_PIN_RESET);
	if (DRV8825_SetCurrent(d, DRV8825_INITIAL_MOVE_CURRENT) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Start PWM timer */
	if (HAL_TIM_PWM_Start_IT(d->PWMTimerHandle, d->PWMTimerChannel) != HAL_OK)
	{
		status = HAL_ERROR;
		/* Disable driver */
		HAL_GPIO_WritePin(d->EnablePort, d->EnablePin, GPIO_PIN_SET);
		_Error_Handler(__FILE__, __LINE__);
	}

	//	/* Start acceleration timer */
	if (HAL_TIM_OC_Start_IT(d->AccelTimerHandle, d->AccelTimerChannel) != HAL_OK)
	{
		status = HAL_ERROR;
		/* Disable driver */
		HAL_GPIO_WritePin(d->EnablePort, d->EnablePin, GPIO_PIN_SET);
		_Error_Handler(__FILE__, __LINE__);
	}
	return status;
}

HAL_StatusTypeDef DRV8825_Stop(DRV8825_TypeDef *const d)
{
	HAL_StatusTypeDef status = HAL_OK;
	if (HAL_TIM_OC_Stop_IT(d->AccelTimerHandle, d->AccelTimerHandle->Channel) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_TIM_PWM_Stop_IT(d->PWMTimerHandle, d->PWMTimerHandle->Channel) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
	if (DRV8825_SetCurrent(d, DRV8825_HOLD_CURRENT) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
	}
#if DRV8825_DEBUG
	printf("Target reached.\n");
#endif
	return status;
}

HAL_StatusTypeDef DRV8825_SetCurrent(DRV8825_TypeDef *const d, float current)
{
	HAL_StatusTypeDef status = HAL_OK;
	if (current > DRV8825_MAX_WINDING_CURRENT)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
		return status;
	}
	float v = 5.0f * current * DRV8825_ISENSE_RESISTOR_VALUE; /* DRV8825 VRef amplifier gain = 5 */
	if (TPL0401SetWiperVoltage(d->tpl0401, v) != HAL_OK)
	{
		status = HAL_ERROR;
		_Error_Handler(__FILE__, __LINE__);
		return status;
	}

	return status;
}

HAL_StatusTypeDef DRV8825_SetDecayMode(DRV8825_TypeDef *const d, uint8_t mode)
{
	HAL_StatusTypeDef status = HAL_OK;

	switch (mode)
	{
	case DRV8825_DECAY_MODE_SLOW:
		HAL_GPIO_WritePin(d->DecayPort, d->DecayPin, GPIO_PIN_RESET);
		break;
	case DRV8825_DECAY_MODE_FAST:
		HAL_GPIO_WritePin(d->DecayPort, d->DecayPin, GPIO_PIN_SET);
		break;
	case DRV8825_DECAY_MODE_MIXED:
		/* Set pin to high-impedance input mode for floating state. */
		break;
	}

	return status;
}

void DRV8825_SetAcceleration(DRV8825_TypeDef *const d, float accel)
{
	d->Acceleration = accel;
}

/* PWM TIMER INTERRUPTS */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	for (int i = 0; i < DRV8825ListLen; i++) {
//		if (htim->Instance == DRV8825List[i]->PWMTimerHandle->Instance) {
//			HAL_GPIO_WritePin(DRV8825List[i]->StepPort, DRV8825List[i]->StepPin,
//					GPIO_PIN_SET);
//		}
//	}
//}
void DRV8825_PWMInterruptHandler(TIM_HandleTypeDef *htim)
{
	/* Only traverse the list if more than one DRV8825 has been allocated. */
	DRV8825_TypeDef *d = NULL;
	if (DRV8825ListLen > 1)
	{
		for (int i = 0; i < DRV8825ListLen; i++)
		{
			d = DRV8825List[i];
			if ((htim->Instance == DRV8825List[i]->PWMTimerHandle->Instance) && (htim->Channel == DRV8825List[i]->PWMTimerHandle->Channel))
			{
				break;
			}
		}
	}
	else if (DRV8825ListLen == 1)
	{
		d = DRV8825List[0];
	}
	if (d->moveMode == DRV8825_MOVE_FINITE)
	{
		if (--(d->stepsRemaining) == 0)
		{
			DRV8825_Stop(d);
		}
	}
}

void DRV8825_AccelInterruptHandler(TIM_HandleTypeDef *htim)
{
	/* This interrupt fires every 10ms (100Hz) to update the velocity
	 * based on the acceleration setting. */

	DRV8825_TypeDef *d = NULL;
	/* Only traverse the list if more than one DRV8825 has been allocated. */
	if (DRV8825ListLen > 1)
	{
		for (int i = 0; i < DRV8825ListLen; i++)
		{
			d = DRV8825List[i];
			if ((htim->Instance == d->AccelTimerHandle->Instance) && (htim->Channel == d->AccelTimerHandle->Channel))
			{
				break;
			}
		}
	}
	else if (DRV8825ListLen == 1)
	{
		d = DRV8825List[0];
	}
	/* Make sure valid DRV8825 instance exists in list. */
	if (d != NULL)
	{
		float tv = d->targetVelocity;
		float cv = d->currentVelocity;
		float a = d->Acceleration;
		/* Only do the math if current velocity is less than target velocity. */
		if (cv < tv)
		{
			/* Increase the current velocity. */
			float tmp = cv + (0.004f * a);
			d->currentVelocity = (tmp <= tv) ? tmp : tv;

			/* Round timer period to nearest integer value for desired value in steps/sec.
			 * Floating point rounding is OK here, as this function is called infrequently. */
			uint32_t tmp2 = (uint32_t)((DRV8825_PWM_TIMER_FREQUENCY / tmp) + 0.5f);
			__HAL_TIM_SET_AUTORELOAD(d->PWMTimerHandle, tmp2);
		}
	}
	else
	{
		/* No DRV8825 Instance found in list. */
		_Error_Handler(__FILE__, __LINE__);
	}
}
