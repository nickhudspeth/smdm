/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "tpl0401.h"
#include "drv8825.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t temp_conv_complete = 0;
volatile int32_t ADC_values[2] = { 0 }; /* CHANNEL_9, CHANNEL_TEMP */
int32_t temp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles(void);
int32_t GetTemperature(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* ! \tpdo Debounce triggers here. */
	switch (GPIO_Pin) {
	case FAULT_Pin:
		printf("DRV8825 fault condition triggered.\n");
		break;
	case LIML_Pin:
		printf("Low-side limit switch triggered.\n");
		break;
	case LIMH_Pin:
		printf("High-side limit switch triggered.\n");
		break;
	default:
		break;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		DRV8825_PWMInterruptHandler(htim);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16) {
		DRV8825_AccelInterruptHandler(htim);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	static uint8_t index = 0;
	int32_t vdd = 0;
	int32_t temperature = 0; /* Temperature in degrees Celsius*/
	/* Orcer of conversion is CHANNEL_9, TEMP_CHANNEL, VREF_CHANNEL */
	static int32_t ADC_raw_values[3] = { 0 };
	/* Perform these actions at the end of conversion for each channel. */
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
		/* Read the converted ADC value for the current channel and increment the index*/
		ADC_raw_values[index++] = HAL_ADC_GetValue(hadc);
	}

	/* Perform these actions after all channels have been read. */
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) {
		index = 0;
		vdd = 3300 * (*VREFINT_CAL_ADDR) / ADC_raw_values[2];
		temperature = (((int32_t) ADC_raw_values[1] * (vdd / 3300))
				- (int32_t) *TEMP30_CAL_ADDR);
		//temperature = (((int32_t)ADC_raw_values[1] * (3000/3300)) - (int32_t) *TEMP30_CAL_ADDR);
		temperature = temperature * (int32_t) (110 - 30);
		temperature = temperature
				/ (int32_t) (*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
		temperature = temperature + 30;

		ADC_values[0] = (vdd * ADC_raw_values[0] / 4095);
		ADC_values[1] = temperature;
		temp_conv_complete = 1;
	}
}

int32_t GetTemperature(void) {
	uint32_t start = HAL_GetTick();
	temp_conv_complete = 0;
	HAL_ADC_Start_IT(&hadc);
	/* Wait up to 100ms for conversion to complete*/
	while ((HAL_GetTick() - start) < 100) {
		if (temp_conv_complete) {
			return ADC_values[1];
		}
	}
	return 0;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	/* Enable semihosting. TURN THIS OFF FOR THE PRODUCTION BUILD OR CHIP WILL HANG HERE! */

	//initialise_monitor_handles();
	/* Start timer for heartbeat LED*/
	if (HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	DRV8825_TypeDef drv;
	drv.DecayPort = DECAY_GPIO_Port;
	drv.DirPort = DIR_GPIO_Port;
	drv.EnablePort = ENABLE_GPIO_Port;
	drv.StepPort = STEP_GPIO_Port;
	drv.Mode0Port = MODE0_GPIO_Port;
	drv.Mode1Port = MODE1_GPIO_Port;
	drv.Mode2Port = MODE2_GPIO_Port;
	drv.DecayPin = DECAY_Pin;
	drv.DirPin = DIR_Pin;
	drv.EnablePin = ENABLE_Pin;
	drv.StepPin = STEP_Pin;
	drv.Mode0Pin = MODE0_Pin;
	drv.Mode1Pin = MODE1_Pin;
	drv.Mode2Pin = MODE2_Pin;
	drv.StepsPerRevolution = 200;
	drv.MicrostepResolution = DRV8825_MICROSTEP_SIXTEENTH;
	drv.Acceleration = 2000;
	drv.AccelTimerHandle = &htim16;
	drv.AccelTimerChannel = TIM_CHANNEL_1;
	drv.PWMTimerHandle = &htim2;
	drv.PWMTimerChannel = TIM_CHANNEL_1;
	drv.I2CHandle = &hi2c1;
	DRV8825_Init(&drv);
	DRV8825_MoveContinuous(&drv, DRV8825_DIR_CCW, 1000);
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		temp = GetTemperature();
		//printf("temperature = %ld.\n", temp);
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_SET);
	printf("Error handler entered from %s:%d\n", file, line);
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
