/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define FAULT_Pin GPIO_PIN_0
#define FAULT_GPIO_Port GPIOF
#define FAULT_EXTI_IRQn EXTI0_1_IRQn
#define LIML_Pin GPIO_PIN_3
#define LIML_GPIO_Port GPIOA
#define LIML_EXTI_IRQn EXTI2_3_IRQn
#define HEARTBEAT_Pin GPIO_PIN_4
#define HEARTBEAT_GPIO_Port GPIOA
#define LIMH_Pin GPIO_PIN_5
#define LIMH_GPIO_Port GPIOA
#define LIMH_EXTI_IRQn EXTI4_15_IRQn
#define MODE2_Pin GPIO_PIN_6
#define MODE2_GPIO_Port GPIOA
#define MODE1_Pin GPIO_PIN_7
#define MODE1_GPIO_Port GPIOA
#define MODE0_Pin GPIO_PIN_0
#define MODE0_GPIO_Port GPIOB
#define ASA_Pin GPIO_PIN_1
#define ASA_GPIO_Port GPIOB
#define ASDIR_Pin GPIO_PIN_8
#define ASDIR_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_15
#define STEP_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_3
#define ENABLE_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOB
#define DECAY_Pin GPIO_PIN_5
#define DECAY_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
