/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct Var_data
{
	float 	Err;
	float 	Integ;
	float		Filter;
	struct	PID_param* PID;
};

extern struct PID_param I_PID_param;
extern struct PID_param V_PID_param;

struct PID_param
{
	volatile float Kp;
	volatile float Ki;
	volatile float Kd;
	volatile float limit;
	volatile float Nd;
};

extern struct Var_data I_data;				
extern struct Var_data V_data;

extern volatile uint32_t dataADC1[8];

extern float volatile Uin;
extern uint32_t volatile Umcu;
extern float volatile Tboard;
extern float volatile Rntc; 
extern int32_t volatile Tmcu;

extern volatile float Ifb[3];
extern volatile float Ifb0;
extern float volatile Ifb0_tmp; // general currnet
extern volatile float Iref;
extern float const Kfb;
extern float const invKfb;
extern float const Ibias; 
extern volatile float Vfb;
extern volatile float Vref;
extern volatile float P;
extern uint32_t volatile P_BIT; // position of rotor

extern uint32_t volatile PWM[3];
extern float volatile rangePWM;
extern float volatile rangeEnc;
extern float volatile U0;
extern const float T1;
extern const float T2;
extern float const F1; //
extern float const F2; // 
extern uint32_t const zp;
extern uint32_t volatile flag_work;
extern uint16_t volatile tick;
extern uint16_t volatile Ntick;

extern float volatile SIN_BIT[4000]; // sin(position of rotor)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const float PI;
extern const float PI23;
extern const float PI43;
extern const float PI2;

extern const uint32_t PI_BIT;
extern const uint32_t PI23_BIT;
extern const uint32_t PI43_BIT;
extern const uint32_t PI2_BIT;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float Control_PI( struct Var_data *Data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
