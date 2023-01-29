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

/* Struct for variables of state of system. 
 * Using struct is useful for parameter passing in function of PID controller
 */
struct Var_data
{
	float 	Err; // Error between target and feedback
	float 	Integ; // Integlral part of output
	float		Filter; // 
	struct	PID_param* PID; // coef of PID controllera
};

extern struct PID_param I_PID_param;
extern struct PID_param V_PID_param;

struct PID_param
{
	volatile float Kp; // P part of PID controller
	volatile float Ki; // I part of PID controller
	volatile float Kd; // D part of PID controller
	volatile float limit; // Limitting of output
	volatile float Nd;
};

extern struct Var_data I_data;				
extern struct Var_data V_data;

extern volatile uint32_t dataADC1[8]; // Array is filled by DMA

extern float volatile Uin;// supply voltage
extern uint32_t volatile Umcu; // mcu voltage
extern float volatile Tboard; // temperature of board
extern float volatile Rntc;  // 
extern int32_t volatile Tmcu; // temperature of mcu

extern volatile float Ifb[3]; // phase currents
extern volatile float Ifb0; // general currnet
extern float volatile Ifb0_tmp; 
extern volatile float Iref; // reference current
extern float const Kfb;  // Kfb=Rshunt*Kopa*KADC
extern float const invKfb;
extern float const Ibias; //Ibias=Ubias/Rshunt/Kop
extern volatile float Vfb; // speed of rotor
extern volatile float Vref; // reference speed of roto
extern volatile float P; // position of rotor in radians
extern uint32_t volatile P_BIT; // position of rotor in bits

extern uint32_t volatile PWM[3];
extern float volatile rangePWM;
extern float volatile rangeEnc;
extern float volatile U0; // output of current regulator
extern const float T1; // sample time for current loop
extern const float T2; // sample time for speed loop
extern float const F1; // frequency of current loop
extern float const F2; // frequency of speed loop
extern uint32_t const zp; // pare pole
extern uint32_t volatile flag_work;
extern uint16_t volatile tick;
extern uint16_t volatile Ntick;

extern float volatile SIN_BIT[4000]; // sin(position of rotor)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const float PI23;
extern const float PI2;

extern const uint32_t PI43_BIT;
extern const uint32_t PI23_BIT;
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
