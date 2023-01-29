/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	tick = LL_TIM_GetCounter(TIM4);
	static uint32_t count=0; // count is used to reduce frequency of velocity loop in 10 time
	static float Pdel=0;
	static uint32_t direct=0; // direction of rotate
  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
	P_BIT = LL_TIM_GetCounter(TIM2); // position of encoder in bits
	P = PI2*(float)LL_TIM_GetCounter(TIM2)/rangeEnc; // position of encoder in physics units (radians)
	if (count==10UL)
	{
		count=0UL;
		direct	= (uint32_t)READ_BIT(TIM2->CR1, TIM_CR1_DIR); // detect of direction
		if (direct==0UL) // solve velocity taking in account direction of rotate and transition over zerrow
		{
			if (P>=Pdel) Vfb = (P-Pdel)*F2;
			else Vfb = (P+PI2-Pdel)*F2; // transition over zerrow
		} 
		else
		{
			if (P<=Pdel) Vfb = (P-Pdel)*F2;
			else Vfb = (-Pdel-PI2+P)*F2; // transition over zerrow
		}
		Pdel = P;
		V_data.Err = Vref - Vfb;
		Iref = -Control_PI(&V_data);
	}
	else count++;	
	// get currents of phases in physics units (Ampers)
	Ifb[0]=-((dataADC1[0])*invKfb - Ibias);
	Ifb[1]=-((dataADC1[1])*invKfb - Ibias);
	Ifb[2]=-((dataADC1[2])*invKfb - Ibias);
	
	// solve of general current of stator where
	// SIN_BIT[BIT] - array where index is position of encoder in bits and value is sin(index)
	// (P_BIT*zp)%PI2_BIT - angle between phase A and axix of general current
	Ifb0=0.6667f*(SIN_BIT[(P_BIT*zp)%PI2_BIT]*Ifb[0] + SIN_BIT[(P_BIT*zp+PI23_BIT)%PI2_BIT]*Ifb[1] + SIN_BIT[(P_BIT*zp+PI43_BIT)%PI2_BIT]*Ifb[2]);
	
	I_data.Err = Iref - Ifb0;
	U0 = Control_PI(&I_data);	// control signal for PWM
	if (flag_work) 
	{
		static float mod3harm=0;
		mod3harm =0.1667f*SIN_BIT[(P_BIT*zp*3)%PI2_BIT]; // part of modulation of 3rd harmonic
		PWM[0]=(uint16_t)((1.155f*U0*(SIN_BIT[(P_BIT*zp)%PI2_BIT]+mod3harm) + 1.0f)*0.5f * rangePWM);
		PWM[1]=(uint16_t)((1.155f*U0*(SIN_BIT[(P_BIT*zp+PI23_BIT)%PI2_BIT]+mod3harm) + 1.0f)*0.5f * rangePWM);
		PWM[2]=(uint16_t)((1.155f*U0*(SIN_BIT[(P_BIT*zp+PI43_BIT)%PI2_BIT]+mod3harm) + 1.0f)*0.5f * rangePWM);
		// where 1/6 = 0.1667, 2/sqrt(3) = 1.155
		updataPWM(PWM);
	}
	Ntick = LL_TIM_GetCounter(TIM4)>tick ? LL_TIM_GetCounter(TIM4)-tick : LL_TIM_GetAutoReload(TIM4)-tick+LL_TIM_GetCounter(TIM4);
	LL_DMA_ClearFlag_TC1(DMA1);
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	Uin = (float)dataADC1[3]/64.79f;
	Rntc = 19246.5f/((float)dataADC1[4]) - 4.7f;
	Umcu = __LL_ADC_CALC_VREFANALOG_VOLTAGE( dataADC1[6], LL_ADC_RESOLUTION_12B );
	Tmcu = __LL_ADC_CALC_TEMPERATURE( Umcu, dataADC1[5], LL_ADC_RESOLUTION_12B );
	Vref = ((float)dataADC1[7]-2048.0f)/20.41f;
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, (uint32_t)(Ifb0*1240.66f + 1861L)); // out: 1.5V + [-1.5; 1.5] for Ifb0 [ -1.5 ; 1.5] A
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uint32_t)(Vfb*12.4f + 1861L)); // out: 1.5V + [-1.5; 1.5]	for Vfb [-150; 150] rad/s
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	while (LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1 )
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */
	uint8_t CMD_UART=0; 
  CMD_UART=LL_LPUART_ReceiveData8(LPUART1);
	if(CMD_UART=='0') LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
	else LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
  /* USER CODE END LPUART1_IRQn 0 */
  /* USER CODE BEGIN LPUART1_IRQn 1 */
	LL_LPUART_TransmitData8(LPUART1, CMD_UART);
  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
