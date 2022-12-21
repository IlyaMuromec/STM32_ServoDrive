/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const float PI=3.14159265f;
const float PI23=2.0943952f;
const float PI43=4.1887902f;
const float PI2=6.2832f;

const uint32_t PI_BIT=1000UL;
const uint32_t PI23_BIT=667;
const uint32_t PI43_BIT=1333UL;
const uint32_t PI2_BIT=2000UL;

struct Var_data I_data={0};				
struct Var_data V_data={0};

struct PID_param I_PID_param={0};
struct PID_param V_PID_param={0};

uint32_t volatile dataADC1[8]={0};
// dataADC[0]=IA;
// dataADC[1]=IB;
// dataADC[2]=IC;
// dataADC[3]=Uin  if Uin=10V then dataADC[3]=10*9.31/(178.31)/3.3*4095=647;
// dataADC[4]=Tboard if Tboard=0 then dataADC[4]=3.3*4.7/(4.7+10)/3.3*4095=1309;
// dataADC[5]~Tmc;
// dataADC[6]~Umc;
// max data = 2^12 - 1 = 4095;

float volatile Uin=0;
uint32_t volatile Umcu=0;
float volatile Tboard=0;
float volatile Rntc=4.7f; // kOmh
int32_t volatile Tmcu=0;

float volatile Ifb[3]={0}; // phase current
float volatile Ifb0=0; // general currnet
float volatile Ifb0_tmp=0; // general currnet
float volatile Iref=0; // reference current

float const Kfb=625.3065f; // Kfb=Rshunt*Kopa*KADC=0.33*1.527*(2^12-1)/3.3
float const invKfb=0.0016f; // 1/Kfb
float const Ibias=3.0918f; //Ibias=Ubias/Rshunt/Kop

float volatile Vfb=0; // speed of rotor
float volatile Vref=50; // reference speed of rotor
float volatile P=0; // position of rotor
uint32_t volatile P_BIT=0; // position of rotor
float volatile SIN_BIT[4000]={0}; // sin(position of rotor)

uint32_t volatile flag_work=0L;
uint32_t volatile PWM[3]={0};
float volatile rangeEnc=0;
float volatile rangePWM=0;
float volatile U0=0; // output of current regulator
float const T1 = 0.00005f; // sample time for current loop
float const T2 = 0.0005f; // sample time for speed loop
float const F1 = 20000.f; //
float const F2 = 2000.f; // 
uint32_t const zp=4; // pare pole 
uint16_t volatile tick=0; 
uint16_t volatile Ntick=0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	USER_LPUART1_UART_Init();
	USER_DMA_Init();
	USER_ADCx_Init(ADC1);
	USER_TIM1_PWM_Init();
	USER_TIM2_ENCODER_Init();
	USER_TIM3_Init();
	USER_TIM4_Init();
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// prepare regulators 
	rangePWM = LL_TIM_GetAutoReload(TIM1);
	rangeEnc = LL_TIM_GetAutoReload(TIM2);
	
	for(int i=0; i<4000; i++)
	{
		SIN_BIT[i]=sin(i*PI2/rangeEnc);
	}
	I_PID_param.Ki = 0.0187f; 
	I_PID_param.Kp = 0.5625f;
	I_PID_param.limit = 0.95f;
	I_data.PID = &I_PID_param;

	V_PID_param.Ki = 0.00026f;
	V_PID_param.Kp = 0.0103f;
	V_PID_param.limit = 5.0f;
	V_data.PID = &V_PID_param;
	
	startPWM(); // enable channal of PWM of MCU
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12); // enable channal of PWM of L6230	
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11); // enable L6230
	
	calibEncoder(PWM);
	
	flag_work=1; // enable regulators
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1µs transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  LL_Init1msTick(160000000);

  LL_SetSystemCoreClock(160000000);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
}

/* USER CODE BEGIN 4 */
/* --------------- PI Regulators ---------------*/
float Control_PI( struct Var_data *Data)
{
	float contr_out=0;
	
	Data->Integ = Data->Err * Data->PID->Ki + Data->Integ;
	
	if (Data->Integ  >  Data->PID->limit){
		Data->Integ = Data->PID->limit;}
	else if (Data->Integ  <  -Data->PID->limit){
		Data->Integ =  -Data->PID->limit;}
	else{
		Data->Integ = Data->Integ;}
	
	contr_out = Data->Integ + Data->PID->Kp * Data->Err;
	
	if (contr_out  >  Data->PID->limit){
		contr_out = Data->PID->limit;}
	else if (contr_out  <  -Data->PID->limit){
		contr_out = -Data->PID->limit;}
	else{
		contr_out = contr_out;}

	return contr_out;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
