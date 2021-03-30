/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

uint16_t tim = 0;
uint16_t R[161] = {1912, 1952, 1998, 2052, 2113, 2180, 2255, 2335, 2419, 2506, 2593, 2677, 2754, 2821, 2875, 2913, 2936, 2942, 2933, 2912, 2881, 2843, 2801, 2757, 2712, 2669, 2628, 2591, 2558, 2530, 2508, 2490, 2479, 2473, 2473, 2478, 2489, 2506, 2528, 2556, 2588, 2625, 2665, 2708, 2753, 2798, 2840, 2879, 2910, 2932, 2942, 2937, 2916, 2878, 2826, 2760, 2683, 2600, 2513, 2426, 2341, 2261, 2186, 2118, 2056, 2002, 1955, 1915, 1882, 1856, 1835, 1821, 1814, 1812, 1816, 1826, 1842, 1865, 1894, 1930, 1972, 2022, 2079, 2143, 2214, 2291, 2373, 2459, 2546, 2632, 2713, 2786, 2848, 2895, 2926, 2940, 2940, 2925, 2899, 2865, 2824, 2781, 2736, 2692, 2650, 2610, 2575, 2545, 2519, 2499, 2484, 2475, 2472, 2474, 2483, 2496, 2516, 2540, 2570, 2605, 2643, 2685, 2729, 2774, 2818, 2859, 2894, 2922, 2938, 2941, 2929, 2901, 2856, 2797, 2726, 2646, 2560, 2473, 2387, 2304, 2226, 2154, 2089, 2031, 1980, 1936, 1899, 1869, 1845, 1828, 1817, 1812, 1813, 1820, 1833, 1852, 1877, 1910, 1948, 1994, 2047};
uint16_t L[161] = {1577, 1652, 1742, 1847, 1968, 2107, 2264, 2439, 2632, 2840, 3061, 3291, 3525, 3755, 3977, 4182, 4367, 4527, 4663, 4774, 4862, 4929, 4979, 5015, 5040, 5056, 5065, 5069, 5071, 5071, 5069, 5068, 5067, 5066, 5066, 5066, 5068, 5069, 5071, 5071, 5070, 5065, 5056, 5041, 5018, 4983, 4934, 4868, 4782, 4673, 4539, 4380, 4197, 3994, 3774, 3543, 3310, 3079, 2857, 2648, 2454, 2278, 2119, 1979, 1856, 1750, 1659, 1583, 1520, 1471, 1433, 1407, 1393, 1390, 1397, 1416, 1446, 1488, 1542, 1610, 1692, 1788, 1901, 2030, 2177, 2343, 2526, 2726, 2940, 3166, 3399, 3632, 3859, 4073, 4270, 4444, 4593, 4717, 4817, 4895, 4954, 4997, 5028, 5048, 5061, 5067, 5071, 5071, 5070, 5069, 5067, 5066, 5066, 5066, 5067, 5068, 5070, 5071, 5071, 5068, 5062, 5050, 5032, 5003, 4962, 4906, 4831, 4734, 4614, 4469, 4299, 4106, 3894, 3669, 3436, 3203, 2976, 2759, 2557, 2371, 2202, 2052, 1920, 1805, 1706, 1622, 1552, 1496, 1452, 1420, 1399, 1390, 1392, 1404, 1428, 1464, 1511, 1572, 1646, 1734, 1838};
uint16_t step = 0;
float aL = 0.1234;
float aP = 0.14;
float vL = 0;
float vP = 0;
uint32_t EN_L = 0;
uint32_t EN_P = 0;
uint32_t ERR_L = 0;
uint32_t ERR_P = 0;
uint32_t ERR_L_SUM = 0;
uint32_t ERR_P_SUM = 0;

float P = 0.5;
float I = 0.00001;


float x = 0;
float y = 1562.04993518;
float om = 0;
float cm = 0.04908738521;
uint32_t EN_L_ = 0;
uint32_t EN_P_ = 0;

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
extern TIM_HandleTypeDef htim16;
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
  while (1)
  {
  }
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  tim++;
  if(tim == 1000) // start z opóźnieniem
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	  TIM2 -> CCR1 = 0; // prawy
	  TIM2 -> CCR2 = 0; // lewy
  }
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  if(tim > 1000)
  {
	  if(tim%120 == 0)
	  {
		  if(step < 161)
		  {
			  vL = L[step]*aL;
			  vP = R[step]*aP;
			  step++;
		  }
		  else
		  {
			  vL = 0;
			  vP = 0;
		  }
	  }
	  if(tim%20 == 0)
	  {
		  if(EN_L > TIM3->CNT) // korekcja przejścia przez zero
			  EN_L -= 65536;
		  if(EN_P > TIM4->CNT)
			  EN_P -= 65536;

		  ERR_L = ((uint32_t)TIM3->CNT-EN_L)*50;
		  ERR_P = ((uint32_t)TIM4->CNT-EN_P)*50;
		  ERR_L_SUM += ERR_L;
		  ERR_P_SUM += ERR_P;
		  float PI_L = fmin(fmax(P*ERR_L+I*ERR_L_SUM, -vL*0.1f), vL*0.1f);
		  float PI_P = fmin(fmax(P*ERR_P+I*ERR_P_SUM, -vP*0.1f), vP*0.1f);

		  TIM2 -> CCR1 = (uint16_t)(vP+PI_P);
		  TIM2 -> CCR2 = (uint16_t)(vL+PI_L);

		  EN_L = TIM3->CNT;
		  EN_P = TIM4->CNT;
	  }
  }
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
