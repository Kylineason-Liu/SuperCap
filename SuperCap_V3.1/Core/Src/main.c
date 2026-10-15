/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "power_calculation.h"
#include "OLED.h"
#include "LED.h"
#include "CAN_driver.h"
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
uint32_t Debug[10];
float DealtT[10];
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();
  MX_FDCAN2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP5_Init();
  MX_OPAMP6_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//LED初始化&亮红灯表示初始化化
	LED_init();
	LED_SetColor(RED_COLOR);
	
	//初始化DWT计时
	DWT_Init(170);
	
	//开启ADC校准
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc5,ADC_SINGLE_ENDED);
	
	//内部运放初始化
	HAL_OPAMP_Start(&hopamp3);
	HAL_OPAMP_Start(&hopamp5);
	HAL_OPAMP_Start(&hopamp6);
	
	//使能ADC&DMA传输
	HAL_Delay(5);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&in_v.valu, 1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&cap_v.valu, 1);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&out_c.valu, 1);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t *)&in_c.valu, 1);
	HAL_ADC_Start_DMA(&hadc5, (uint32_t *)&cap_c.valu, 1);
	
	//OLED初始化
	//To do: OLED检测优化与热插拔支持
	OLED_Init();

	//FDCAN初始化
	FDCAN_FilterInit();
	FDCAN_start();
	
	//功率控制初始化
	DCDC_init();
	
	//使能PWM控制信号输出&控制终端定时器&错误检测定时器,时钟同步为TIM1->HRTIM->TIM4
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
		OLED_printf(0, 0, "I1N");
		OLED_printf(20, 0, "%4.2f;%4.2f;%3.1f", in.U_now, in.I_now, in.P_now);
		OLED_printf(0, 12, "OUT");
		OLED_printf(20, 12, "%4.2f;%4.2f;%3.1f", out.U_now, out.I_now, out.P_now);
		OLED_printf(0, 24, "I2N");
		OLED_printf(20, 24, "%4.2f;%4.2f;%3.1f", dcdc_in.U_now, dcdc_in.I_now, dcdc_in.P_now);
		OLED_printf(0, 36, "CAP");
		OLED_printf(20, 36, "%4.2f;%4.2f;%3.1f", cap.U_now, cap.I_now, cap.P_now);

		OLED_printf(0, 48, "MODE");
		OLED_printf(26, 48, "%d", buckboost.loop_mode);

		#if DCDC_OUTPUT == 1
		OLED_printf(40, 48, "%3.2f", ratio_display);
		#elif DCDC_OUTPUT == 2
		OLED_printf(40, 48, "%3.2f;%3.2f", cloop_display,vloop_display);
		#elif DCDC_OUTPUT == 3
		OLED_printf(40, 48, "%3.2f", ratio_display);		
		OLED_printf(80, 48, "%d", buckboost.max_power_lim);
		#endif
		OLED_Refresh_Gram(); //更新显示,cpuz占用极高必须优化降频

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
