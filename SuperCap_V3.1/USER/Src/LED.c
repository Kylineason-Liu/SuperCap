#include "tim.h"
#include "stdint.h"
#include "LED.h"

uint8_t LED_state;

/**
 * @description: LED初始化，使能PWM时钟
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void LED_init()
 {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 	//Blue
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 	//Red
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 	//Green
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR1 = 0;
	LED_state = LED_OFF;
 }
 
 /**
  * @description: 设定LED颜色
  * @param {unused} 
  * @return: void
  * @note: 
  */ 
void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue)
 {
	TIM3->CCR2 = red;
	TIM3->CCR3 = green;
	TIM3->CCR1 = blue;
 }
 
/**
 * @description: 开启LED
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
 void LED_on()
 {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 	//Blue
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 	//Red
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 	//Green
	LED_state = LED_ON;
 }
 
 /**
 * @description: 关闭LED
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
 void LED_off()
 {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); 	//Blue
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); 	//Red
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); 	//Green
	LED_state = LED_OFF;
 }
 
 /**
  * @description: LED以固定频率闪烁
  * @param frequency:闪烁频率，0为不闪烁 
  * @return: void
  * @note: 
  */ 
 void LED_blink(uint8_t frequency)
 {
	//TIM6 PSC为17000-1，计数频率为10k
	 if(frequency == 0)
		 HAL_TIM_Base_Stop_IT(&htim6);
	 else
	 {
		TIM6->ARR = 10000/2/frequency - 1;
		HAL_TIM_Base_Start_IT(&htim6);
	 }
 }
