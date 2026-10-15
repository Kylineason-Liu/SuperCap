#include "tim.h"
#include "time_task.h"
#include "power_calculation.h"
#include "LED.h"
#include "hrtim.h"
#include "CAN_driver.h"

uint16_t power_supply_off = 0;
uint8_t time10ms = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		control_calc();
	}
	
	if(htim == &htim4)
	{
		Debug_task();
	}
	
	if(htim == &htim6)
	{
		if(LED_state == LED_OFF)
			LED_on();
		else
			LED_off();
	}
}

void Debug_task()
{
#if DCDC_OUTPUT == 3 && ADC_CALIBRATION == 1
	if (in.I_now < 0.11f && in.U_now > 5.0f  && cap.U_now < CAPV_MAX)
		power_supply_off++;
	else	
		power_supply_off = 0;
	
	if(power_supply_off > 1000)
		HAL_NVIC_SystemReset();
	else if(power_supply_off > 300)
	{
		LED_SetColor(YELLOW_COLOR);
		DCDC_deinit();
	}
#endif
	if(time10ms == 9)
	{
		time10ms = 0;
		power_send();
	}
	else
		time10ms ++;
}
