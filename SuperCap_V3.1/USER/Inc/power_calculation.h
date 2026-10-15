#ifndef _POWER_CALCULATION_H
#define _POWER_CALCULATION_H

#include "stdint.h"

/*功能相关*/
#define ADC_CALIBRATION		1 			//0:关闭校准		1:开启校准
#define LOW_PASS_FILTTER	0	  		//0:关闭低通滤波	1:开启低通滤波
#define DCDC_OUTPUT			3			//0:DCDC关闭 		1:DCDC恒定升降压输出	2:DCDC PID闭环稳压限流输出	3:DCDC PID闭环充放电(无校准时禁用)
#define FIXED_RATIO			0.8f	//恒定升降压输出升降压比例, 28V: 1.09375, 5V: 0.191,

#define BOARD_NUM 5		   	//板子编号，区别不同校准参数 

/*PWM相关*/
#define shift_duty 0.98f
#define shift_duty_2 2 * shift_duty
#define MAX_PWM_CMP (uint32_t)(shift_duty * HRTIM1->sMasterRegs.MPER)       //PWM最大比较值
#define MIN_PWM_CMP (uint32_t)((1 - shift_duty) * HRTIM1->sMasterRegs.MPER) //PWM最小比较值
#define DP_PWM_PER HRTIM1->sMasterRegs.MPER                                 //定时器HRTIM周期寄存器
#define volt_ratio_max (shift_duty) / (1 - shift_duty)
#define volt_ratio_min (1 - shift_duty) / (shift_duty)r


/*ADC相关*/
#define adc_maxvalu 4095													//12位adc，硬件16x过采样右移4位,结果为12位
#define adc_vref 3.2958f													//adc参考电压
#define adc_conv_fact adc_vref / adc_maxvalu			//adc结果转换系数
#define V_GAIN 20.0f															//电压增益
#define I_GAIN 1.0f / 100.0f											//电流增益倒数
#define R_SAMP_inv 2000.0f 												//采样电阻倒数 1/0.5mΩ
#define I_BIAS 775															  //无外接时的IAR240偏置电压,理论值为0.625V

/*硬件限制相关*/
#define INC_MAX 30.0f
#define OUTC_MAX 30.0f

#define INV_MAX 28.0f
#define INV_MIN 14.0f
#define max_volt_ratio 1.5f		//最大升压比
#define CAPV_MAX 20.0f       //理论最大耐压32.45V 官方模块最大耐压30V
#define CAPV_MIN 5.0f       
#define DCDCOUTV_MAX 40.0f
#define max_capc_nega -20.0f//放电最大电流
#define max_capc_posi 9.0f//充电最大电流

#define cap_resistance 160.0f/1000.0f//电容内阻毫欧

typedef struct
{
	uint8_t elec_type;      //0电压 1电流
	uint16_t valu; 			//ADC原始数据
	double real_valu1;      //真实值1 外部仪表获得
	double get_volt1;       //测量值1  单片机滤波后获得
	double real_valu2;      //真实值2
	double get_volt2;       //测量值2
	float offset;           //漂移
	float ratio;            //比例系数
	float result;			//解算后的真实数据(电压单位V 电流单位A)
} ELEC_INFO_STRUCT;


typedef struct
{
	float U_now;
	float I_now;
	float P_now;
} CONTROL_STRUCT;

volatile typedef struct
{
	uint8_t loop_mode;      //环路模式  0：cc(ConstantCurrency充电)  1：cv(ConstantVoltage充电)  2：cc(ConstantCurrency放电)
	uint16_t max_power_lim; //裁判系统最大功率
	float ratio;
	float efficiency;       //总效率,可用于估算发热功率
} BUCKBOOST_STRUCT;

extern ELEC_INFO_STRUCT cap_c, cap_v, in_c, in_v, out_c;
extern CONTROL_STRUCT in, out, cap, dcdc_in;
extern BUCKBOOST_STRUCT buckboost;
extern float ratio_display;
extern float cloop_display;
extern float vloop_display;

void DCDC_init(void);
void DCDC_deinit(void);
void buckboost_pwm_update(float volt_ratio);
float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p);
void control_calc(void);
void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2);

#endif
