#include "math.h"
#include "hrtim.h"
#include "LED.h"
#include "power_calculation.h"
#include "PID.h"

//ADC直接采样结果，电容口向电容充电为负，电容向外放电为正，量程为-10A - 30A
//为了让电压环和电流环功率控制逻辑相同，设定电容电流初始系数为负数,使电容口向电容充电为正，电容向外放电为负，量程变为-30A - 10A
//校准时由于是在原有系数上相乘，因此可以按照电容充电为正，放电为负填入数据

ELEC_INFO_STRUCT cap_c;                      //电容电压参数
ELEC_INFO_STRUCT cap_v;                      //电容电流参数
ELEC_INFO_STRUCT in_c;                      //输入电流参数
ELEC_INFO_STRUCT in_v;                      //输入电压参数
ELEC_INFO_STRUCT out_c;                     //底盘电流参数
CONTROL_STRUCT in, out, cap, dcdc_in; 	//总输入，底盘输出，dcdc输入，dcdc输出
BUCKBOOST_STRUCT buckboost;
float ratio_display;
float vloop_display;
float cloop_display;
uint8_t init_flag = 0;

void DCDC_init(void)
{
	in_v.elec_type = 0;
	in_v.ratio = adc_conv_fact * V_GAIN;
	cap_v.elec_type = 0;
	cap_v.ratio = adc_conv_fact * V_GAIN;
	//电容电流取负号
	cap_c.elec_type = 1;
	cap_c.ratio = -adc_conv_fact * I_GAIN * R_SAMP_inv;
	in_c.elec_type = 1;
	in_c.ratio = adc_conv_fact * I_GAIN * R_SAMP_inv;
	out_c.elec_type = 1;
	out_c.ratio = adc_conv_fact * I_GAIN * R_SAMP_inv;

	buckboost.max_power_lim = 120.0f;
	init_flag = 0;
	
	if(DCDC_OUTPUT != 0)
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);  
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER);

	
#if ADC_CALIBRATION == 1
	#if BOARD_NUM == 1
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
	  ADC_Linear_calibration_init(&in_v,22.84,22.56,25.84,25.60);	//ok r1: 22.84, g1: 22.56, r2: 25.84, g2: 25.60
	  ADC_Linear_calibration_init(&cap_v,5.0003,5.19,28.21,27.95);	//ok r1: 4.92, g1: 5.14, r2: 28.21, g2: 27.95
	  ADC_Linear_calibration_init(&in_c,0.50,0.5495,2.80,2.8521);		//ok r1: 0.5, g1: 0.55, r2: 2.8, g2: 2.85
	  ADC_Linear_calibration_init(&out_c,0.50,0.558,2.80,2.85);	//ok rl: 0.5, g1: 0.56, r2: 2.8, g2: 2.85
	  ADC_Linear_calibration_init(&cap_c,0.50,0.58,2.80,2.8815);	//ok r1: 0.5, g1: 0.58, r2: 2.8, g2: 2.88
	
	#endif
	#if BOARD_NUM == 2
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.92,23.02,25.44,25.49);	//ok
		ADC_Linear_calibration_init(&cap_v,4.85,4.83,27.97,27.92);	//ok
		ADC_Linear_calibration_init(&in_c,0.500,0.557,2.800,2.840);		//ok
		ADC_Linear_calibration_init(&out_c,0.500,0.537,2.800,2.867);	//ok
		ADC_Linear_calibration_init(&cap_c,0.500,0.545,2.800,2.854);	//ok

#endif
	#if BOARD_NUM == 3
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.22,22.23,26.01,25.95);	//ok
		ADC_Linear_calibration_init(&cap_v,5.01,4.87,28.59,28.33);	//ok
		ADC_Linear_calibration_init(&in_c,0.500,0.554,2.800,2.822);		//ok
		ADC_Linear_calibration_init(&out_c,0.500,0.538,2.800,2.827);	//ok
		ADC_Linear_calibration_init(&cap_c,0.500,0.502,2.800,2.836);	//ok
	#endif
	#if BOARD_NUM == 4
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.69,22.58,25.59,25.50);	//ok
		ADC_Linear_calibration_init(&cap_v,4.900,4.45,28.02,27.84);	//ok
		ADC_Linear_calibration_init(&in_c,0.50,0.53,2.805,2.82);		//ok
		ADC_Linear_calibration_init(&out_c,0.50,0.535,2.802,2.785);	//ok
		ADC_Linear_calibration_init(&cap_c,0.50,0.594,2.80,2.923);	//ok
	#endif
	#if BOARD_NUM == 5
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,24.96,24.71,22.69,22.39);		//ok
		ADC_Linear_calibration_init(&cap_v,4.764,4.58,27.28,27.12);		//ok
		ADC_Linear_calibration_init(&in_c,0.50,0.535,2.80,2.85);		//ok
		ADC_Linear_calibration_init(&out_c,0.50,0.528,2.80,2.84);	//ok
		ADC_Linear_calibration_init(&cap_c,0.503,0.582,2.8,2.905);	//ok
	#endif
	#if BOARD_NUM == 6
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
   		ADC_Linear_calibration_init(&in_v,22.743,22.71,25.584,25.53);	//ok
		ADC_Linear_calibration_init(&cap_v,4.5341,4.50,27.196,26.87);	//ok
		ADC_Linear_calibration_init(&in_c,0.5975,0.55,2.8870,2.83);		//ok
		ADC_Linear_calibration_init(&out_c,0.4977,0.47,2.7875,2.77);	//ok
		ADC_Linear_calibration_init(&cap_c,0.4964,0.64,2.7817,2.95);	//ok
	#endif
	#if BOARD_NUM == 7
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.937,22.84,25.440,25.37);	//ok
		ADC_Linear_calibration_init(&cap_v,4.5893,4.56,27.518,27.29);	//ok
		ADC_Linear_calibration_init(&in_c,0.5478,0.53,2.8863,2.85);		//ok
		ADC_Linear_calibration_init(&out_c,0.4486,0.42,2.7872,2.77);	//ok
		ADC_Linear_calibration_init(&cap_c,0.4434,0.67,2.7808,3.05);	//ok
	#endif
	#if BOARD_NUM == 8
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.476,22.49,24.838,24.89);	//ok
		ADC_Linear_calibration_init(&cap_v,4.9653,5.28,27.312,27.32);	//ok
		ADC_Linear_calibration_init(&in_c,0.5541,0.54,2.8945,2.86);		//ok
		ADC_Linear_calibration_init(&out_c,0.4497,0.46,2.7907,2.80);	//ok
		ADC_Linear_calibration_init(&cap_c,0.4470,0.46,2.7895,2.85);	//ok
	#endif
	#if BOARD_NUM == 9
		//ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2)
		ADC_Linear_calibration_init(&in_v,22.743,22.71,25.584,25.53);	//
		ADC_Linear_calibration_init(&cap_v,4.5341,4.50,27.196,26.87);	//
		ADC_Linear_calibration_init(&in_c,0.5975,0.55,2.8870,2.83);		//
		ADC_Linear_calibration_init(&out_c,0.4977,0.47,2.7875,2.77);	//
		ADC_Linear_calibration_init(&cap_c,0.4964,0.64,2.7817,2.95);	//
	#endif
#endif

  PID_init(&curr_loop, PID_DELTA, 0.15f, 0.015f, 0, 30, 25, 0, -25); 		//电流环
  PID_init(&volt_high_loop, PID_DELTA, 0.1f, 0.006f, 0, 30, 25, 0, -25);	//高电压环 //1.0f 0.06f 4.0f 
  PID_init(&volt_low_loop, PID_DELTA, 0.1f, 0.006f, 0, 7, 25, 3, -25);	//低电压环 //1.0f 0.06f 4.0f 
	
#if LPF_ENABLE == 1
	low_filter_init(&lpf[0], 10e3, 1e2);		//inv
	low_filter_init(&lpf[1], 10e3, 1e3);		//inc
 	low_filter_init(&lpf[2], 10e3, 1e3);		//outc
	low_filter_init(&lpf[3], 10e3, 1e3);		//capv
	low_filter_init(&lpf[4], 10e3, 1e3);		//capc
#endif
	
}

void DCDC_deinit(void)
{
	buckboost.max_power_lim = 120.0f;
	init_flag = 0;
	HRTIM1->sTimerxRegs[0].CMP1xR = HRTIM1->sMasterRegs.MPER / 2u;
	HRTIM1->sTimerxRegs[0].CMP3xR = HRTIM1->sMasterRegs.MPER / 2u;
	HRTIM1->sTimerxRegs[1].CMP1xR = HRTIM1->sMasterRegs.MPER / 2u; 
	HRTIM1->sTimerxRegs[1].CMP3xR = HRTIM1->sMasterRegs.MPER / 2u;	
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); //关闭输出
}

void control_calc(void)
{
	static float inv, inc, outc, capv, capc;
	static float ratio_pass = 0, vloop = 0 , cloop = 0 ,charge_curr = 0,charge_power = 0;

	inv = ADCvalue_to_ELEC(&in_v);
	inc = ADCvalue_to_ELEC(&in_c);
	outc = ADCvalue_to_ELEC(&out_c);
	capv = ADCvalue_to_ELEC(&cap_v);
	capc = ADCvalue_to_ELEC(&cap_c);

#if LPF_ENABLE == 1
	lpf[0].Input = inv;
	lpf[1].Input = inc;
	lpf[2].Input = outc;
	lpf[3].Input = capv;
	lpf[4].Input = capc;
	for (char i = 0; i < 5; i++)
	{
	low_filter_calc(&lpf[i]);
	}
	inv = lpf[0].Output;
	inc = lpf[1].Output;
	outc = lpf[2].Output;
	capv = lpf[3].Output;
	capc = lpf[4].Output;
#endif
	
	in.U_now = inv;
	in.I_now = inc;
	in.P_now = in.U_now * in.I_now;

	out.U_now = inv;
	out.I_now = outc;
	out.P_now = out.U_now * out.I_now;

	dcdc_in.U_now = inv;
	dcdc_in.I_now = inc - outc;		//流入DCDC为正
	dcdc_in.P_now = dcdc_in.U_now * dcdc_in.I_now;

	cap.U_now = capv;
	cap.I_now = capc;
	cap.P_now = cap.U_now * cap.I_now;	
	
	if(in.U_now > 20 && buckboost.max_power_lim > 0 && buckboost.max_power_lim < 250)		//运算与更新条件
	{
		buckboost.efficiency = (cap.P_now) / dcdc_in.P_now * 100;

		charge_power = buckboost.max_power_lim - out.P_now;
		charge_power = charge_power>2  ? (charge_power-2) : charge_power;				    //降低充电功率以补充缓冲能量
		charge_power = charge_power < (-400)  ? -400 : charge_power;
		//电容端电流限制
		charge_curr = charge_power / in.U_now;
		charge_curr = fminf(charge_curr, max_capc_posi);								//充电限流9A
		
	#if DCDC_OUTPUT == 1
		buckboost_pwm_update(FIXED_RATIO);
		ratio_display = FIXED_RATIO;
	#elif DCDC_OUTPUT == 2
		cloop = PID_calc(&curr_loop, cap.I_now, 8) / in.U_now;
		vloop = PID_calc(&volt_high_loop, cap.U_now, 27.5) / in.U_now;
		if (vloop <= cloop)		//电压环
		{
			ratio_pass = vloop;
			buckboost.loop_mode = 0;			//环路模式  0：cv(ConstantVoltage)  1：cc(ConstantCurrency)  2：cp(ConstantPower)
		}
		else									//电流环
		{
			ratio_pass = cloop;
			buckboost.loop_mode = 1;
		}
		buckboost_pwm_update(ratio_pass);
		cloop_display = cloop;
		vloop_display = vloop;
		ratio_display = ratio_pass;
	#elif DCDC_OUTPUT == 3 && ADC_CALIBRATION == 1
		if (init_flag == 0)
		{
			//初始化时，使用电压采样值计算初始升降压比
			curr_loop.out = cap.U_now;
			volt_high_loop.out = cap.U_now;
			volt_low_loop.out = CAPV_MIN;
			ratio_pass = cap.U_now / in.U_now;
			init_flag = 1;		
			HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);			
		}
		else if (charge_power > 0)			//电容充电
		{
			if(buckboost.loop_mode == 2)
			{
				curr_loop.out = ratio_pass * in.U_now;
				volt_high_loop.out = ratio_pass * in.U_now;
			}
			cloop = PID_calc(&curr_loop, dcdc_in.I_now, charge_curr) / in.U_now;
			vloop = PID_calc(&volt_high_loop, cap.U_now, CAPV_MAX) / in.U_now;

			if (vloop <= cloop)		//电压环
			{
				ratio_pass = vloop;
				buckboost.loop_mode = 0;			//环路模式  0：cv(ConstantVoltage)  1：cc(ConstantCurrency)  2：cp(ConstantPower)
			}
			else									//电流环
			{
				ratio_pass = cloop;
				buckboost.loop_mode = 1;
			}
			LED_SetColor(GREEN_COLOR);
		}
		else if (charge_power <= 0)		//电容放电
		{
			if(buckboost.loop_mode != 2)
			{
				curr_loop.out = ratio_pass * in.U_now;
				volt_low_loop.out = ratio_pass * in.U_now;
			}
			cloop = PID_calc(&curr_loop, dcdc_in.I_now, charge_curr) / in.U_now;
			vloop = PID_calc(&volt_low_loop, cap.U_now, CAPV_MIN) / in.U_now;	
			
			if (vloop <= cloop)		//电压环
			{
				ratio_pass = cloop;
			}
			else									//电流环
			{
				ratio_pass = vloop;
			}

			buckboost.loop_mode = 2;
			LED_SetColor(BLUE_COLOR);
		}
		buckboost_pwm_update(ratio_pass);
		ratio_display = ratio_pass;
	#endif
	}
}

void buckboost_pwm_update(float volt_ratio)
{
	volatile uint32_t CompareValue;
	volatile uint32_t buck_duty = 0;
	volatile uint32_t boost_duty = 0;
	volt_ratio = fminf(max_volt_ratio, volt_ratio);
	/* 
	* 升降压的PWM方案：
	* 1.由于MOS采用自举电容驱动方式，故上管不能100%占空比导通，这里我们设为99%。（需要下管导通一小段时间给自举二极管充电）
	* 2.本函数的输入参数为总的占空比(??电压比吧，<1降压，>1升压)，需要分别算出两个半桥的占空比。
	* 3.降压工作时，BOOST的半桥的占空比是固定的为99%，通过改变BUCK半桥占空比实现稳压。
	* 3.升压或等压工作时，BUCK半桥的占空比是固定的为99%，通过改变BOOST半桥占空比实现稳压。
	*/
	if (volt_ratio >= 1.0f)
	CompareValue = DP_PWM_PER * (shift_duty_2 - shift_duty / volt_ratio);
	else
	CompareValue = shift_duty * volt_ratio * DP_PWM_PER;

	if (CompareValue > shift_duty * DP_PWM_PER)   //volt_ratio > 1
	boost_duty = shift_duty_2 * DP_PWM_PER - CompareValue; //BOOST半桥占空比，Dboost = D总 - Dbuck  （boost_duty = shift_duty / volt_ratio）
	else
	boost_duty = shift_duty * DP_PWM_PER; //BUCK模式,boost_duty给固定占空比

	if (CompareValue > shift_duty * DP_PWM_PER)
	buck_duty = shift_duty * DP_PWM_PER; //BOOST模式,buck_duty给固定占空比
	else
	buck_duty = CompareValue; //BUCK半桥占空比

	/*
	*  占空比限制
	*/
	if (boost_duty > MAX_PWM_CMP)
	boost_duty = MAX_PWM_CMP;
	if (boost_duty < MIN_PWM_CMP)
	boost_duty = MIN_PWM_CMP;
	if (buck_duty > MAX_PWM_CMP)
	buck_duty = MAX_PWM_CMP;
	if (buck_duty < MIN_PWM_CMP)
	buck_duty = MIN_PWM_CMP;
	/*
	*  更新比较寄存器
	*/
	HRTIM1->sTimerxRegs[0].CMP1xR = (HRTIM1->sMasterRegs.MPER - buck_duty) / 2u;
	HRTIM1->sTimerxRegs[0].CMP3xR = (HRTIM1->sMasterRegs.MPER + buck_duty) / 2u;
	HRTIM1->sTimerxRegs[1].CMP1xR = (HRTIM1->sMasterRegs.MPER - boost_duty) / 2u; 
	HRTIM1->sTimerxRegs[1].CMP3xR = (HRTIM1->sMasterRegs.MPER + boost_duty) / 2u;	
}

float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p)
{
  switch (p->elec_type)		//0电压 1电流
  {
  case 0:		//0电压
    p->result = (p->valu) * p->ratio + p->offset;
    break;
  case 1:		//1电流
    p->result = ((p->valu) - I_BIAS) * p->ratio + p->offset;
    break;
  }
  return p->result;
}

void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2) //线性校准运行一次
{
  double a, b;
  p->get_volt1 = get1;					//更新数据
  p->real_valu1 = real1;
  p->get_volt2 = get2;
  p->real_valu2 = real2;
  if ((p->get_volt2 - p->get_volt1 == 0) || (p->real_valu2 - p->real_valu1 == 0))		//若两次值之间无差
  {
    return;
  }

  a = (p->real_valu2 - p->real_valu1) / (p->get_volt2 - p->get_volt1);		//线性校准
  b = (p->real_valu1 - p->get_volt1 * a);

  p->ratio = a * p->ratio;
  p->offset = b;
}
