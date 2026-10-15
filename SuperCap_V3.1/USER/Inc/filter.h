#ifndef __filter_H
#define	__filter_H
//#include "stm32g4xx_hal.h"
#include "stdint.h"
//#include "core_cm4.h"
    typedef  uint8_t    u8;                 /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
    typedef  uint16_t   u16;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
    typedef  uint32_t   u32;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
//1.限幅滤波
//u16 amplitudeLimiterFilter(void);

//2.中位值滤波
u16 middleValueFilter(void);

//3.算术平均滤波
//u16 averageFilter(void);

//4.递推平均滤波法（又称滑动平均滤波法）
//u16 moveAverageFilter(void);

//5.中位值平均滤波（防脉冲干扰平均滤波法）
//u16 middleAverageFilter(void);

//6.递推中位值滤波法
//u16 recursionMiddleFilter(void);

//7.限幅平均滤波法
//u16 filter(void );

//8.一阶滞后滤波法
//u16 firstOrderFilter(u16 newValue, u16 oldValue);

//9.加权递推平均滤波法（并没有递推）
//u16 weightAverageFilter(void)
	
//10.消抖滤波法
//u16 glitchFilter(u16 oldValue);

//11.三西格玛滤波法
//u16 Three_Standard_Deviation_filter(void);

//12.去极值算数平均滤波法
//u16 abandon_extremum_averageFilter(void);

//获得AD值
u16 getValue(void);
#define low_filter_default {0, 0, 0, 0, 0, 0}
typedef struct
{
	float  Input;
	float  Output;
	float  Fc;
	float  Fs;
	float  Ka;
	float  Kb;
	
}LOWPASS_FILTER_STRUCT;
typedef struct
{
	double  Input;
	double  Output;
	double  Input_buff[3];
	double  Output_buff[3];
	//double  fenzi;
	//double  fenmu;
	
}MY_LOWPASS_FILTER_STRUCT;
#define PI    3.14159265F // π
#define PI2   6.28318530F // 2π
extern LOWPASS_FILTER_STRUCT lpf[5];
extern void bubble_sort( uint16_t *a ,unsigned char Num);//冒泡法排序
extern uint32_t middle_filter(uint16_t *a ,unsigned char Num,char extremum);
extern void low_filter_calc(LOWPASS_FILTER_STRUCT *p);
extern void low_filter_init(LOWPASS_FILTER_STRUCT *p,float sample_f,float cutoff_f);
#endif

