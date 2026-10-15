#include "main.h"
#include "stdint.h"
#include "bsp_dwt.h"

static uint32_t CPU_FREQ_Hz;
static uint32_t CYCCNT_RountCount;
static float DWT_Timeline;

/**
 * @description: 初始化DWT外设
 * @param 系统主频，单位为MHz
 * @return: void
 * @note: 
 */ 
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CYCCNT_RountCount = 0;
    DWT_Timeline = 0;
}

/**
 * @description: 读取时间差，返回float
 * @param 用于存储计数器数值的外部变量地址
 * @return: float类型，单位为us
 * @note: 通常在需要计时的函数段前调用该函数记录起始时间，在结束处再次调用计算时间差
 */ 
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz))*1e6f;
    *cnt_last = cnt_now;
    return dt;
}

/**
 * @description: 读取时间差，返回double
 * @param 用于存储计数器数值的外部变量地址
 * @return: double类型，单位为s
 * @note: 通常在需要计时的函数段前调用该函数记录起始时间，在结束处再次调用计算时间差
 */ 
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    return dt;
}

/**
 * @description: 记录时间戳
 * @param {unused} 
* @return: float 时间戳数值，单位为s
 * @note: 需定期调用以保证其准确
 */ 
float DWT_GetTimeline(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint32_t CYCCNT_Last;

    if (cnt_now < CYCCNT_Last)
        CYCCNT_RountCount++;

    DWT_Timeline = ((uint64_t)((uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX) + (uint64_t)cnt_now) / ((float)(CPU_FREQ_Hz));

    CYCCNT_Last = cnt_now;
	
	return DWT_Timeline;
}
