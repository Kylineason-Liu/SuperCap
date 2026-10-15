#ifndef PID_H
#define PID_H

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;
    float T;//计算周期
    
    float max_out;  //最大输出
    float min_out;  //最输出	
    float max_iout; //最大积分输出
    float min_iout; //最小积分输出
    float set;//设定值
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

void PID_init(pid_type_def *pid, uint8_t mode, const float p,const float i,const float d, float max_out, float max_iout,float min_out,float min_iout);
float PID_calc(pid_type_def *pid, float ref, float set);
void PID_clear(pid_type_def *pid);

extern pid_type_def volt_high_loop;
extern pid_type_def volt_low_loop;
extern pid_type_def curr_loop;
#endif
