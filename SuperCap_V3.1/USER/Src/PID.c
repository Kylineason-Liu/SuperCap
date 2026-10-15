#include "main.h"
#include "PID.h"

#define LimitMax(input, max) \
  {                          \
    if (input > max)         \
    {                        \
      input = max;           \
    }                        \
	}													 \
	
#define LimitMin(input, min) \
  {                          \
    if (input < min)         \
    {                        \
      input = min;           \
    }                        \
	}

pid_type_def volt_high_loop;
pid_type_def volt_low_loop;
pid_type_def curr_loop;

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
void PID_init(pid_type_def *pid, uint8_t mode, const float p, const float i, const float d, float max_out, float max_iout, float min_out, float min_iout)
{
  if (pid == NULL || pid == NULL)
  {
    return;
  }
  pid->mode = mode;
  pid->Kp = p;
  pid->Ki = i;
  pid->Kd = d;
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->min_out = min_out;
  pid->min_iout = min_iout;  
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float PID_calc(pid_type_def *pid, float ref, float set)
{
  if (pid == NULL)
  {
    return 0.0f;
  }

  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  if (pid->mode == PID_POSITION)
  {
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
  }
  else if (pid->mode == PID_DELTA)
  {
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
	LimitMin(pid->out, pid->min_out);
  }
  return pid->out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = 0.0f;
}
