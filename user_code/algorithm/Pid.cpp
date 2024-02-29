#include "Pid.h"

#include "user_lib.h"

#define NOW 0
#define OLD 1
#define NEW 2

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_iout: pid最大积分输出
  * * @param[in]      max_out: pid最大输出
  * @retval         none
  */
void Pid::init(pid_mode_e mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_)
{
    mode = mode_;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.max_iout = pid_parm[3];
    data.max_out = pid_parm[4];

    data.set = set_;
    data.ref = ref_;
    data.error = *set_ - *ref_;

    if (data.mode == PID_ANGLE)
        data.error_delta = erro_delta_;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */


 fp32 Pid::pid_calc()
 {
     data.last_error = data.error;
     data.error = *data.set - *data.ref;
     if (mode == PID_SPEED)
         data.error_delta = data.error - data.last_error;

     if (mode == PID_ANGLE){
        data.error = rad_format(data.error);
        data.error_delta = data.error - data.last_error;       
         }

     data.Pout = data.Kp * data.error;
     data.Iout += data.Ki * data.error;
     data.Dout = data.Kd * (data.error_delta);

     LimitMax(data.Iout, data.max_iout);

     data.out = data.Pout + data.Iout + data.Dout;
     LimitMax(data.out, data.max_out);

     return data.out;
}

//  fp32 Pid::pid_calc(fp32 _T)
//  {
//     data.last_error = data.error;
//     data.error = *data.set - *data.ref;

//     data.Pout = data.Kp * data.error;

//     if (mode == PID_SPEED)
//         data.error_delta = data.error - data.last_error;

//      if (mode == PID_ANGLE){
//         data.error = rad_format(data.error);
//         data.error_delta = data.error - data.last_error;       
//          }

//     if(_T == 0)
//     {
//         _T = getCycleT();
//         //如果间隔时间过长, 不计算积分微分
//         if(_T > 0.1f) //100ms
//         {
//             Clear();
//             data.out = data.Pout;
//             LimitMax(data.out,data.max_out);
//             return data.out;
//         }
//     }

//     data.Iout += data.Ki * data.error * _T;
//     LimitMax(data.Iout, data.max_iout);

//     if(_T != 0)
//     {
//         data.Dout  = data.error_delta / _T * data.Kd;
//     }
//     if(data.Ki == 0)
//     {
//         data.Iout = 0;
//     }

//     data.out = data.Pout + data.Iout + data.Dout;
//     LimitMax(data.out, data.max_out);
//     return data.out;
//  }

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void Pid::pid_clear()
{
    data.last_error = 0;
    data.error = 0;
    *data.set = 0;
    *data.ref = 0;
    data.out =  0;
    data.Pout = 0;
    data.Iout = 0;
    data.Dout = 0;
}

void Pid::Clear()
{
    data.last_error = 0;
    data.Iout = 0;
    data.Dout = 0;
    data.out = 0;
    getCycleT();
}


fp32 Pid::getCycleT()	
{
	cycleTime[OLD] = cycleTime[NOW];	//上一次的时间
	cycleTime[NOW] = getSysTimeUs()/1000000.0f; //本次的时间
	cycleTime[NEW] = (( cycleTime[NOW] - cycleTime[OLD] ) );//间隔的时间（周期）
	//第一次运行返回零
	if(cycleTime[OLD]==0)
	{
		return 0;
	}
	return cycleTime[NEW];
}