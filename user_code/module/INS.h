/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-04-2021     summerpray       1. doing
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_H
#define INS_H

#ifdef __cplusplus
extern "C"{
#endif

#include <string.h>
#include "Pid.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "ahrs.h"

#ifdef __cplusplus
}
#endif

#include "Communicate.h"
#include "detect_task.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

//由于C板固定方式,再次进行了修整
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_ROLL_ADDRESS_OFFSET   1
#define INS_PITCH_ADDRESS_OFFSET  2



#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \




class INS{
public:

/*******************************************(C) 陀螺仪基本参数 ***********************************************/
    bmi088_real_data_t bmi088_real_data;                            //IMU数据存储
    ist8310_real_data_t ist8310_real_data;                          //磁力计数据存储
    
    fp32 INS_gyro[3];
    fp32 INS_accel[3];
    fp32 INS_mag[3];
    fp32 INS_quat[4];
    fp32 INS_angle[3];                                              //euler angle, unit rad.欧拉角 单位 rad

    //加速度计低通滤波
    fp32 accel_fliter_1[3];
    fp32 accel_fliter_2[3];
    fp32 accel_fliter_3[3];
/*******************************************(C) 陀螺仪基本参数 ***********************************************/

/*******************************************(C) 串行外设接口 ************************************************/
    
    SPI_HandleTypeDef hspi1;                                        //串行外设接口
/*******************************************(C) 串行外设接口 ************************************************/

/*******************************************(C) 陀螺仪返回参数 ***********************************************/
    const fp32 *get_INS_quat_point(void);                                 //获取四元数
    const fp32 *get_INS_angle_point(void);                                //获取欧拉角,0:yaw, 1:roll, 2:pitch单位 rad
    const fp32 *get_gyro_data_point(void);                                //获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
    const fp32 *get_accel_data_point(void);                               //获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
/*******************************************(C) 陀螺仪返回参数 ***********************************************/

/*************************************************(C) PID *************************************************/
    Pid imu_temp_pid;                                               //陀螺仪临时PID
    fp32 temperature_fp32;
/*************************************************(C) PID *************************************************/
    float timing_time;                               //tast run time , unit s.任务运行的时间 单位 s
    void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3],bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

    void init(void);                                                //陀螺仪硬件初始化
    void INS_Info_Get(void);                                        //陀螺仪数据传输
};



/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);

/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

extern INS imu;

#endif
