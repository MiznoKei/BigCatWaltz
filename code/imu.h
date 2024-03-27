#include "zf_common_headfile.h"

// 姿态参数
typedef struct
{
  // 度每秒
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // 加速度
  float acc_x;
  float acc_y;
  float acc_z;
   
  float roll;   // 翻滚角
  float pitch;  // 俯仰角
  float yaw;    // 偏航角
}att_data;


extern att_data imu;    //陀螺仪数据存储
extern bool isFLT_acc;
extern bool isFLT_gyro;


void  IIR_init(void);   // 获得IIR低通滤波参数
void  imu_main(void);        // 解算
