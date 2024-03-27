/***********************************************************
 * 文件名称          PID.h
 * 
 * 修改记录
 * 日期              作者                备注
 * 2024-03-04       Mizno            first version
 ***********************************************************/

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

// 限幅处理
#define max(a, b) (a > b ? a : b) 
#define min(a, b) (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

struct pid_param
{
    // 速度环参数
    float speed_P;
    float speed_I;
    float speed_D;
    float speed_output;
    // 角度环参数
    float angle_P;  // 输入的比例系数
    float angle_P2; // 自适应后的比例系数
    float angle_I;
    float angle_D;
    float angle_output;
    // 角速度环参数
    float gyro_P;
    float gyro_I;
    float gyro_D;
    float gyro_output;
    // 设置平衡角度
    float setBalance;
};

extern struct pid_param pid_bal;

void pid_main(void);


#endif /* CODE_PID_H_ */