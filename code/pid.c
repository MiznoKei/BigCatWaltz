/********************************************************************************
 * @Author: Mizno
 * @Date: 2024-03-07 10:07:10
 * @LastEditTime: 2024-03-25 17:09:25
 * @FilePath: \01_BigCatWaltz_v3.24\code\pid.c
 * @Description: pid控制算法
 * @
 * @Copyright (c) 2024 by Mizno, All Rights Reserved. 
 ********************************************************************************/

#include "zf_common_headfile.h"

float speed1 = 0;   // 编码器获得的速度

int gyro_counter = 0, angle_counter = 0, speed_counter = 0; // 控制环调用计数器

// PID平衡所需参数
struct pid_param pid_bal = {
    // 速度环参数
    .speed_P = 0,
    .speed_I = 0,
    .speed_D = 0,
    .speed_output = 0,
    // 角度环参数
    .angle_P = -20,  // 输入的比例系数
    .angle_P2 = 0, // 自适应后的比例系数
    .angle_I = 0.0071,
    .angle_D = 0.0065,
    .angle_output = 0,
    // 角速度环参数
    .gyro_P = 3.5,
    .gyro_I = 0.5,
    .gyro_D = 2.6,
    .gyro_output = 0,
    // 需要平衡的角度
    .setBalance = 0,
};


//------------------------------------------------------------
//description: 速度环PID
//param error       误差
//param prev_error  前误差
//param speed_output速度环输出
//return 
//------------------------------------------------------------
void pid_SpeedLoop()
{
    static float prev_error = 0.0f; // 前误差
    static float integral = 0.0f;   // 积分量

    float error;    // 误差

    // 计算误差 -speed1 编码器速度
    error = speed1;
    //error = setpoint - speed1;

    // 积分累计
    integral += error;
    // if(error > 0 && (error-error_last) < 0) pid2_error_i = 0;
    // if(error < 0 && (error-error_last) > 0) pid2_error_i = 0;

    //限幅
    integral = range(integral, -300, 300);

    // pid运算
    pid_bal.speed_output = pid_bal.speed_P * error + pid_bal.speed_I * integral + pid_bal.speed_D * (error - prev_error);
    
    // 更新上一次的误差
    prev_error = error;
}



void Vertical(float target_angle, float current_angle, float gyro_x)
{
    float Vetical_PWM;
    static float error;

    error += current_angle - target_angle;

    error = range(error, -8, 8);    //积分限幅


    Vetical_PWM = pid_bal.angle_P * (current_angle - target_angle) +
                  pid_bal.angle_I * error +
                  pid_bal.angle_D * (gyro_x);

    motor_balance(Vetical_PWM);
}


//------------------------------------------------------------
//description: 角度环PID，P自适应改变
//param pid_bal.angle_P     输入的角度环比例系数
//param pid_bal.angle_P2    更新后的比例系数
//param pid_bal.setBalance  车辆目标角度
//param imu.roll            计算出的翻滚角
//return 
//------------------------------------------------------------
//volatile float error_P1=error, angle_P2, angle_D2, increase_sin;
void pid_AngleLoop()
{
    static float prev_error = 0.0f; // 前误差
    static float integral = 0.0f;   // 积分量

    float error;    // 误差
    float increase_sin = 0;

    // 叠加目标角度，上级输出与实际翻滚角
    //error = pid_bal.speed_output + pid_bal.setBalance - imu.pitch;
    error = pid_bal.speed_output + pid_bal.setBalance - imu.roll;

    if(0){
        // PID运算
        pid_bal.angle_output = pid_bal.angle_P2 * error + pid_bal.angle_I * integral + pid_bal.angle_D * (error - prev_error);
        prev_error = error;
        integral += error;
        // 限幅
        integral = range(integral, -100, 100);

        // 对偏差进行变P
        increase_sin = 5 * sin(error / 3.1415926);
        if (error < 0)
        {
            pid_bal.angle_P2 = pid_bal.angle_P * (1 - increase_sin);
        }
        else
        {
            pid_bal.angle_P2 = pid_bal.angle_P * (1 + increase_sin);
        }
    }
    else
    {
        integral += error;
        // 限幅
        integral = range(integral, -1000, 1000);

        pid_bal.angle_output = pid_bal.angle_P * error + pid_bal.angle_I * integral + pid_bal.angle_D * (error - prev_error);

        prev_error = error;
    }

    // 限幅
    pid_bal.angle_output = range(pid_bal.angle_output, -2000, 2000);
}




//------------------------------------------------------------
//description: 角速度环PID
//param pwm             角速度环输出
//param pwm_set_duty    PWM控制电机
//return 
//------------------------------------------------------------
void pid_GyroLoop()
{
    float error, integral = 0;
    static float prev_error, pwm = 0;
    // 得到PI,并限幅
    error = pid_bal.angle_output - imu.gyro_x;
    integral += error;
    integral = range(integral, -1000, 1000);

    // PID运算
    pid_bal.gyro_output = pid_bal.gyro_P * error + pid_bal.gyro_I * integral + pid_bal.gyro_D * (error - prev_error);
    // 限幅
    pid_bal.gyro_output = range(pid_bal.gyro_output, -2000, 2000);

    prev_error = error;
    pwm = pid_bal.gyro_output;

    motor_balance(pwm);
    // if (pwm > 0)
    // {
    //     //小轮向左
    //     pwm_set_duty(ATOM1_CH7_P02_7, 0);
    //     pwm_set_duty(ATOM1_CH5_P02_5, 600 + pwm);
    //     pwm_set_duty(ATOM2_CH5_P33_13, 600 + pwm);
    //     pwm_set_duty(ATOM1_CH6_P23_1, 0);

    // }
    // else
    // {
    //     //小轮向右
    //     pwm_set_duty(ATOM1_CH7_P02_7, 600 - pwm);
    //     pwm_set_duty(ATOM1_CH5_P02_5, 0);
    //     pwm_set_duty(ATOM2_CH5_P33_13, 0);
    //     pwm_set_duty(ATOM1_CH6_P23_1, 600 - pwm);
    // }
}



//------------------------------------------------------------
//description: PID 控制总函数，按不同周期进行三个环的控制
//param imu
//------------------------------------------------------------
void pid_main()
{
#if 0
    if (imu.roll < 20 && imu.roll > -20)
    {
        gyro_counter++;
        angle_counter++;
        speed_counter++;
        if (gyro_counter == 1)
        {
            gyro_counter = 0;
            pid_GyroLoop();
        }
        if (angle_counter == 2)
        {
            angle_counter = 0;
            pid_AngleLoop();
        }
        if (speed_counter == 5)
        {
            speed_counter = 0;
            pid_SpeedLoop();
        }
    }
    else
    {
        motor_init();
    }
#endif
    Vertical(0, imu.roll, imu.gyro_x);
}
