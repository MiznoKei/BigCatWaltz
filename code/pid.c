/********************************************************************************
 * @Author: Mizno
 * @Date: 2024-03-07 10:07:10
 * @LastEditTime: 2024-03-25 17:09:25
 * @FilePath: \01_BigCatWaltz_v3.24\code\pid.c
 * @Description: pid�����㷨
 * @
 * @Copyright (c) 2024 by Mizno, All Rights Reserved. 
 ********************************************************************************/

#include "zf_common_headfile.h"

float speed1 = 0;   // ��������õ��ٶ�

int gyro_counter = 0, angle_counter = 0, speed_counter = 0; // ���ƻ����ü�����

// PIDƽ���������
struct pid_param pid_bal = {
    // �ٶȻ�����
    .speed_P = 0,
    .speed_I = 0,
    .speed_D = 0,
    .speed_output = 0,
    // �ǶȻ�����
    .angle_P = -20,  // ����ı���ϵ��
    .angle_P2 = 0, // ����Ӧ��ı���ϵ��
    .angle_I = 0.0071,
    .angle_D = 0.0065,
    .angle_output = 0,
    // ���ٶȻ�����
    .gyro_P = 3.5,
    .gyro_I = 0.5,
    .gyro_D = 2.6,
    .gyro_output = 0,
    // ��Ҫƽ��ĽǶ�
    .setBalance = 0,
};


//------------------------------------------------------------
//description: �ٶȻ�PID
//param error       ���
//param prev_error  ǰ���
//param speed_output�ٶȻ����
//return 
//------------------------------------------------------------
void pid_SpeedLoop()
{
    static float prev_error = 0.0f; // ǰ���
    static float integral = 0.0f;   // ������

    float error;    // ���

    // ������� -speed1 �������ٶ�
    error = speed1;
    //error = setpoint - speed1;

    // �����ۼ�
    integral += error;
    // if(error > 0 && (error-error_last) < 0) pid2_error_i = 0;
    // if(error < 0 && (error-error_last) > 0) pid2_error_i = 0;

    //�޷�
    integral = range(integral, -300, 300);

    // pid����
    pid_bal.speed_output = pid_bal.speed_P * error + pid_bal.speed_I * integral + pid_bal.speed_D * (error - prev_error);
    
    // ������һ�ε����
    prev_error = error;
}



void Vertical(float target_angle, float current_angle, float gyro_x)
{
    float Vetical_PWM;
    static float error;

    error += current_angle - target_angle;

    error = range(error, -8, 8);    //�����޷�


    Vetical_PWM = pid_bal.angle_P * (current_angle - target_angle) +
                  pid_bal.angle_I * error +
                  pid_bal.angle_D * (gyro_x);

    motor_balance(Vetical_PWM);
}


//------------------------------------------------------------
//description: �ǶȻ�PID��P����Ӧ�ı�
//param pid_bal.angle_P     ����ĽǶȻ�����ϵ��
//param pid_bal.angle_P2    ���º�ı���ϵ��
//param pid_bal.setBalance  ����Ŀ��Ƕ�
//param imu.roll            ������ķ�����
//return 
//------------------------------------------------------------
//volatile float error_P1=error, angle_P2, angle_D2, increase_sin;
void pid_AngleLoop()
{
    static float prev_error = 0.0f; // ǰ���
    static float integral = 0.0f;   // ������

    float error;    // ���
    float increase_sin = 0;

    // ����Ŀ��Ƕȣ��ϼ������ʵ�ʷ�����
    //error = pid_bal.speed_output + pid_bal.setBalance - imu.pitch;
    error = pid_bal.speed_output + pid_bal.setBalance - imu.roll;

    if(0){
        // PID����
        pid_bal.angle_output = pid_bal.angle_P2 * error + pid_bal.angle_I * integral + pid_bal.angle_D * (error - prev_error);
        prev_error = error;
        integral += error;
        // �޷�
        integral = range(integral, -100, 100);

        // ��ƫ����б�P
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
        // �޷�
        integral = range(integral, -1000, 1000);

        pid_bal.angle_output = pid_bal.angle_P * error + pid_bal.angle_I * integral + pid_bal.angle_D * (error - prev_error);

        prev_error = error;
    }

    // �޷�
    pid_bal.angle_output = range(pid_bal.angle_output, -2000, 2000);
}




//------------------------------------------------------------
//description: ���ٶȻ�PID
//param pwm             ���ٶȻ����
//param pwm_set_duty    PWM���Ƶ��
//return 
//------------------------------------------------------------
void pid_GyroLoop()
{
    float error, integral = 0;
    static float prev_error, pwm = 0;
    // �õ�PI,���޷�
    error = pid_bal.angle_output - imu.gyro_x;
    integral += error;
    integral = range(integral, -1000, 1000);

    // PID����
    pid_bal.gyro_output = pid_bal.gyro_P * error + pid_bal.gyro_I * integral + pid_bal.gyro_D * (error - prev_error);
    // �޷�
    pid_bal.gyro_output = range(pid_bal.gyro_output, -2000, 2000);

    prev_error = error;
    pwm = pid_bal.gyro_output;

    motor_balance(pwm);
    // if (pwm > 0)
    // {
    //     //С������
    //     pwm_set_duty(ATOM1_CH7_P02_7, 0);
    //     pwm_set_duty(ATOM1_CH5_P02_5, 600 + pwm);
    //     pwm_set_duty(ATOM2_CH5_P33_13, 600 + pwm);
    //     pwm_set_duty(ATOM1_CH6_P23_1, 0);

    // }
    // else
    // {
    //     //С������
    //     pwm_set_duty(ATOM1_CH7_P02_7, 600 - pwm);
    //     pwm_set_duty(ATOM1_CH5_P02_5, 0);
    //     pwm_set_duty(ATOM2_CH5_P33_13, 0);
    //     pwm_set_duty(ATOM1_CH6_P23_1, 600 - pwm);
    // }
}



//------------------------------------------------------------
//description: PID �����ܺ���������ͬ���ڽ����������Ŀ���
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
