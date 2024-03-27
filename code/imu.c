/********************************************************************************
 * @Author: Mizno
 * @Date: 2024-03-07 10:07:10
 * @LastEditTime: 2024-03-25 16:59:27
 * @FilePath: \01_BigCatWaltz_v3.24\code\imu.c
 * @Description: 陀螺仪姿态解算
 * @
 * @Copyright (c) 2024 by Mizno, All Rights Reserved. 
 ********************************************************************************/

#include "zf_common_headfile.h"

// _Attitude att = {0};
// _Matrix Mat = {0};
att_data imu = {0};

bool isyaw = 0;
bool isFLT_acc = 1;
bool isFLT_gyro = 0;
bool isTrans_acc = 0;
bool isTrans_gyro = 1;

const float MahonyPERIOD        = 1.0f;           // 姿态解算周期（ms）
// const float mahony_P            = 0.5f;           // 角速度误差P
// const float mahony_I            = 0.0001f;        // 角速度积分I
const float mahony_P            = 1.0f;           // 角速度误差P
const float mahony_I            = 0.2f;        // 角速度积分I

float IIR_accAlpha = 0.0f , IIR_gyroAlpha = 0.0f;             // IIR滤波器参数α
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;       // 四元数的元素，对应 w,x,y,z

//float Atmpe_Y,Atmpe_X;

//------------------------------------------------------------
//description: IIR滤波器初始化，gyro和acc分别获取一次IIR滤波器参数
//param IIR_accAlpha    滤波器参数
//param IIR_gyroAlpha   滤波器参数
//return 
//------------------------------------------------------------
//void IIR_init()
//{
//    IIR_accAlpha = IIR_getAlpha(0.001f, 20);
//    IIR_gyroAlpha = IIR_getAlpha(0.001f, 20);
//}


//------------------------------------------------------------
//description: 获得IIR滤波器参数
//param T   采样周期
//param fc  截止频率
//return    滤波器参数
//------------------------------------------------------------
float IIR_getAlpha(float T, float fc)
{
    //0.001f,10
    float alpha = T / (T + 1/(2.0f * PI * fc));
    // const float two_pi_T = 2.0f * PI * T; //2πT
    // float alpha = two_pi_T / (two_pi_T + 1);
    return alpha;
}


//------------------------------------------------------------
//description: IIR低通滤波器
//param input   输入
//param output  上一时刻输出
//param alpha   滤波器参数
//return        下一时刻输出
//------------------------------------------------------------
#define FLT_ALPHA 0.35f
float IIR_LowPassFilter(float input, float output, float alpha)
{
    // y[n] = α * x[n] + (1 - α) * y[n-1]
    if (0)
        output = output + alpha * (input - output);
    else
        output = output + FLT_ALPHA * (input - output);
    return output;
}

//------------------------------------------------------------
//description: 快速平方根倒数算法
//param x   输入
//return    输出
//------------------------------------------------------------
float invSqrt(float x)
{
    float halfx = 0.5f * x;           // 计算x的一半，用于后续计算
    float y = x;                      // 将输入值x赋值给变量y
    long i = *(long *)&y;             // 通过类型转换，将y的浮点数位模式解释为一个长整数
    i = 0x5f3759df - (i >> 1);        // 一个神奇的数字0x5f3759df，用于初始近似，并进行位运算
    y = *(float *)&i;                 // 将新计算出的整数位模式重新解释为浮点数
    y = y * (1.5f - (halfx * y * y)); // 牛顿迭代法的一次迭代，用于提高结果的精度
    return y;
}

    /*姿态解算*/
    /* Atmpe_X=(atan(imu.acc_g.y/imu.acc_g.z))*180/3.14;
    Atmpe_Y=(atan(imu.acc_g.x/imu.acc_g.z))*180/3.14;
    hubulvbo_Y(imu.acc_g.y,imu.deg_s.y);
    imu.pitch = 0.2 * imu.acc_g.x + (1 - 0.2) * (Atmpe_X + imu.deg_s.x * 0.001);
    imu.yaw += imu.deg_s.z * 0.001f;
    */

//------------------------------------------------------------
//description: 基于四元数的mahony滤波器姿态估计
//param gx  陀螺仪测量到的角速度值, 单位为度每秒
//param ax  加速度计测量到的加速度值（单位：g）
//return 
//------------------------------------------------------------
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * DELTA_T;

    float vx, vy, vz;   // 通过估计的姿态得到的世界坐标系中重力向量在机体坐标系中的投影
    float ex, ey, ez;   // 加速度计测量的重力向量（ax,ay,az）和预估的重力向量（vx,vy,vz）之间的误差
    float exInt = 0, eyInt = 0, ezInt = 0;      // 误差积分

    // 如果加速度计测量值为零，则退出函数
    if (ax * ay * az == 0)
        return;

    // 转弧度制
    gx = gx * (PI / 180.0f);
    gy = gy * (PI / 180.0f);
    gz = gz * (PI / 180.0f);

    // 快速求平方根倒数 + 归一化
    float rsqrt = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * rsqrt;
    ay = ay * rsqrt;
    az = az * rsqrt;

    // vx,vy,vz 是通过估计的姿态得到的世界坐标系中重力向量在机体坐标系中的投影
    // 从四元数方向余弦矩阵转换得到
    /* 给定单位四元数 q = (q0, q1, q2, q3)，下面是对应的旋转矩阵 R
    旋转矩阵：机体坐标系 -> 地理坐标系
    R = | 1 -2*q2^2 -2*q3^2    2*q1*q2 - 2*q3*q0    2*q1*q3 + 2*q2*q0 |
        | 2*q1*q2 + 2*q3*q0    1 -2*q1^2 -2*q3^2    2*q2*q3 - 2*q1*q0 |
        | 2*q1*q3 - 2*q2*q0    2*q2*q3 + 2*q1*q0    1 -2*q1^2 -2*q2^2 |
    */
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = 1 - 2.0f * q1 * q1 - 2.0f * q2 * q2;
    
    // vx = Mat.DCM_T[0][2];
    // vy = Mat.DCM_T[1][2];
    // vz = Mat.DCM_T[2][2];

    // 计算加速度计测量的重力向量（ax,ay,az）和预估的重力向量（vx,vy,vz）之间的误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    if (0)
    {
        // 对误差向量进行积分
        exInt = exInt + ex * mahony_I;
        eyInt = eyInt + ey * mahony_I;
        ezInt = ezInt + ez * mahony_I;

        // 姿态误差补偿到角速度上，修正角速度积分漂移，通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
        gx = gx + mahony_P * ex + exInt;
        gy = gy + mahony_P * ey + eyInt;
        gz = gz + mahony_P * ez + ezInt;

        // 一阶龙格库塔法更新四元数
        q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * MahonyPERIOD * 0.0005f;
        q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * MahonyPERIOD * 0.0005f;
        q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * MahonyPERIOD * 0.0005f;
        q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * MahonyPERIOD * 0.0005f;
    }
    else
    {
        exInt = exInt + ex * DELTA_T;
        eyInt = eyInt + ey * DELTA_T;
        ezInt = ezInt + ez * DELTA_T;

        gx = gx + mahony_P * ex + exInt * mahony_I;
        gy = gy + mahony_P * ey + eyInt * mahony_I;
        gz = gz + mahony_P * ez + ezInt * mahony_I;

        q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    }

    // 把上述运算后的四元数进行归一化处理，得到了物体经过旋转后的新的四元数。
    rsqrt = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * rsqrt;
    q1 = q1 * rsqrt;
    q2 = q2 * rsqrt;
    q3 = q3 * rsqrt;
}

//------------------------------------------------------------
//description: 四元素转欧拉角，单位为角度制
//param imu.roll    翻滚角
//param imu.pitch   俯仰角
//param imu.yaw     偏航角
//return 
//------------------------------------------------------------
void quat2euler()
{
    float yaw = 0.0f, yaw_last = 0.0f;  // 前后偏航角
    float yaw_change = 0.0f, yaw_change_adjusted = 0.0f, yaw_corrOffset = 0.0f; // 偏航角的变化量和补偿

    // 欧拉角计算,单位为角度制
    imu.roll    = atan2(2.0f * q0 * q1 + 2.0f * q2 * q3, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1) * (180.0f / PI);
    imu.pitch   = asin (2.0f * q1 * q3 - 2.0f * q0 * q2)                                       * (180.0f / PI);
    yaw         = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1) * (180.0f / PI);
    // roll =  atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;     
    // pitch =  asinf(2*q1*q3 - 2*q0*q2)*57.3;                                                          
    // yaw  =  -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3;
    imu.roll=imu.roll+0.5;

    // ----------------对偏航角进行单独处理-------------------
    if(isyaw){
        // 偏航角变化量
        yaw_change = yaw - yaw_last;

        // 修正因为突然大幅度变化而不太可能是真实方向变化的角度
        if (yaw_change < -300)
        {
            yaw_corrOffset = yaw_corrOffset + 360;
        }
        else if (yaw_change > 300)
        {
            yaw_corrOffset = yaw_corrOffset - 360;
        }
        // 更新偏航角
        yaw_last = yaw;

        // 累加偏移量
        yaw = yaw + yaw_corrOffset;

        // 规范化偏航角到[0, 360)度范围内
        if (yaw > 360)
        {
            yaw = yaw - 360;
        }
        else if (yaw < 0)
        {
            yaw = yaw + 360;
        }

        // 计算规范化后的偏航角变化量
        yaw_change_adjusted = yaw - yaw_last;

        // 特判角度
        // int imuflag = 0;
        // if (yaw < -170)
        // {
        //     imuflag = 1;
        // }

        // 确保调整后的偏航角变化量在[-180, 180)度范围内，以适应环绕连续性
        if (yaw_change_adjusted > 180)
        {
            yaw_change_adjusted = yaw_change_adjusted - 360;
        }
        else if (yaw_change_adjusted < -180)
        {
            yaw_change_adjusted = yaw_change_adjusted + 360;
        }

        // 累加偏移量并更新偏航角
        yaw = yaw + yaw_change_adjusted;
    }
    // 储存到结构体
    imu.yaw = yaw;

    // z轴角速度积分的偏航角
    // imu.yaw += imu.deg_s.z  * MahonyPERIOD * 0.001f;
}


// void rotation_matrix(void)
// {
//   Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
//   Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
//   Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

//   Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
//   Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
//   Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);

//   Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
//   Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
//   Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
// }

// void rotation_matrix_T(void)
// {
//   Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
//   Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);
//   Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2);

//   Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
//   Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
//   Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);

//   Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
//   Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
//   Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
// }


// void Matrix_ready(void)
// {
//   rotation_matrix();                      //旋转矩阵更新
//   rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
// }



//------------------------------------------------------------
//description: 中断调用，将IMU数据滤波和解算
//param imu
//return 
//------------------------------------------------------------
void imu_main()
{
    int16 FLT_gyro_x = 0, FLT_gyro_y = 0, FLT_gyro_z = 0;    // 滤波后陀螺仪数据
    int16 FLT_acc_x = 0, FLT_acc_y = 0, FLT_acc_z = 0;       // 滤波后加速度计数据

    //获取 IMU660RA 加速度计和陀螺仪数据
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    // 滤波算法
    if (isFLT_acc)
    {
        FLT_acc_x = IIR_LowPassFilter(imu660ra_acc_x, FLT_acc_x, IIR_accAlpha);
        FLT_acc_y = IIR_LowPassFilter(imu660ra_acc_y, FLT_acc_y, IIR_accAlpha);
        FLT_acc_z = IIR_LowPassFilter(imu660ra_acc_z, FLT_acc_z, IIR_accAlpha);
    }
    else
    {
        FLT_acc_x = imu660ra_acc_x;
        FLT_acc_y = imu660ra_acc_y;
        FLT_acc_z = imu660ra_acc_z;
        
    }

    if (isFLT_gyro)
    {
        FLT_gyro_x = IIR_LowPassFilter(imu660ra_gyro_x, FLT_gyro_x, IIR_gyroAlpha);
        FLT_gyro_y = IIR_LowPassFilter(imu660ra_gyro_y, FLT_gyro_y, IIR_gyroAlpha);
        FLT_gyro_z = IIR_LowPassFilter(imu660ra_gyro_z, FLT_gyro_z, IIR_gyroAlpha);
    }
    else
    {
        FLT_gyro_x = imu660ra_gyro_x;
        FLT_gyro_y = imu660ra_gyro_y;
        FLT_gyro_z = imu660ra_gyro_z;
    }

    // 数据存储
    if(isTrans_acc)
    {
        imu.acc_x = imu660ra_acc_transition(FLT_acc_x);
        imu.acc_y = imu660ra_acc_transition(FLT_acc_y);
        imu.acc_z = imu660ra_acc_transition(FLT_acc_z);
    }
    else
    {
        imu.acc_x = FLT_acc_x;
        imu.acc_y = FLT_acc_y;
        imu.acc_z = FLT_acc_z;
    }
    if(isTrans_gyro)
    {
        imu.gyro_x = imu660ra_gyro_transition(FLT_gyro_x);
        imu.gyro_y = imu660ra_gyro_transition(FLT_gyro_y);
        imu.gyro_z = imu660ra_gyro_transition(FLT_gyro_z);
    }
    else
    {
        imu.gyro_x = FLT_gyro_x;
        imu.gyro_y = FLT_gyro_y;
        imu.gyro_z = FLT_gyro_z;
    }

    // mahony姿态解算 + 欧拉角转换
    mahony_update(imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.acc_x, imu.acc_y, imu.acc_z);
    quat2euler();
    // Matrix_ready();
}
