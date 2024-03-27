/********************************************************************************
 * @Author: Mizno
 * @Date: 2024-03-07 10:07:10
 * @LastEditTime: 2024-03-25 16:59:27
 * @FilePath: \01_BigCatWaltz_v3.24\code\imu.c
 * @Description: ��������̬����
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

const float MahonyPERIOD        = 1.0f;           // ��̬�������ڣ�ms��
// const float mahony_P            = 0.5f;           // ���ٶ����P
// const float mahony_I            = 0.0001f;        // ���ٶȻ���I
const float mahony_P            = 1.0f;           // ���ٶ����P
const float mahony_I            = 0.2f;        // ���ٶȻ���I

float IIR_accAlpha = 0.0f , IIR_gyroAlpha = 0.0f;             // IIR�˲���������
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;       // ��Ԫ����Ԫ�أ���Ӧ w,x,y,z

//float Atmpe_Y,Atmpe_X;

//------------------------------------------------------------
//description: IIR�˲�����ʼ����gyro��acc�ֱ��ȡһ��IIR�˲�������
//param IIR_accAlpha    �˲�������
//param IIR_gyroAlpha   �˲�������
//return 
//------------------------------------------------------------
//void IIR_init()
//{
//    IIR_accAlpha = IIR_getAlpha(0.001f, 20);
//    IIR_gyroAlpha = IIR_getAlpha(0.001f, 20);
//}


//------------------------------------------------------------
//description: ���IIR�˲�������
//param T   ��������
//param fc  ��ֹƵ��
//return    �˲�������
//------------------------------------------------------------
float IIR_getAlpha(float T, float fc)
{
    //0.001f,10
    float alpha = T / (T + 1/(2.0f * PI * fc));
    // const float two_pi_T = 2.0f * PI * T; //2��T
    // float alpha = two_pi_T / (two_pi_T + 1);
    return alpha;
}


//------------------------------------------------------------
//description: IIR��ͨ�˲���
//param input   ����
//param output  ��һʱ�����
//param alpha   �˲�������
//return        ��һʱ�����
//------------------------------------------------------------
#define FLT_ALPHA 0.35f
float IIR_LowPassFilter(float input, float output, float alpha)
{
    // y[n] = �� * x[n] + (1 - ��) * y[n-1]
    if (0)
        output = output + alpha * (input - output);
    else
        output = output + FLT_ALPHA * (input - output);
    return output;
}

//------------------------------------------------------------
//description: ����ƽ���������㷨
//param x   ����
//return    ���
//------------------------------------------------------------
float invSqrt(float x)
{
    float halfx = 0.5f * x;           // ����x��һ�룬���ں�������
    float y = x;                      // ������ֵx��ֵ������y
    long i = *(long *)&y;             // ͨ������ת������y�ĸ�����λģʽ����Ϊһ��������
    i = 0x5f3759df - (i >> 1);        // һ�����������0x5f3759df�����ڳ�ʼ���ƣ�������λ����
    y = *(float *)&i;                 // ���¼����������λģʽ���½���Ϊ������
    y = y * (1.5f - (halfx * y * y)); // ţ�ٵ�������һ�ε�����������߽���ľ���
    return y;
}

    /*��̬����*/
    /* Atmpe_X=(atan(imu.acc_g.y/imu.acc_g.z))*180/3.14;
    Atmpe_Y=(atan(imu.acc_g.x/imu.acc_g.z))*180/3.14;
    hubulvbo_Y(imu.acc_g.y,imu.deg_s.y);
    imu.pitch = 0.2 * imu.acc_g.x + (1 - 0.2) * (Atmpe_X + imu.deg_s.x * 0.001);
    imu.yaw += imu.deg_s.z * 0.001f;
    */

//------------------------------------------------------------
//description: ������Ԫ����mahony�˲�����̬����
//param gx  �����ǲ������Ľ��ٶ�ֵ, ��λΪ��ÿ��
//param ax  ���ٶȼƲ������ļ��ٶ�ֵ����λ��g��
//return 
//------------------------------------------------------------
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * DELTA_T;

    float vx, vy, vz;   // ͨ�����Ƶ���̬�õ�����������ϵ�����������ڻ�������ϵ�е�ͶӰ
    float ex, ey, ez;   // ���ٶȼƲ���������������ax,ay,az����Ԥ��������������vx,vy,vz��֮������
    float exInt = 0, eyInt = 0, ezInt = 0;      // ������

    // ������ٶȼƲ���ֵΪ�㣬���˳�����
    if (ax * ay * az == 0)
        return;

    // ת������
    gx = gx * (PI / 180.0f);
    gy = gy * (PI / 180.0f);
    gz = gz * (PI / 180.0f);

    // ������ƽ�������� + ��һ��
    float rsqrt = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * rsqrt;
    ay = ay * rsqrt;
    az = az * rsqrt;

    // vx,vy,vz ��ͨ�����Ƶ���̬�õ�����������ϵ�����������ڻ�������ϵ�е�ͶӰ
    // ����Ԫ���������Ҿ���ת���õ�
    /* ������λ��Ԫ�� q = (q0, q1, q2, q3)�������Ƕ�Ӧ����ת���� R
    ��ת���󣺻�������ϵ -> ��������ϵ
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

    // ������ٶȼƲ���������������ax,ay,az����Ԥ��������������vx,vy,vz��֮������
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    if (0)
    {
        // ������������л���
        exInt = exInt + ex * mahony_I;
        eyInt = eyInt + ey * mahony_I;
        ezInt = ezInt + ez * mahony_I;

        // ��̬���������ٶ��ϣ��������ٶȻ���Ư�ƣ�ͨ������Kp��Ki�������������Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
        gx = gx + mahony_P * ex + exInt;
        gy = gy + mahony_P * ey + eyInt;
        gz = gz + mahony_P * ez + ezInt;

        // һ�����������������Ԫ��
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

    // ��������������Ԫ�����й�һ�������õ������徭����ת����µ���Ԫ����
    rsqrt = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * rsqrt;
    q1 = q1 * rsqrt;
    q2 = q2 * rsqrt;
    q3 = q3 * rsqrt;
}

//------------------------------------------------------------
//description: ��Ԫ��תŷ���ǣ���λΪ�Ƕ���
//param imu.roll    ������
//param imu.pitch   ������
//param imu.yaw     ƫ����
//return 
//------------------------------------------------------------
void quat2euler()
{
    float yaw = 0.0f, yaw_last = 0.0f;  // ǰ��ƫ����
    float yaw_change = 0.0f, yaw_change_adjusted = 0.0f, yaw_corrOffset = 0.0f; // ƫ���ǵı仯���Ͳ���

    // ŷ���Ǽ���,��λΪ�Ƕ���
    imu.roll    = atan2(2.0f * q0 * q1 + 2.0f * q2 * q3, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1) * (180.0f / PI);
    imu.pitch   = asin (2.0f * q1 * q3 - 2.0f * q0 * q2)                                       * (180.0f / PI);
    yaw         = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1) * (180.0f / PI);
    // roll =  atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;     
    // pitch =  asinf(2*q1*q3 - 2*q0*q2)*57.3;                                                          
    // yaw  =  -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3;
    imu.roll=imu.roll+0.5;

    // ----------------��ƫ���ǽ��е�������-------------------
    if(isyaw){
        // ƫ���Ǳ仯��
        yaw_change = yaw - yaw_last;

        // ������ΪͻȻ����ȱ仯����̫��������ʵ����仯�ĽǶ�
        if (yaw_change < -300)
        {
            yaw_corrOffset = yaw_corrOffset + 360;
        }
        else if (yaw_change > 300)
        {
            yaw_corrOffset = yaw_corrOffset - 360;
        }
        // ����ƫ����
        yaw_last = yaw;

        // �ۼ�ƫ����
        yaw = yaw + yaw_corrOffset;

        // �淶��ƫ���ǵ�[0, 360)�ȷ�Χ��
        if (yaw > 360)
        {
            yaw = yaw - 360;
        }
        else if (yaw < 0)
        {
            yaw = yaw + 360;
        }

        // ����淶�����ƫ���Ǳ仯��
        yaw_change_adjusted = yaw - yaw_last;

        // ���нǶ�
        // int imuflag = 0;
        // if (yaw < -170)
        // {
        //     imuflag = 1;
        // }

        // ȷ���������ƫ���Ǳ仯����[-180, 180)�ȷ�Χ�ڣ�����Ӧ����������
        if (yaw_change_adjusted > 180)
        {
            yaw_change_adjusted = yaw_change_adjusted - 360;
        }
        else if (yaw_change_adjusted < -180)
        {
            yaw_change_adjusted = yaw_change_adjusted + 360;
        }

        // �ۼ�ƫ����������ƫ����
        yaw = yaw + yaw_change_adjusted;
    }
    // ���浽�ṹ��
    imu.yaw = yaw;

    // z����ٶȻ��ֵ�ƫ����
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
//   rotation_matrix();                      //��ת�������
//   rotation_matrix_T();                    //��ת�������������
// }



//------------------------------------------------------------
//description: �жϵ��ã���IMU�����˲��ͽ���
//param imu
//return 
//------------------------------------------------------------
void imu_main()
{
    int16 FLT_gyro_x = 0, FLT_gyro_y = 0, FLT_gyro_z = 0;    // �˲�������������
    int16 FLT_acc_x = 0, FLT_acc_y = 0, FLT_acc_z = 0;       // �˲�����ٶȼ�����

    //��ȡ IMU660RA ���ٶȼƺ�����������
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    // �˲��㷨
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

    // ���ݴ洢
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

    // mahony��̬���� + ŷ����ת��
    mahony_update(imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.acc_x, imu.acc_y, imu.acc_z);
    quat2euler();
    // Matrix_ready();
}
