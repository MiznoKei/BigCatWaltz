/***********************************************************
 * �ļ�����          PID.h
 * 
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-03-04       Mizno            first version
 ***********************************************************/

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

// �޷�����
#define max(a, b) (a > b ? a : b) 
#define min(a, b) (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

struct pid_param
{
    // �ٶȻ�����
    float speed_P;
    float speed_I;
    float speed_D;
    float speed_output;
    // �ǶȻ�����
    float angle_P;  // ����ı���ϵ��
    float angle_P2; // ����Ӧ��ı���ϵ��
    float angle_I;
    float angle_D;
    float angle_output;
    // ���ٶȻ�����
    float gyro_P;
    float gyro_I;
    float gyro_D;
    float gyro_output;
    // ����ƽ��Ƕ�
    float setBalance;
};

extern struct pid_param pid_bal;

void pid_main(void);


#endif /* CODE_PID_H_ */