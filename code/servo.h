#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM             (ATOM1_CH1_P33_9)                           // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ            (50 )                                       // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX           (150)                                       // ���������϶�����Χ �Ƕ�

//1200 �� 300 �� 750 �м�

//900 750 600

void servo_set(void);