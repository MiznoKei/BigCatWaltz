#include "zf_common_headfile.h"

#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR_R1              (P02_4)
#define PWM_R1              (ATOM0_CH5_P02_5)
#define DIR_L1              (P02_6)
#define PWM_L1              (ATOM0_CH7_P02_7)

#define DIR_R2              (P21_2)
#define PWM_R2              (ATOM0_CH1_P21_3)
#define DIR_L2              (P21_4)
#define PWM_L2              (ATOM0_CH3_P21_5)

void motor_init(void);
void motor_test(void);
void motor_balance(int pwm);