#include "zf_common_headfile.h"


void motor_init(void){
    // gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    // pwm_init(PWM_R1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    // gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    // pwm_init(PWM_L1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L2, 17000, 0);   
}

bool dir = true;
void motor_test(void)
{
    static int duty = 0;

    if(duty >= 0)                                                           // 正转
        {
//            gpio_set_level(DIR_L1, GPIO_HIGH);                                  // DIR输出高电平
//            pwm_set_duty(PWM_L1, duty * (PWM_DUTY_MAX / 100/2));                  // 计算占空比
//
//            gpio_set_level(DIR_R1, GPIO_HIGH);                                  // DIR输出高电平
//            pwm_set_duty(PWM_R1, duty * (PWM_DUTY_MAX / 100/2));                  // 计算占空比

            gpio_set_level(DIR_L2, GPIO_HIGH);                                  // DIR输出高电平
            pwm_set_duty(PWM_L2, duty * (PWM_DUTY_MAX / 100/2));                  // 计算占空比

            gpio_set_level(DIR_R2, GPIO_LOW);                                  // DIR输出高电平
            pwm_set_duty(PWM_R2, duty * (PWM_DUTY_MAX / 100/2));                  // 计算占空比
        }
        else                                                                    // 反转
        {
//            gpio_set_level(DIR_L1, GPIO_LOW);                                   // DIR输出低电平
//            pwm_set_duty(PWM_L1, (-duty) * (PWM_DUTY_MAX / 100/2));               // 计算占空比
//
//            gpio_set_level(DIR_R1, GPIO_LOW);                                   // DIR输出低电平
//            pwm_set_duty(PWM_R1, (-duty) * (PWM_DUTY_MAX / 100/2));               // 计算占空比

            gpio_set_level(DIR_L2, GPIO_LOW);                                   // DIR输出低电平
            pwm_set_duty(PWM_L2, (-duty) * (PWM_DUTY_MAX / 100/2));               // 计算占空比

            gpio_set_level(DIR_R2, GPIO_HIGH);                                   // DIR输出低电平
            pwm_set_duty(PWM_R2, (-duty) * (PWM_DUTY_MAX / 100/2));               // 计算占空比
        }
        if(dir)                                                                 // 根据方向判断计数方向 本例程仅作参考
        {
            duty ++;                                                            // 正向计数
            if(duty >= MAX_DUTY)                                                // 达到最大值
                dir = false;                                                    // 变更计数方向
        }
        else
        {
            duty --;                                                            // 反向计数
            if(duty <= -MAX_DUTY)                                               // 达到最小值
                dir = true;                                                     // 变更计数方向
        }
        system_delay_ms(50);
}

void motor_balance(int duty)
{
    if(duty >= 0)                                                           // 正转
        {
            gpio_set_level(DIR_L2, GPIO_LOW);                                  // DIR输出高电平
            pwm_set_duty(PWM_L2, duty);                  // 计算占空比

            gpio_set_level(DIR_R2, GPIO_HIGH);                                  // DIR输出高电平
            pwm_set_duty(PWM_R2, duty);                  // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR_L2, GPIO_HIGH);                                   // DIR输出低电平
            pwm_set_duty(PWM_L2, -duty);               // 计算占空比

            gpio_set_level(DIR_R2, GPIO_LOW);                                   // DIR输出低电平
            pwm_set_duty(PWM_R2, -duty);               // 计算占空比
        }
        // system_delay_ms(10);
}
