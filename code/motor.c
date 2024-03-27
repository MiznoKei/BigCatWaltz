#include "zf_common_headfile.h"


void motor_init(void){
    // gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    // pwm_init(PWM_R1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    // gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    // pwm_init(PWM_L1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_R2, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_L2, 17000, 0);   
}

bool dir = true;
void motor_test(void)
{
    static int duty = 0;

    if(duty >= 0)                                                           // ��ת
        {
//            gpio_set_level(DIR_L1, GPIO_HIGH);                                  // DIR����ߵ�ƽ
//            pwm_set_duty(PWM_L1, duty * (PWM_DUTY_MAX / 100/2));                  // ����ռ�ձ�
//
//            gpio_set_level(DIR_R1, GPIO_HIGH);                                  // DIR����ߵ�ƽ
//            pwm_set_duty(PWM_R1, duty * (PWM_DUTY_MAX / 100/2));                  // ����ռ�ձ�

            gpio_set_level(DIR_L2, GPIO_HIGH);                                  // DIR����ߵ�ƽ
            pwm_set_duty(PWM_L2, duty * (PWM_DUTY_MAX / 100/2));                  // ����ռ�ձ�

            gpio_set_level(DIR_R2, GPIO_LOW);                                  // DIR����ߵ�ƽ
            pwm_set_duty(PWM_R2, duty * (PWM_DUTY_MAX / 100/2));                  // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {
//            gpio_set_level(DIR_L1, GPIO_LOW);                                   // DIR����͵�ƽ
//            pwm_set_duty(PWM_L1, (-duty) * (PWM_DUTY_MAX / 100/2));               // ����ռ�ձ�
//
//            gpio_set_level(DIR_R1, GPIO_LOW);                                   // DIR����͵�ƽ
//            pwm_set_duty(PWM_R1, (-duty) * (PWM_DUTY_MAX / 100/2));               // ����ռ�ձ�

            gpio_set_level(DIR_L2, GPIO_LOW);                                   // DIR����͵�ƽ
            pwm_set_duty(PWM_L2, (-duty) * (PWM_DUTY_MAX / 100/2));               // ����ռ�ձ�

            gpio_set_level(DIR_R2, GPIO_HIGH);                                   // DIR����͵�ƽ
            pwm_set_duty(PWM_R2, (-duty) * (PWM_DUTY_MAX / 100/2));               // ����ռ�ձ�
        }
        if(dir)                                                                 // ���ݷ����жϼ������� �����̽����ο�
        {
            duty ++;                                                            // �������
            if(duty >= MAX_DUTY)                                                // �ﵽ���ֵ
                dir = false;                                                    // �����������
        }
        else
        {
            duty --;                                                            // �������
            if(duty <= -MAX_DUTY)                                               // �ﵽ��Сֵ
                dir = true;                                                     // �����������
        }
        system_delay_ms(50);
}

void motor_balance(int duty)
{
    if(duty >= 0)                                                           // ��ת
        {
            gpio_set_level(DIR_L2, GPIO_LOW);                                  // DIR����ߵ�ƽ
            pwm_set_duty(PWM_L2, duty);                  // ����ռ�ձ�

            gpio_set_level(DIR_R2, GPIO_HIGH);                                  // DIR����ߵ�ƽ
            pwm_set_duty(PWM_R2, duty);                  // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {
            gpio_set_level(DIR_L2, GPIO_HIGH);                                   // DIR����͵�ƽ
            pwm_set_duty(PWM_L2, -duty);               // ����ռ�ձ�

            gpio_set_level(DIR_R2, GPIO_LOW);                                   // DIR����͵�ƽ
            pwm_set_duty(PWM_R2, -duty);               // ����ռ�ձ�
        }
        // system_delay_ms(10);
}
