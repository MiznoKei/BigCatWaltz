#include "zf_common_headfile.h"


void servo_set(void)
{
    pwm_set_duty(SERVO_MOTOR_PWM, 750);
}
