#include "zf_common_headfile.h"
// ips200_width_max    = 240;
// ips200_height_max   = 320;
// IPS200_8X16_FONT

bool ismenu = 1;
bool is1, is2, is3, is4 = 0;

void ips_menu(void)
{

    if (key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        printf("key1\n");
        if (is1)
        {
            gpio_set_level(DIR_L1, GPIO_LOW);
        }
        else
        {
            gpio_set_level(DIR_L1, GPIO_HIGH);
        }
    }
    else if (key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
        printf("key2\n");
        if (is2)
        {
            pwm_set_duty(PWM_L1, 0);
        }
        else
        {
            pwm_set_duty(PWM_L1, PWM_DUTY_MAX / 5);
        }
    }
    else if (key_get_state(KEY_3) == KEY_LONG_PRESS)
    {
        printf("key3\n");
        if (is3)
        {
            gpio_set_level(DIR_R1, GPIO_LOW);
        }
        else
        {
            gpio_set_level(DIR_R1, GPIO_HIGH);
        }
    }
    else if (key_get_state(KEY_4) == KEY_LONG_PRESS)
    {
        printf("key4\n");
        if (is4)
        {
            pwm_set_duty(PWM_R1, 0);
        }
        else
        {
            pwm_set_duty(PWM_R1, PWM_DUTY_MAX / 5);
        }
    }
}

void ips_bluetooth_tuning(void)
{
    ips200_show_string(0, 0, "angle P:");
    ips200_show_string(0, 16, "angle I:");
    ips200_show_string(0, 32, "angle D:");
    ips200_show_string(0, 64, "gtro  P:");
    ips200_show_string(0, 80, "gtro  I:");
    ips200_show_string(0, 96, "gtro  D:");

    ips200_show_float(80,  0, pid_bal.angle_P, 4, 4);
    ips200_show_float(80, 16, pid_bal.angle_I, 4, 4);
    ips200_show_float(80, 32, pid_bal.angle_D, 4, 4);
    ips200_show_float(80, 64, pid_bal.gyro_P , 4, 4);
    ips200_show_float(80, 80, pid_bal.gyro_I , 4, 4);
    ips200_show_float(80, 96, pid_bal.gyro_D , 4, 4);

    //ips200_show_wave(0, 120, data, 128, 64, 64, 32);
}

void ips_show_imu(void)
{
    ips200_show_string(0, 120+0 , "yaw:");
    ips200_show_string(0, 120+16, "pitch:");
    ips200_show_string(0, 120+32, "roll:");

    ips200_show_float(80, 120+ 0, imu.yaw,   4, 4);
    ips200_show_float(80, 120+16, imu.pitch, 4, 4);
    ips200_show_float(80, 120+32, imu.roll,  4, 4);

    ips200_show_string(0, 180+0, "spd_out:");
    ips200_show_string(0, 180+16, "ang_out:");
    ips200_show_string(0, 180+32, "gyr_out:");

# if 0
    ips200_show_float(80, 180+ 0, pid_bal.speed_output, 2, 4);
    ips200_show_float(80, 180+16, pid_bal.angle_output, 2, 4);
    ips200_show_float(80, 180+32, pid_bal.gyro_output, 2, 4);
    //ips200_show_wave(0, 120, data, 128, 64, 64, 32);
# endif

    ips200_show_string(0, 240+0, "deg_x:");
    ips200_show_string(0, 240+16, "deg_y:");
    ips200_show_string(0, 240+32, "deg_z:");

    ips200_show_float(80, 240+ 0, imu.gyro_x, 4, 4);
    ips200_show_float(80, 240+16, imu.gyro_y, 4, 4);
    ips200_show_float(80, 240+32, imu.gyro_z, 4, 4);
}
