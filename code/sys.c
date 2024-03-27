#include "zf_common_headfile.h"


void sys_init(void){
    ips200_init(IPS200_TYPE_SPI);
    ips200_show_string(0, 0, "start init.");

    key_init(20);   // ������ʼ��

    //��������ʼ��
	// LSB��33_7   DIR��33_6
	// LSB��02_8   DIR��00_9
	// LSB��10_3   DIR��10_1
	// LSB��20_3   DIR��20_0
    // ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
    // ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
    // ENC_InitConfig( ENC4_InPut_P02_8,  ENC4_Dir_P00_9);
    
    // ��̬������imu660ra��ʼ��
    if(1){
        while(1)
        {
            if(imu660ra_init())
                ips200_show_string(0, 32, "IMU660RA init error.");// IMU660RA ��ʼ��ʧ��
                //printf("\r\n IMU660RA init error.");
            else
                break;
        }
    }

    //��ȡIIR��ͨ�˲�
    // IIR_init(); 
    
    // ��ʼ������UART0
    uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);

    // ������ʼ��
    if(1){
        while(1)                    
        {
            if(bluetooth_ch9141_init())
            {
                ips200_show_string(0, 32, "CH9141 init error.");
            }
            else{
                bluetooth_ch9141_send_byte('\r');
                bluetooth_ch9141_send_byte('\n');
                bluetooth_ch9141_send_string("SEEKFREE ch9141 success.\r\n");
                break;
            }
        }
    }

    // �����ʼ��
    motor_init();

    //�����ʼ��
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 750);

    // �жϳ�ʼ��
    // ö��ͨ����CCU60_CH0, CCU60_CH1, CCU61_CH0, CCU61_CH1
    //pit_ms_init(CCU60_CH0, 20); // ���������ݲɼ�
    pit_ms_init(CCU60_CH1, 5);  // imu660���ݲɼ�
    // pit_ms_init(CCU61_CH0, 5); // pid������

    ips200_show_string(0, 16, "init success.");
    system_delay_ms(1000);
    ips200_clear();
}
