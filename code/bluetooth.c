#include "zf_common_headfile.h"


void bluetooth_pid_tuning(void)
{
    uint8 data_buffer[32];
    uint8 data_len = 0;
    float temp = 0;

    data_len = (uint8)bluetooth_ch9141_read_buffer(data_buffer, 32); // �鿴�Ƿ�����Ϣ Ĭ�ϻ������� BLUETOOTH_CH9141_BUFFER_SIZE �ܹ� 64 �ֽ�
    if (data_len != 0)                                             // �յ�����Ϣ ��ȡ�����᷵��ʵ�ʶ�ȡ�������ݸ���
    {
        if (data_len < 32) {
            data_buffer[data_len] = '\0';
        } else {
            // ������յ������ݳ��ȵ��ڻ�������С���������һ��λ�����ÿ��ַ�
            data_buffer[31] = '\0';
        }

        //ips200_show_string(0, 32, (const char *)data_buffer); // �������Ļ����(0,0)��ʼ��ʾ


        // bluetooth_ch9141_send_buffer(data_buffer, data_len); // ���յ�����Ϣ���ͻ�ȥ
        // memset(data_buffer, 0, 32);
        // func_uint_to_str((char *)data_buffer, data_len);
        // bluetooth_ch9141_send_string("\r\ndata len:");                            // ��ʾʵ���յ���������Ϣ
        // bluetooth_ch9141_send_buffer(data_buffer, strlen((const char *)data_buffer)); // ��ʾ�յ������ݸ���
        // bluetooth_ch9141_send_string(".\r\n");

        temp = func_str_to_float((char *)&data_buffer[2]);
        switch ((char)data_buffer[0])
        {
        case 'A':
            //ips200_show_uint(0, 0, data_buffer[0], 3);
            pid_bal.angle_P = temp;
            break;
        case 'B':
            pid_bal.angle_I = temp;
            break;
        case 'C':
            pid_bal.angle_D = temp;
            break;
        case 'D':
            //ips200_show_uint(0, 0, data_buffer[0], 3);
            pid_bal.gyro_P = temp;
            break;
        case 'E':
            pid_bal.gyro_I = temp;
            break;
        case 'F':
            pid_bal.gyro_D = temp;
            break;
        case 'Y':
            isFLT_acc = temp;
            break;
        case 'Z':
            isFLT_gyro = temp;
            break;
        default:
            break;
        }

        memset(data_buffer, 0, 32);
    }
}
