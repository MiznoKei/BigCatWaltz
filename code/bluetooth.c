#include "zf_common_headfile.h"


void bluetooth_pid_tuning(void)
{
    uint8 data_buffer[32];
    uint8 data_len = 0;
    float temp = 0;

    data_len = (uint8)bluetooth_ch9141_read_buffer(data_buffer, 32); // 查看是否有消息 默认缓冲区是 BLUETOOTH_CH9141_BUFFER_SIZE 总共 64 字节
    if (data_len != 0)                                             // 收到了消息 读取函数会返回实际读取到的数据个数
    {
        if (data_len < 32) {
            data_buffer[data_len] = '\0';
        } else {
            // 如果接收到的数据长度等于缓冲区大小，就在最后一个位置设置空字符
            data_buffer[31] = '\0';
        }

        //ips200_show_string(0, 32, (const char *)data_buffer); // 假设从屏幕坐标(0,0)开始显示


        // bluetooth_ch9141_send_buffer(data_buffer, data_len); // 将收到的消息发送回去
        // memset(data_buffer, 0, 32);
        // func_uint_to_str((char *)data_buffer, data_len);
        // bluetooth_ch9141_send_string("\r\ndata len:");                            // 显示实际收到的数据信息
        // bluetooth_ch9141_send_buffer(data_buffer, strlen((const char *)data_buffer)); // 显示收到的数据个数
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
