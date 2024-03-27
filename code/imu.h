#include "zf_common_headfile.h"

// ��̬����
typedef struct
{
  // ��ÿ��
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // ���ٶ�
  float acc_x;
  float acc_y;
  float acc_z;
   
  float roll;   // ������
  float pitch;  // ������
  float yaw;    // ƫ����
}att_data;


extern att_data imu;    //���������ݴ洢
extern bool isFLT_acc;
extern bool isFLT_gyro;


void  IIR_init(void);   // ���IIR��ͨ�˲�����
void  imu_main(void);        // ����
