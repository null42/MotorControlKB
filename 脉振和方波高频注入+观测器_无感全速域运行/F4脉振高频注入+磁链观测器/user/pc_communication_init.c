/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#include "main.h"
#include "pc_communication_init.h"





s32 int_test1 = 0;
s32 int_test2 = 2;
s32 int_test3 = 3;
s32 int_test4 = 4;
s32 int_test5 = 5;
s32 int_test6 = 6;
s32 int_test7 = 7;
s32 int_test8 = 8;
s32 int_test9 = 9;
s32 int_test10 = 10;
float float_test1 = 0.0f;
float float_test2 = 0.0f;
float float_test3 = 3.0f;
float float_test4 = 4.0f;
float float_test5 = 5.0f;
float float_test6 = 6.0f;
float float_test7 = 7.0f;
float float_test8 = 8.0f;
float float_test9 = 9.0f;
float float_test10 = 0.0f;

u16 position_vlue;

extern float Q_0_0;
extern float Q_1_1;
extern float Q_2_2;
extern float Q_3_3;
extern float R_0_0;
extern float R_1_1;

extern float Rs;
extern float Ls;
extern float flux;
extern int32_t ia_test,ib_test,ic_test;

extern float theta_add;
extern float Ia_test,Ib_test,Ic_test;
void pc_communication_init(void)
{
  communication_init();
  float_wave_upload_init(0,&FOC_Input.theta);
  float_wave_upload_init(1,&float_test3);
  float_wave_upload_init(2,&Flux_Observer.theta);
  float_wave_upload_init(3,&Flux_Observer.lpf_we);
  float_wave_upload_init(4,&float_test1);
  float_wave_upload_init(5,&Current_Lpf_Idq.Iq);
  int_wave_upload_init(0,0x06,&int_test1);
  int_wave_upload_init(1,0x06,&int_test2);
  int_wave_upload_init(2,0x06,&int_test3);
  int_wave_upload_init(3,0x06,&int_test4);
  float_data_upload_init(0,&float_test1);
  float_data_upload_init(1,&float_test1);
  float_data_upload_init(2,&float_test1);
  float_data_upload_init(3,&float_test1);
  float_data_upload_init(4,&float_test1);
  float_data_upload_init(5,&float_test1);
  float_data_upload_init(6,&float_test1);
  float_data_upload_init(7,&float_test1);
  float_data_upload_init(8,&float_test1);
  float_data_upload_init(9,&float_test1);
  int_data_upload_init(0,0x06,&int_test1);
  int_data_upload_init(1,0x06,&int_test2);
  int_data_upload_init(2,0x06,&int_test3);
  int_data_upload_init(3,0x06,&int_test4);
  int_data_upload_init(4,0x06,&int_test5);
  int_data_upload_init(5,0x06,&int_test6);
  int_data_upload_init(6,0x06,&int_test7);
  int_data_upload_init(7,0x06,&int_test8);
  int_data_upload_init(8,0x06,&int_test9);
  int_data_upload_init(9,0x06,&int_test10);
  
  float_data_download_init(0,&float_test2);
  float_data_download_init(1,&float_test2);
  float_data_download_init(2,&float_test1);
  float_data_download_init(3,&float_test1);
  float_data_download_init(4,&float_test1);
  float_data_download_init(5,&float_test1);
  float_data_download_init(6,&float_test1);
  float_data_download_init(7,&float_test1);
  float_data_download_init(8,&float_test1);
  float_data_download_init(9,&float_test1);
  
  speed_ref_data_download_init(&Speed_Ref);
  motor_start_stop_data_download_init(&motor_start_stop);
  position_ref_data_download_init(&position_vlue);
}
