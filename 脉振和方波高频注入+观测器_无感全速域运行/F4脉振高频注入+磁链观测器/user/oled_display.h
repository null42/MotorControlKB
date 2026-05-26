/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#ifndef __OLED_DISPLAY_H_
#define __OLED_DISPLAY_H_












extern u8 usb_open_display_flag;
extern u8 data_upload_display_flag;

extern u8 motor_run_display_flag;
extern u8 display_static_flag;
extern u8 init_dispaly_flag;
extern u8 display_index_key;
extern u8 display_cnt;
extern u8 display_flag;
extern u8 drv8301_fault_flag;
extern void oled_display_handle(void);
#endif
