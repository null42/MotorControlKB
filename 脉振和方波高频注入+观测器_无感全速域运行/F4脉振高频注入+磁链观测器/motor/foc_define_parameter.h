/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#ifndef __FOC_DEFINE_PARAMETER_H_
#define __FOC_DEFINE_PARAMETER_H_


#define MOTOR_STARTUP_CURRENT   1.0f   //电机启动电流，根据自己实际负载设置 
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  //速度环切入闭环的速度  单位: rad/s

//有感FOC 或 无感FOC选择，总得注释掉其中一个
//#define HALL_FOC_SELECT          //此行注释掉就不使用有感FOC运行
#define SENSORLESS_FOC_SELECT    //此行注释掉就不使用无感感FOC运行

//传感器选择
//#define HALL_SENSOR_SELECT      //此行注释掉就不使用霍尔传感器
#define ENCODER_SENSOR_SELECT   //此行注释掉就不使用编码器传感器

//电机参数配置（电阻，电感，磁链）
#define RS_PARAMETER     0.59f           //电阻
#define LS_PARAMETER     0.001f          //电感
#define FLUX_PARAMETER   0.01150f        //磁链

#define MOTOR_POLE_PAIRS   2

#endif
