/*
 * application.h
 *
 *  Created on: 27 Mar 2018
 *      Author: LMD
 */
#ifndef SOURCE_APPLICATION_H_
#define SOURCE_APPLICATION_H_

#define SCI             1
#define CAN             2

#define  COMM           SCI//SCI //CAN


#define SW_OVERCURRENT			_IQ(15.0/20)
#define ACCURRENTTRANSFE    	_IQ(3.3/4096/0.0192/20)           //AC电流标幺 64A
#define DCVOLAGETRANSFE    		_IQ(3.3/4096/0.00286564/1024*1.05)  //DC电压标幺 1024V

#define ANGULARSPEED			_IQ(1.0/4096)					// 角速度标幺 4096

#define KWI2V		(long)(4096.0*64/1024)    //角速度乘以电流转化成电压变比

#define TIMER10MS			10
#define TIMER20MS			20
#define TIMER100MS			100
#define TIMER200MS			200
#define TIMER400MS			400
#define TIMER500MS			500
#define TIMER1S				1000
#define TIMER2S				2000
#define TIMER10S			10000
enum STATEMACHINE{
	system_init	= 0,
	system_stop = 1,
	system_preopen,
	system_run,
	system_alarm
};

enum PWMSTATE{
	pwm_off = 0,
	pwm_on = 1
};

typedef struct{
	enum STATEMACHINE MachineState;
	enum PWMSTATE PwmState;
	Uint16 PwmPreopen;
	Uint16 Controlmode;


}SystemStateMode;

typedef struct{
	Uint16 Timer1msIndex;
	Uint16 Timer20msIndex;
	Uint16 Timer1SIndex;
	Uint16 StateMachineIndex;
}SystemTimerMode;

typedef struct{
	SystemStateMode State;
	SystemTimerMode Timer;
}SystemParameterMode;

typedef struct{
	_iq Iu;
	_iq Iv;
	_iq Iw;
	_iq Vbus;
	_iq TempHeatsink;
	_iq TempIgbt;
	_iq TempMotor;

}ADValueMode;


typedef struct{
	ADValueMode AdSum;
	ADValueMode AdOffset;
	Uint16 Counter;
	Uint16 Finish;
}ADCalibrationMode;

typedef struct{
	Uint16 Year;
	Uint16 Month_Date;
	Uint16 hour_Min;
}VersionMode;
extern Uint16 start_flag;
extern volatile SystemParameterMode SystemParameter;
extern volatile ADValueMode ADValue;
extern volatile ADCalibrationMode ADCalibration;

inline void SetStateMachine(enum STATEMACHINE state)
{
	SystemParameter.Timer.StateMachineIndex	= 0;
	SystemParameter.State.MachineState	= state;
}


inline void DisablePwm(void)   {
	EALLOW;
//	EPwm1Regs.TZFRC.bit.OST		= 1;
//	EPwm2Regs.TZFRC.bit.OST		= 1;
//	EPwm3Regs.TZFRC.bit.OST		= 1;
//	GpioDataRegs.GPASET.bit.GPIO24	= 1;
//    GpioDataRegs.GPACLEAR.bit.GPIO10=1;
//	start_flag=0;
	EDIS;
}

inline void EnablePwm(void)    {
	EALLOW;
	EPwm1Regs.TZCLR.bit.OST		= 1;
	EPwm2Regs.TZCLR.bit.OST		= 1;
	EPwm3Regs.TZCLR.bit.OST		= 1;
	GpioDataRegs.GPACLEAR.bit.GPIO24		= 1;
    GpioDataRegs.GPASET.bit.GPIO10=1;
	start_flag=1;
	EDIS;
}

void Parameter_Init(void);
void Timer2ms_Fun(void);
void Timer20ms_Fun(void);
void StateMachine_Fun(void);
Uint16 CurrentOverRead_Fun(void);
void FaultProtection_Fun(void);

interrupt void cpu_timer0_isr(void);
interrupt void adc_svpwm_isr(void);
interrupt void adc_svpwm_isr1(void);
//void SVGENDQ(SVGEN svgen1);

#endif /* SOURCE_APPLICATION_H_ */
