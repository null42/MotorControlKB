/*
 * application.c
 *
 *  Created on: 27 Mar 2018
 *      Author: LMD
 */
#include "DSP28x_Project.h"
#include "device.h"
#include "application.h"
#include "led.h"
#include "alarm.h"
#include "control.h"
#include "cancomm.h"
#include "esmohfi.h"
//#include "voliate.h"
#define I_CHOKE_OFFSET				2048
#define MAX_SIG_DEVIATION 			100

volatile SystemParameterMode SystemParameter;
volatile ADValueMode ADValue;
volatile ADCalibrationMode ADCalibration;
static VersionMode Version;
const char SW_Versions[13] = VER;
Uint8 Test[10];
Uint8 Test1[10];
Uint8 Index = 0,Timer = 0,START=0;
int16 CAN_count=200,CAN_enable=1,can_count=0;
int16 can_start_flag=0,can_start_flag1=0;
extern Uint16 lsw;
//extern ESMOPOS  esmo1;
//extern RMPCNTL rc1;
extern _iq T;
extern PI_CONTROLLER pi_spd;
extern PI_CONTROLLER pi_id;
extern PI_CONTROLLER pi_iq;
extern HFI hfi1;
extern RMPCNTL rc1;
extern ESMOPOS  esmo1;

//Uint16 START=0;
void StateMachine_Fun(void);
static Uint16 CurrentOffset_Cal(void);
static void ParametricUpdate(void);
void Parametricdownload(void);
void Version_Fun(void);

inline Uint16 MotorDriverStartup(void)
{
//	return ((ModbusParameter.PublicReg[0])&(0x0001));
    return (START)&(0x0001);
}

void Parameter_Init(void)
{
	int i;

	for(i=0;i<sizeof(SystemParameter);i++)
	{
		((char*)&SystemParameter)[i]		= 0;
	}
	for(i=0;i<sizeof(ADValue);i++)
	{
		((char*)&ADValue)[i]				= 0;
	}
	for(i=0;i<sizeof(ADCalibration);i++)
	{
		((char*)&ADCalibration)[i]			= 0;
	}


	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO26			= 0;
	GpioCtrlRegs.GPADIR.bit.GPIO26			= 1;
	EDIS;
	GpioDataRegs.GPADAT.bit.GPIO26			= 1;

	ModbusParameter_Init();
	ModbusParameter.ModbusCallBack			= &Parametricdownload;
	ECanaParameter.ReceiveCallBack          = &Parametricdownload;

	ControlParameter_Init();
	LedInit();
	Version_Fun();
}


void Timer2ms_Fun(void)
{
	if(2 <= SystemParameter.Timer.Timer1msIndex)
	{
		SystemParameter.Timer.Timer1msIndex	= 0;

		ParametricUpdate();

		LedCtrl(SystemParameter.State.MachineState,&LedParameter,5);

		ModbusTask_Fun();

//		ECANRxProtocol_fun();

/*		if(1 == ECANATxDataFromMailBox0(&ECanaParameter.TxFrame[ECanaParameter.TxFrameIndex]))
		{
		    if(++ECanaParameter.TxFrameIndex >= TXFRAMEINCDEMAX)
		    {
		        ECanaParameter.TxFrameIndex         = 0;
		    }
		}*/

		//===================Test=====================//

	}

}


void Timer20ms_Fun(void)
{
	if(TIMER20MS <= SystemParameter.Timer.Timer20msIndex)
	{
		SystemParameter.Timer.Timer20msIndex	= 0;
/*==================================================================*/
		ADValue.TempHeatsink			= AdcResult.ADCRESULT12;
		ADValue.TempIgbt				= AdcResult.ADCRESULT13;
		ADValue.TempMotor				= AdcResult.ADCRESULT14;


		AdcRegs.ADCSOCFRC1.bit.SOC12	= 1;
		AdcRegs.ADCSOCFRC1.bit.SOC13	= 1;
		AdcRegs.ADCSOCFRC1.bit.SOC14	= 1;

        ECanaMboxes.MBOX0.MDL.all = Fault.all;
        ECanaMboxes.MBOX0.MDH.all = lsw;
        ECanaRegs.CANTRS.all = 0;
        ECanaRegs.CANTRS.bit.TRS0=1;
//        ECanbRegs.CANTRS.bit.TRS3=1;
//        ECanaRegs.CANTRS.bit.TRS5=1;
            // if(CAN_enable==1)
            //  { while(ECanbRegs.CANTA.all==0){} ; }// Wait for TA0 bit to be set..
                while(CAN_count--);
                if(CAN_count==-1)
                                            {
                                            CAN_count=200;
                                            if(ECanaRegs.CANTA.bit.TA0==0)
                                            {
                                                CAN_enable=0;

                                            }
                                            else
                                                CAN_enable=1;
                                            }

//                ECanbRegs.CANTA.all = 0x00000001;
                ECanaRegs.CANTA.bit.TA0=1;
//                ECanbRegs.CANTA.bit.TA3=1;
//                ECanaRegs.CANTA.bit.TA5=1;

                if(CAN_enable==0)
                {
                    if(can_count++>=5)
                    {
                        can_count=0;
                        PieCtrlRegs.PIEIER9.bit.INTx7 = 0;
                        ECanconfiger();
                        PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
                    }
                }
                else
                    can_count=0;

                if((can_start_flag1--)<1)
                //if((can_start_flag1)<1)
                {
                    can_start_flag=0;
                    can_start_flag1=0;
                    //canTicker2=0;
                }
/*==================================================================*/
	}

}


#pragma CODE_SECTION(StateMachine_Fun,"ramfuncs");
void StateMachine_Fun(void)
{
	if((system_init != SystemParameter.State.MachineState)&&(Fault.all != 0))
	{
		SetStateMachine(system_alarm);
	}

	switch(SystemParameter.State.MachineState)
	{
		case system_init:
		{
		    lsw=0;
		    DisablePwm();
			Fault.all									= 0;
			SystemParameter.State.PwmState				= pwm_off;
			SystemParameter.State.PwmPreopen			= 0;
			SystemParameter.Timer.StateMachineIndex++;
			lsw=0;

			if(20 >= SystemParameter.Timer.StateMachineIndex)
			{
				I_ERROR_RST();
			}
			else
			{
				I_ERROR_RECOVER();
			}
			if((1 == CurrentOffset_Cal())\
					&& (SystemParameter.Timer.StateMachineIndex >= TIMER100MS))
			{
				SetStateMachine(system_stop);
			}
		}break;
		case system_stop:
		{
			DisablePwm();
			SystemParameter.State.PwmState		= pwm_off;
			SystemParameter.State.PwmPreopen	= 0;
			SystemParameter.Timer.StateMachineIndex++;
            lsw=0;

			if(1 == MotorDriverStartup()&&(SystemParameter.Timer.StateMachineIndex >= TIMER200MS))
          //  if(1 == START&&(SystemParameter.Timer.StateMachineIndex >= TIMER200MS))
			{
				SetStateMachine(system_preopen);
				lsw=0;
			//	START=1;
	            RESET_PI(pi_id);
	            RESET_PI(pi_iq);
			}

		}break;
		case system_preopen:
		{
			SystemParameter.Timer.StateMachineIndex++;
			SystemParameter.State.PwmState		= pwm_on;
			EnablePwm();
			if(SystemParameter.Timer.StateMachineIndex < TIMER1S)
			{
				SystemParameter.State.PwmPreopen	= 1;

			}
			else
			{
				SystemParameter.State.PwmPreopen    = 0;
				SvpwmParameter.Vd				    = 0;
				SvpwmParameter.Vq				    = 0;
				anxid_reg.Ref                       = 0;
				anxiq_reg.Ref                       = 0;
		        anxid_reg.Ui64                      = 0;
		        anxid_reg.Out                       = 0;
		        anxiq_reg.Ui64                      = 0;
		        anxiq_reg.Out                       = 0;

		        SensorlessVars.IdsOld               = 0;
		        SensorlessVars.IqsOld               = 0;
		        SensorlessVars.ErrIdlpf             = 0;
		        SensorlessVars.ErrIqlpf             = 0;
		        SensorlessVars.EmfEst               = 0;
		        SensorlessVars.WsEstlpf             = 0;
		        SensorlessVars.Theta                = 0;
		        SensorlessVars.ThetaOut             = 0;
		        SensorlessVars.VdOld                = 0;
		        SensorlessVars.VqOld                = 0;

	            RESET_PI(pi_id);
	            RESET_PI(pi_iq);
	            pi_iq.Ref = _IQ(0.0);
	            RESET_PI(pi_spd);
	            hfi1.Squ_PRD_set  = 8;//INDEX_CNT;        /* 16 IPD // 8 low speed for Bosch // 1 for E-bike *///16
	            hfi1.dutyMax  = _IQ(0.8);


	            rc1.SetpointValue=_IQ(0.000);   //0.002
	            esmo1.Kslide   =_IQ(0.02508703613);//_IQ(0.00508703613);// _IQ(0.05308703613);
	            esmo1.Valpha=_IQ(0.0);esmo1.Vbeta=_IQ(0.0);esmo1.Zalpha=_IQ(0.0);esmo1.EstIalpha=_IQ(0.0);esmo1.IalphaError=_IQ(0.0);esmo1.Theta=_IQ(0.0);
	            esmo1.Ealpha=_IQ(0.0);esmo1.Ebeta=_IQ(0.0);esmo1.Zbeta=_IQ(0.0);esmo1.EstIbeta=_IQ(0.0);esmo1.IbetaError=_IQ(0.0);

			}
			if(SystemParameter.Timer.StateMachineIndex > TIMER1S)
			{
				SetStateMachine(system_run);
				lsw=1;
			}
			if(0 == MotorDriverStartup())
//			if(0 == START)
			{
				SetStateMachine(system_stop);
				lsw=0;
			//	START=0;
			}

		}break;
		case system_run:
		{
			SystemParameter.State.PwmState		= pwm_on;
			EnablePwm();
			if(0 == MotorDriverStartup())
        //    if(0 == START)
            {
                SetStateMachine(system_stop);
                lsw=0;
             //   START=0;
			}
		}break;
		case system_alarm:
		{
			DisablePwm();
			SystemParameter.State.PwmState		= pwm_off;
            lsw=0;
			ModbusParameter.PublicReg[0]				= 0;

			if(Fault.all == 0)
			{
				SetStateMachine(system_init);
			}

		}break;
		default :
		{
			DisablePwm();
			Fault.all					= 0xFFFFFFFF;
		}break;
	}
}


#pragma CODE_SECTION(ParametricUpdate,"ramfuncs");
static void ParametricUpdate(void)
{
/*
	ModbusParameter.PublicReg[1]	= (FocIoutParameter.Vd >> 14) & 0x0000ffff;
	ModbusParameter.PublicReg[2]	= (FocIoutParameter.Vq >> 14) & 0x0000ffff;
	ModbusParameter.PublicReg[3]	= (SvpwmParameter.Vd >> 10) & 0x0000ffff;
	ModbusParameter.PublicReg[4]	= (SvpwmParameter.Vq >> 10) & 0x0000ffff;

*/
	ModbusParameter.PublicReg[16]	= Version.Year;
	ModbusParameter.PublicReg[17]	= Version.Month_Date;
	ModbusParameter.PublicReg[18]	= Version.hour_Min;
	ModbusParameter.PublicReg[19]	= 0;
	ModbusParameter.PublicReg[20]	= (SensorlessVars.WsEstlpf >> 9) & 0x0000ffff;
	ModbusParameter.PublicReg[21]	= (FocIoutParameter.Vd >> 14) & 0x0000ffff;
	ModbusParameter.PublicReg[22]	= (FocIoutParameter.Vq >> 14) & 0x0000ffff;
	ModbusParameter.PublicReg[23]	= (SvpwmParameter.Vd >> 10) & 0x0000ffff;
	ModbusParameter.PublicReg[24]	= (SvpwmParameter.Vq >> 10) & 0x0000ffff;
	ModbusParameter.PublicReg[25]	= (SvpwmParameter.Vbus>>10) & 0x0000ffff;

	ModbusParameter.PublicReg[28]	= SystemParameter.State.PwmState;

	ModbusParameter.PublicReg[29]	= Fault.type.error.all;
	ModbusParameter.PublicReg[30]	= Fault.type.warning.all;
}


#pragma CODE_SECTION(Parametricdownload,"ramfuncs");
void Parametricdownload(void)
{
#if COMM == SCI
	SvpwmParameter.Vd						= ((Uint32)ModbusParameter.PublicReg[1] << 10);
	SvpwmParameter.Vq						= ((Uint32)ModbusParameter.PublicReg[2] << 10);
	anxid_reg.Ref							= ((Uint32)ModbusParameter.PublicReg[3] << 14);
	anxiq_reg.Ref							= ((Uint32)ModbusParameter.PublicReg[4] << 14);
	spd_reg.Ref 							= ((Uint32)ModbusParameter.PublicReg[5] << 10);
	SystemParameter.State.Controlmode       = ModbusParameter.PublicReg[6];
//#else if COMM == CAN
//	if(ECanaParameter.RxMessageID == 0x12345678)
//	{
//
//	    SvpwmParameter.Vd                   = ECanaParameter.RxDataBuf[0] + (ECanaParameter.RxDataBuf[1] << 8);
//	    SvpwmParameter.Vq                   = ECanaParameter.RxDataBuf[2] + (ECanaParameter.RxDataBuf[3] << 8);
//	    anxid_reg.Ref                       = ECanaParameter.RxDataBuf[4] + (ECanaParameter.RxDataBuf[5] << 8);
//	    anxid_reg.Ref                       = ECanaParameter.RxDataBuf[6] + (ECanaParameter.RxDataBuf[7] << 8);
//	    spd_reg.Ref                         = ECanaParameter.RxDataBuf[8] + (ECanaParameter.RxDataBuf[9] << 8);
//	}
//

#endif

}


static Uint16 CurrentOffset_Cal(void)
{
	Uint16 Temp = 0;

	ADCalibration.Counter++;
	if(ADCalibration.Counter <= 100)
	{
		ADCalibration.AdSum.Iu 					+= ADValue.Iu;
		ADCalibration.AdSum.Iv	 				+= ADValue.Iv;
		ADCalibration.AdSum.Iw 					+= ADValue.Iw;
	}
	else if(ADCalibration.Counter == 101)
	{
		ADCalibration.AdOffset.Iu				= ADCalibration.AdSum.Iu / 100;
		if((ADCalibration.AdOffset.Iu > (I_CHOKE_OFFSET + MAX_SIG_DEVIATION))\
				||(ADCalibration.AdOffset.Iu < (I_CHOKE_OFFSET - MAX_SIG_DEVIATION)))
		{
		//	ADCalibration.AdOffset.Iu			= I_CHOKE_OFFSET;
			Fault.type.error.bit.calibration	= 1;
		}

		ADCalibration.AdOffset.Iv				= ADCalibration.AdSum.Iv / 100;
		if((ADCalibration.AdOffset.Iv > (I_CHOKE_OFFSET + MAX_SIG_DEVIATION))\
				||(ADCalibration.AdOffset.Iv < (I_CHOKE_OFFSET - MAX_SIG_DEVIATION)))
		{
	//		ADCalibration.AdOffset.Iv			= I_CHOKE_OFFSET;
			Fault.type.error.bit.calibration	= 1;
		}

		ADCalibration.AdOffset.Iw				= ADCalibration.AdSum.Iw / 100;
		if((ADCalibration.AdOffset.Iw > (I_CHOKE_OFFSET + MAX_SIG_DEVIATION))\
				||(ADCalibration.AdOffset.Iw < (I_CHOKE_OFFSET - MAX_SIG_DEVIATION)))
		{
	//		ADCalibration.AdOffset.Iw			= I_CHOKE_OFFSET;
			Fault.type.error.bit.calibration	= 1;
		}

	}
	else if(ADCalibration.Counter > 101)
	{
		Temp									= 1;
		ADCalibration.AdSum.Iu					= 0;
		ADCalibration.AdSum.Iv					= 0;
		ADCalibration.AdSum.Iw					= 0;
		ADCalibration.Counter					= 0;
	}

	return Temp;
}


#pragma CODE_SECTION(FaultProtection_Fun,"ramfuncs");
void FaultProtection_Fun(void)
{
	//输入欠压故障
	if(SvpwmParameter.Vbus < _IQ(200.0/512))
	{
		AlarmTimer.UnderOfDc++;
		if(AlarmTimer.UnderOfDc > 5)
		{
//			Fault.type.warning.bit.Dc_Under_INS		= 1;
		}
	}else{
		AlarmTimer.UnderOfDc 						= 0;
		Fault.type.warning.bit.Dc_Under_INS			= 0;
	}
	//温度保护


}


Uint16 CurrentOverRead_Fun(void)
{
	Uint16 TempU = 0, TempV = 0, TempW = 0;
	TempU							= (0 == GpioDataRegs.GPADAT.bit.GPIO12)? 1 : 0;
	TempV							= (0 == GpioDataRegs.GPADAT.bit.GPIO12)? 2 : 0;
	TempW							= (0 == GpioDataRegs.GPADAT.bit.GPIO12)? 4 : 0;//16 17 18
	TempU							= TempU + TempV + TempW;
	if(0 != TempU)
	{
		Fault.type.error.bit.IoutOverHard	= 1;
	}
	return (TempU);

}

void Version_Fun(void)
{
	Version.Year		= (SW_Versions[0] - 48)*1000 + (SW_Versions[1] - 48)*100\
							+ (SW_Versions[2] - 48)*10 + (SW_Versions[3] - 48);

	Version.Month_Date	= ((SW_Versions[4] - 48)*10 + (SW_Versions[5] - 48))*256\
							+ (SW_Versions[6] - 48)*10 + (SW_Versions[7] - 48);

	Version.hour_Min	= ((SW_Versions[9] - 48)*10 + (SW_Versions[10] - 48))*256\
							+ (SW_Versions[11] - 48)*10 + (SW_Versions[12] - 48);
}





