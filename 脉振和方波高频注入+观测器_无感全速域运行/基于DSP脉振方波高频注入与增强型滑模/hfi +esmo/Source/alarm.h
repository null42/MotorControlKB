/*
 * alarm.h
 *
 *  Created on: 2 Apr 2018
 *      Author: LMD
 */

#ifndef SOURCE_ALARM_H_
#define SOURCE_ALARM_H_


#define  I_ERROR_RST()      	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1
#define  I_ERROR_RECOVER()  	GpioDataRegs.GPASET.bit.GPIO19 = 1


typedef struct{
	Uint16 OverOfIin;
	Uint16 OverOfVin;
	Uint16 OverOfDc;
	Uint16 UnderOfVin;
	Uint16 UnderOfDc;
	Uint16 OverOfTigbt;
}ALARMTIMERMODE;


typedef struct{
	Uint16 IoutOverIns   	: 1; //0 bit
	Uint16 IoutOverHard  	: 1; //1 bit
	Uint16 TigbtOver  		: 1;   //2 bit
	Uint16 TboardOver 		: 1;   //3 bit
	Uint16 calibration    	: 1; //4 bit
//	Uint16 Tigbt_Over_RMS	: 1; //5 bit
//	Uint16 Vin_Over_INS   	: 1; //6 bit
//	Uint16 Vin_Under_INS  	: 1; //7 bit
//	Uint16 Dc_Over_INS  	: 1;   //8 bit
//	Uint16 Dc_Under_INS 	: 1;   //9 bit
//	Uint16 Iin_Over_INS   	: 1; //10bit
//	Uint16 Tigbt_Over_INS 	: 1; //11bit
	Uint16 Resverved      	: 11; //4~14bit
	Uint16 Fault_Total    	: 1; //15bit
}ERRORBITMODE;


typedef struct{
	Uint16 Dc_Over_INS   	: 1; //0 bit
	Uint16 Dc_Under_INS  	: 1; //1 bit
	Uint16 Communication  	: 1;   //2 bit
//	Uint16 Dc_Under_RMS 	: 1;   //3 bit
//	Uint16 Iin_Over_RMS   	: 1; //4 bit
//	Uint16 Tigbt_Over_RMS	: 1; //5 bit
//	Uint16 Vin_Over_INS   	: 1; //6 bit
//	Uint16 Vin_Under_INS  	: 1; //7 bit
//	Uint16 Dc_Over_INS  	: 1;   //8 bit
//	Uint16 Dc_Under_INS 	: 1;   //9 bit
//	Uint16 Iin_Over_INS   	: 1; //10bit
//	Uint16 Tigbt_Over_INS 	: 1; //11bit
	Uint16 Resverved      	: 13; //3~15bit
}WARNINGBITMODE;


union FAULT_ERROR{
	Uint16 all;
	ERRORBITMODE bit;
};


union FAULT_WARNING{
	Uint16 all;
	WARNINGBITMODE bit;
};


typedef struct{
	union FAULT_ERROR error;
	union FAULT_WARNING warning;
}FAULTINTMODE;


union FAULTMODE{
	Uint32 all;
	FAULTINTMODE  type;
};


extern volatile union FAULTMODE Fault;
extern volatile ALARMTIMERMODE AlarmTimer;



#endif /* SOURCE_ALARM_H_ */
