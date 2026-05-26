
#include "DSP28x_Project.h"
#include "application.h"
#include "device.h"

Uint32 EPwm1TimerIntCount = 0;
#define CAN_BUS_SPEED_250KBPS



#define DEADTIME			120

void EpwmReg_Init(void)
{
	DisablePwm();
	EPwm1Regs.TBPRD 					= SVPWMTBPRD;                        // Set timer period
	EPwm1Regs.TBPHS.half.TBPHS 			= 0x0000;           // Phase is 0
	EPwm1Regs.TBCTR 					= 0x0000;                      // Clear counter
	// Setup TBCLK
/*	EPwm1Regs.TBCTL.bit.CTRMODE 		= TB_COUNT_UPDOWN; // Count up
	EPwm1Regs.TBCTL.bit.PHSEN 			= TB_DISABLE;        // Disable phase loading
	EPwm1Regs.TBCTL.bit.SYNCOSEL		= TB_CTR_ZERO;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV 		= TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV 			= TB_DIV1;*/

	EPwm1Regs.CMPCTL.bit.SHDWAMODE 		= CC_SHADOW;    // Load registers every ZERO
	EPwm1Regs.CMPCTL.bit.SHDWBMODE 		= CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE 		= CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE		= CC_CTR_ZERO;
	// Setup compare
	EPwm1Regs.CMPA.half.CMPA			= SVPWMTBPRD/2;

	EPwm1Regs.AQCTLA.bit.CAU			= AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.CAD			= AQ_SET;

	EPwm1Regs.DBCTL.bit.OUT_MODE 		= DB_FULL_ENABLE;
	EPwm1Regs.DBCTL.bit.POLSEL 			= DB_ACTV_HIC;
//    EPwm1Regs.DBCTL.bit.POLSEL          = DB_ACTV_LOC;

	EPwm1Regs.DBCTL.bit.IN_MODE 		= DBA_ALL;
	EPwm1Regs.DBRED						= DEADTIME;
	EPwm1Regs.DBFED 					= DEADTIME;
/*
    EPwm1Regs.DBCTL.bit.OUT_MODE        = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL          = DB_ACTV_LOC;
    EPwm1Regs.DBCTL.bit.IN_MODE         = DBA_ALL;
    EPwm1Regs.DBRED                     = DEADTIME;
    EPwm1Regs.DBFED                     = DEADTIME;
*/

	EALLOW;
	EPwm1Regs.TZCTL.bit.TZB 			= TZ_HIZ;       ///确定拉低还是拉高 否则会短路。
	EPwm1Regs.TZCTL.bit.TZA				= TZ_HIZ;;
	EDIS;

	EPwm2Regs.TBPRD 					= SVPWMTBPRD;                        // Set timer period
	EPwm2Regs.TBPHS.half.TBPHS 			= 0x0000;           // Phase is 0
	EPwm2Regs.TBCTR 					= 0x0000;                      // Clear counter
	// Setup TBCLK
/*	EPwm2Regs.TBCTL.bit.CTRMODE 		= TB_COUNT_UPDOWN; // Count up
	EPwm2Regs.TBCTL.bit.PHSEN 			= TB_ENABLE;        // Disable phase loading
	EPwm2Regs.TBCTL.bit.SYNCOSEL		= TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSDIR			= TB_UP;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV 		= TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV 			= TB_DIV1;*/
	EPwm1Regs.TBCTL.all=0xA00A;
    EPwm2Regs.TBCTL.all=0xA00A;
    EPwm3Regs.TBCTL.all=0xA00A;

	EPwm2Regs.CMPCTL.bit.SHDWAMODE 		= CC_SHADOW;    // Load registers every ZERO
	EPwm2Regs.CMPCTL.bit.SHDWBMODE 		= CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE 		= CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE		= CC_CTR_ZERO;
	// Setup compare
	EPwm2Regs.CMPA.half.CMPA			= SVPWMTBPRD/2;

	EPwm2Regs.AQCTLA.bit.CAU			= AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.CAD			= AQ_SET;

	EPwm2Regs.DBCTL.bit.OUT_MODE 		= DB_FULL_ENABLE;
	EPwm2Regs.DBCTL.bit.POLSEL 			= DB_ACTV_HIC;
//    EPwm2Regs.DBCTL.bit.POLSEL          = DB_ACTV_LOC;
	EPwm2Regs.DBCTL.bit.IN_MODE 		= DBA_ALL;
	EPwm2Regs.DBRED						= DEADTIME;
	EPwm2Regs.DBFED 					= DEADTIME;

	EALLOW;
	EPwm2Regs.TZCTL.bit.TZB 			= TZ_HIZ;       ///确定拉低还是拉高 否则会短路。
	EPwm2Regs.TZCTL.bit.TZA				= TZ_HIZ;
	EDIS;

	EPwm3Regs.TBPRD 					= SVPWMTBPRD;                        // Set timer period
	EPwm3Regs.TBPHS.half.TBPHS 			= 0x0000;           // Phase is 0
	EPwm3Regs.TBCTR 					= 0x0000;                      // Clear counter
	// Setup TBCLK
/*	EPwm3Regs.TBCTL.bit.CTRMODE 		= TB_COUNT_UPDOWN; // Count up
	EPwm3Regs.TBCTL.bit.PHSEN 			= TB_ENABLE;        // Disable phase loading
	EPwm3Regs.TBCTL.bit.SYNCOSEL		= TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSDIR			= TB_UP;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV 		= TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV 			= TB_DIV1;*/

	EPwm3Regs.CMPCTL.bit.SHDWAMODE 		= CC_SHADOW;    // Load registers every ZERO
	EPwm3Regs.CMPCTL.bit.SHDWBMODE 		= CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE 		= CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE		= CC_CTR_ZERO;
	// Setup compare
	EPwm3Regs.CMPA.half.CMPA			= SVPWMTBPRD/2;

	EPwm3Regs.AQCTLA.bit.CAU			= AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.CAD			= AQ_SET;

	EPwm3Regs.DBCTL.bit.OUT_MODE 		= DB_FULL_ENABLE;
	EPwm3Regs.DBCTL.bit.POLSEL 			= DB_ACTV_HIC;
//    EPwm3Regs.DBCTL.bit.POLSEL          = DB_ACTV_LOC;
	EPwm3Regs.DBCTL.bit.IN_MODE 		= DBA_ALL;
	EPwm3Regs.DBRED						= DEADTIME;
	EPwm3Regs.DBFED 					= DEADTIME;

	EALLOW;
	EPwm3Regs.TZCTL.bit.TZB 			= TZ_HIZ;       ///确定拉低还是拉高 否则会短路。
	EPwm3Regs.TZCTL.bit.TZA				= TZ_HIZ;
	EDIS;

	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO24		= 0;
	GpioCtrlRegs.GPADIR.bit.GPIO24		= 1;
	EDIS;

}

void GpioRegs_Init(void)
{

	EALLOW;
	/*I_ERROR_RST*/
	GpioCtrlRegs.GPAMUX2.bit.GPIO19		= 0;
	GpioCtrlRegs.GPADIR.bit.GPIO19		= 1;

	/*SW_FAN*/
	GpioCtrlRegs.GPBMUX1.bit.GPIO43		= 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO43		= 1;

	//   // Enable an GPIO output on GPIO6, set it high
	//   GpioCtrlRegs.GPAPUD.bit.GPIO6	= 0;   // Enable pullup on GPIO6
	//   GpioDataRegs.GPASET.bit.GPIO6 	= 1;   // Load output latch
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO6 	= 0;  // GPIO6 = GPIO6
	//   GpioCtrlRegs.GPADIR.bit.GPIO6 	= 1;   // GPIO6 = output
	//   // Enable eCAP2 on GPIO7
	//   GpioCtrlRegs.GPAPUD.bit.GPIO7 	= 1;   // disable pullup on GPIO7
	////   GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0; // Synch to SYSCLOUT
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO7 	= 1;  // GPIO7 = EPWM4B
	//   // Enable GPIO outputs on GPIO8 - GPIO11, set it high
	//   GpioCtrlRegs.GPAPUD.bit.GPIO8 	= 0;   // Enable pullup on GPIO8
	//   GpioDataRegs.GPASET.bit.GPIO8 	= 1;   // Load output latch
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO8 	= 0;  // GPIO8 = GPIO8
	//   GpioCtrlRegs.GPADIR.bit.GPIO8 	= 1;   // GPIO8 = output
	//
	//   GpioCtrlRegs.GPAPUD.bit.GPIO9 	= 0;   // Enable pullup on GPIO9
	//   GpioDataRegs.GPASET.bit.GPIO9 	= 1;   // Load output latch
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO9 	= 0;  // GPIO9 = GPIO9
	//   GpioCtrlRegs.GPADIR.bit.GPIO9 	= 0;   // GPIO9 = input
	//
	//   GpioCtrlRegs.GPAPUD.bit.GPIO10 	= 0;  // Enable pullup on GPIO10
	//   GpioDataRegs.GPASET.bit.GPIO10 	= 1;  // Load output latch
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO10 	= 0; // GPIO10 = GPIO10
	//   GpioCtrlRegs.GPADIR.bit.GPIO10 	= 0;   // GPIO10 = input
    //   GpioCtrlRegs.GPAPUD.bit.GPIO10     = 0;  // Enable pullup on GPIO10
       GpioDataRegs.GPACLEAR.bit.GPIO10     = 1;  // Load output latch
       GpioCtrlRegs.GPAMUX1.bit.GPIO10    = 0; // GPIO10 = GPIO10
       GpioCtrlRegs.GPADIR.bit.GPIO10     = 1;   // GPIO10 = input
       GpioDataRegs.GPACLEAR.bit.GPIO10     = 1;  // Load output latch

       GpioDataRegs.GPACLEAR.bit.GPIO17     = 1;  // Load output latch
       GpioCtrlRegs.GPAMUX2.bit.GPIO17    = 0; // GPIO10 = GPIO10
       GpioCtrlRegs.GPADIR.bit.GPIO17     = 1;   // GPIO10 = input
       GpioDataRegs.GPACLEAR.bit.GPIO17     = 1;  // Load output latch*/
       /*
	//
	//   GpioCtrlRegs.GPAPUD.bit.GPIO11 	= 0;  // Enable pullup on GPIO11
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO11	= 0; // GPIO11 = GPIO11
	//   GpioCtrlRegs.GPADIR.bit.GPIO11 	= 0;  // GPIO11 = input
	//   // Enable Trip Zone inputs on GPIO12 - GPIO15
	//   GpioCtrlRegs.GPAPUD.bit.GPIO12 	= 0;   // Enable pullup on GPIO12
	//   GpioCtrlRegs.GPAPUD.bit.GPIO14 	= 0;   // Enable pullup on GPIO14
	//   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3; // asynch input
	//   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3; // asynch input
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO12 	= 0;  // GPIO12 = TZ1
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO14 	= 0;  // GPIO14 = TZ3
	//
	//   // Enable SPI-A on GPIO16 - GPIO19
	//   GpioCtrlRegs.GPAPUD.bit.GPIO16 	= 0;   // Enable pullup on GPIO16
	//   GpioCtrlRegs.GPAPUD.bit.GPIO17	= 0;   // Enable pullup on GPIO17
	//   GpioCtrlRegs.GPAPUD.bit.GPIO18	= 0;   // Enable pullup on GPIO18
	//
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // asynch input
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // asynch input
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // asynch input
	////   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // asynch input
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO16	= 1;  // GPIO16 = SPISIMOA
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO17 	= 1;  // GPIO17 = SPIS0MIA
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO18 	= 1;  // GPIO18 = SPICLKA
	//
	//   GpioCtrlRegs.GPAPUD.bit.GPIO19 	= 0;   // Enable pullup on GPIO19
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO19 	= 0;  // GPIO19 = LED
	//   GpioCtrlRegs.GPADIR.bit.GPIO19 	= 1;   // GPIO22 = output
	//   // Enable EQEP1 on GPIO20 - GPIO23
	//   GpioCtrlRegs.GPAPUD.bit.GPIO20 	= 0;   // Enable pullup on GPIO20
	//   GpioCtrlRegs.GPAPUD.bit.GPIO21 	= 0;   // Enable pullup on GPIO21
	//   GpioCtrlRegs.GPAPUD.bit.GPIO23 	= 0;   // Enable pullup on GPIO23
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Synch to SYSCLKOUT
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0; // Synch to SYSCLKOUT
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0; // Synch to SYSCLKOUT
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO20 	= 0;  // GPIO20 = EQEP1A
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO21 	= 0;  // GPIO21 = EQEP1B
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO23 	= 0;  // GPIO23 = EQEP1I
	//   GpioCtrlRegs.GPAPUD.bit.GPIO22 	= 0;   // Enable pullup on GPIO22
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO22 	= 0;  // GPIO22 = PTC_ON
	//   GpioCtrlRegs.GPADIR.bit.GPIO22 	= 1;   // GPIO22 = output
	//   // Enable eCAP1 on GPIO24
	//   GpioCtrlRegs.GPAPUD.bit.GPIO24 	= 0;   // Enable pullup on GPIO24
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0; // Synch to SYSCLKOUT
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO24 	= 0;  // GPIO24 = ECAP1
	//   // Set input qualifcation period for GPIO25 & GPIO26
	//   GpioCtrlRegs.GPACTRL.bit.QUALPRD3=1;  // Qual period = SYSCLKOUT/2
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO25	=2;   // 6 samples
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO26	=2;   // 6 samples
	//   // Make GPIO25 the input source for XINT1
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO25 	= 0;  // GPIO25 = GPIO25
	//   GpioCtrlRegs.GPADIR.bit.GPIO25 	= 0;   // GPIO25 = input
	//   GpioIntRegs.GPIOXINT1SEL.all 	= 25;    // XINT1 connected to GPIO25
	//   // Make GPIO26 the input source for XINT2
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO26 	= 0;  // GPIO26 = GPIO26
	//   GpioCtrlRegs.GPADIR.bit.GPIO26 	= 0;   // GPIO26 = input
	//   GpioIntRegs.GPIOXINT2SEL.all 	= 26;    // XINT2 connected to GPIO26
	//   // Make GPIO27 wakeup from HALT/STANDBY Low Power Modes
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO27 	= 0; // GPIO27 = GPIO27
	//   GpioCtrlRegs.GPADIR.bit.GPIO27	= 0;  // GPIO27 = input
	//   GpioIntRegs.GPIOLPMSEL.bit.GPIO27=1; // GPIO27 will wake the device
	//   SysCtrlRegs.LPMCR0.bit.QUALSTDBY	=2;  // Qualify GPIO27 by 2 OSCCLK
	//                                        // cycles before waking the device
	//                                        // from STANDBY
	//   // Enable SCI-A on GPIO28 - GPIO29
	//   GpioCtrlRegs.GPAPUD.bit.GPIO28 	= 0;   // Enable pullup on GPIO28
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Asynch input
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO28 	= 1;  // GPIO28 = SCIRXDA
	//   GpioCtrlRegs.GPAPUD.bit.GPIO29 	= 0;   // Enable pullup on GPIO29
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO29 	= 1;  // GPIO29 = SCITXDA
	//   // Enable CAN-A on GPIO30 - GPIO31
	//   GpioCtrlRegs.GPAPUD.bit.GPIO30 	= 0;   // Enable pullup on GPIO30
	//   GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3; // Asynch input
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO30 	= 0;  // GPIO30 = CANRXA
	//   GpioCtrlRegs.GPAPUD.bit.GPIO31 	= 0;   // Enable pullup on GPIO31
	//   GpioCtrlRegs.GPAMUX2.bit.GPIO31 	= 0;  // GPIO31 = CANTXA
	//  GPIO-30 - PIN FUNCTION = CANRX-A*/
	    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // 0=GPIO,  1=CANRX-A,  2=Resv,  3=Resv
	    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;     // 1=OUTput,  0=INput
	//  GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;   // uncomment if --> Set Low initially
	//  GpioDataRegs.GPASET.bit.GPIO30 = 1;     // uncomment if --> Set High initially
	//--------------------------------------------------------------------------------------
	//  GPIO-31 - PIN FUNCTION = CANTX-A
	    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // 0=GPIO,  1=CANTX-A,  2=Resv,  3=Resv
	    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // 1=OUTput,  0=INput
	//  GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // uncomment if --> Set Low initially
	//  GpioDataRegs.GPASET.bit.GPIO31 = 1;     // uncomment if --> Set High initially
	//--------------------------------------------------------------------------------------

	//   // Enable I2C-A on GPIO32 - GPIO33
	//   GpioCtrlRegs.GPBPUD.bit.GPIO32 	= 0;   // Enable pullup on GPIO32
	//   GpioCtrlRegs.GPBMUX1.bit.GPIO32 	= 0;  // GPIO32 = SDAA
	//   GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 0; // Asynch input
	//   GpioCtrlRegs.GPBPUD.bit.GPIO33 	= 0;   // Enable pullup on GPIO33
	//   GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 0; // Asynch input
	//   GpioCtrlRegs.GPBMUX1.bit.GPIO33 	= 0;  // GPIO33 = SCLA
	//   // Make GPIO34 an input
	//   GpioCtrlRegs.GPBPUD.bit.GPIO34 	= 0;  // Enable pullup on GPIO34
	//   GpioCtrlRegs.GPBMUX1.bit.GPIO34 	= 0; // GPIO34 = GPIO34
	//   GpioCtrlRegs.GPBDIR.bit.GPIO34 	= 0;  // GPIO34 = input
	//   GpioCtrlRegs.GPAPUD.bit.GPIO13 	= 0;   // Enable pullup on GPIO13
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO13 	= 0;  // GPIO13 = GPIO
	//   GpioCtrlRegs.GPADIR.bit.GPIO13 	= 1;  // GPIO13 = output
	//   GpioCtrlRegs.GPAPUD.bit.GPIO15 	= 0;   // Enable pullup on GPIO15
	//   GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3; // asynch input
	//   GpioCtrlRegs.GPAMUX1.bit.GPIO15 	= 2;  // GPIO15 = SCIRXDB

	EDIS;
}
void AdcRegs_Init(void)
{
	// Configure ADC
	EALLOW;
	AdcRegs.ADCCTL1.bit.TEMPCONV    = 0;    // 使能内部温度
	AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;	// Enable non-overlap mode
	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	// ADCINT1 trips after AdcResults latch
	AdcRegs.ADCINTSOCSEL1.all		= 0;
	AdcRegs.ADCINTSOCSEL2.all		= 0;

	AdcRegs.INTSEL1N2.bit.INT2E		= 1;	// Enabled ADCINT2
	AdcRegs.INTSEL1N2.bit.INT2CONT	= 0;
	AdcRegs.INTSEL1N2.bit.INT2SEL	= 3;	// setup EOC6 to trigger ADCINT2 to fire

	AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 0;    // IU
	AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 1;    // IV
	AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;	// IW
	AdcRegs.ADCSOC3CTL.bit.CHSEL    = 3;	// Vbus

	AdcRegs.ADCSOC12CTL.bit.CHSEL	= 4;		//散热片温度
	AdcRegs.ADCSOC13CTL.bit.CHSEL	= 5;		//IGBT温度
	AdcRegs.ADCSOC14CTL.bit.CHSEL	= 1;		//电机温度
/*
	  AdcRegs.ADCSOC0CTL.bit.CHSEL    = 1;    // IU
	    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 2;    // IV
	    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 3;    // IW
	    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 6;   // Vbus

	    AdcRegs.ADCSOC12CTL.bit.CHSEL   = 12;       //散热片温度
	    AdcRegs.ADCSOC13CTL.bit.CHSEL   = 4;       //IGBT温度
	    AdcRegs.ADCSOC14CTL.bit.CHSEL   = 7;        //电机温度
*/

	AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
	AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;

	AdcRegs.ADCSOC12CTL.bit.TRIGSEL  = 0;
	AdcRegs.ADCSOC13CTL.bit.TRIGSEL  = 0;
	AdcRegs.ADCSOC14CTL.bit.TRIGSEL  = 0;

	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;
	AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;

	AdcRegs.ADCSOC12CTL.bit.ACQPS 	= 6;	// set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC13CTL.bit.ACQPS    = 6;
	AdcRegs.ADCSOC14CTL.bit.ACQPS    = 6;
	EDIS;
	// Assumes ePWM4 clock is already enabled in InitSysCtrl();
	EPwm4Regs.ETSEL.bit.SOCBEN	= 1;		// Enable SOC on A group
	EPwm4Regs.ETSEL.bit.SOCBSEL	= 1;		// Select SOC from ZERO
	EPwm4Regs.ETPS.bit.SOCBPRD 	= 1;		// Generate pulse on 1st event

	EPwm1Regs.ETSEL.bit.SOCAEN	= 1;
	EPwm1Regs.ETSEL.bit.SOCASEL	= 1;
	EPwm1Regs.ETPS.bit.SOCAPRD	= 1;
}








