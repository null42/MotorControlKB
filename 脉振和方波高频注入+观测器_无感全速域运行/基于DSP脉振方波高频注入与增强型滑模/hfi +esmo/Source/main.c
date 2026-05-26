/*
 * main.c
 */
#include "DSP28x_Project.h"
#include "application.h"
#include "device.h"
#include "cancomm.h"
#include "esmohfi.h"
#include "sensorless-settings.h"
#include <math.h>

void ECanconfiger(void);
interrupt void ECAN0INTA_ISR(void);
//#include "voliate.h"
// Include header for HFI group of objects
// Instance a sliding-mode position observer constant Module
// Instance an eSMO position estimator
ESMOPOS  esmo1 = ESMOPOS_DEFAULTS;
SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;
SPEED_ESTIMATION speed3 = SPEED_ESTIMATION_DEFAULTS;
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

_iq  startCnt,               // cntr to help transition from lsw 1 --> 2
     smoK_set = _IQ(0.5);   // sliding mode gain - final value//0.5

  // Instance a transition regulator to work with various rotor angle estimates
  TRANSITION  transition1 = TRANSITION_DEFAULTS;

  // Instance a high pass filter
  HPF  hpf_Iq = HPF_DEFAULTS;

  // Instance a high pass filter coefficients structure
  HPF_COEFF   hpf_coeff1 = HPF_COEFF_DEFAULTS;

  // Instance a High Frequency Injection controller
  HFI hfi1 = HFI_DEFAULTS;

  // Instance a magnetic polarity identifier
  NS_ID  ns_id1 = NS_ID_DEFAULTS;

  float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h


  RAMPGEN rg1 = RAMPGEN_DEFAULTS;
  //rg1.StepAngleMax = _IQ(BASE_FREQ*T);
  // Instance a transition regulator to work with various rotor angle estimates

  CLARKE clarke1 = CLARKE_DEFAULTS;
  PARK park1 = PARK_DEFAULTS;
  IPARK ipark1 = IPARK_DEFAULTS;
  SVGEN svgen1 = SVGEN_DEFAULTS;
  PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;
  PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
  PI_CONTROLLER pi_id  = PI_CONTROLLER_DEFAULTS;
  PI_CONTROLLER pi_iq  = PI_CONTROLLER_DEFAULTS;
  DBC dbc1 = DBC_DEFAULTS;
  PWMGEN1 pwm1 = PWMGEN_DEFAULTS;
extern _iq  SpeedRef;
Uint32 Rec_l=0,Rec_h=0;
extern Uint8 START;
extern int16 can_start_flag,can_start_flag1;

void main(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2806x_SysCtrl.c file.
	InitSysCtrl();

	// Step 2. Initalize GPIO:
	// This example function is found in the F2806x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	// InitGpio();  // Skipped for this example
	InitGpio();
	InitAdcAio();
	GpioRegs_Init();

	Parameter_Init();
	ECanconfiger();
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();

	// Initialize PWM module
	    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1
	    pwm1.HalfPerMax=pwm1.PeriodMax/2;
	    pwm1.Deadband  = 2.0*SYSTEM_FREQUENCY;          // 120 counts -> 2.0 usec for TBCLK = SYSCLK/1
//	    PWM_INIT_MACRO(1,2,3,pwm1)

        rg1.StepAngleMax = _IQ(BASE_FREQ*T);

    // Initialize the SPEED_EST module SMOPOS based speed calculation
    speed3.K1 = _IQ21(1/(BASE_FREQ*T));
    speed3.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed3.K3 = _IQ(1)-speed3.K2;
    speed3.BaseRpm = 120*(BASE_FREQ/POLES);

    smo1_const.Rs = RS;
    smo1_const.Ls = LS;
    smo1_const.Ib = BASE_CURRENT;
    smo1_const.Vb = BASE_VOLTAGE;
    smo1_const.Ts = T;
    SMO_CONST_MACRO(smo1_const)

     // Initialize eSMO parameters
     esmo1.Fsmopos  = _IQ(smo1_const.Fsmopos);
     esmo1.Gsmopos  = _IQ(smo1_const.Gsmopos);
     esmo1.Kslide   = _IQ(0.03308703613);//0.05308703613
     esmo1.base_wTs = _IQ(100*T);//BASE_FREQ

     // Initialize HFI parameters
      // HFI magnitude (duty cycle) parameters
      hfi1.dutyMax  = _IQ(0.8);             //0.24 310V //_IQ(0.4) 180V  // IPD 0.3 squ 16 // 0.4 squ 8//0.65
      hfi1.volt_ipd = VOLT_PU(HFI_VOLT_IPD);
      hfi1.volt_run = VOLT_PU(HFI_VOLT_RUN);

      // HFI frequency (timing) parameters
      hfi1.Squ_PRD_set  = 8;//INDEX_CNT;        /* 16 IPD // 8 low speed for Bosch // 1 for E-bike *///16
      hfi1.HFI_Time1 = 300;//400
      hfi1.HFI_Time2 = 400;//500

      hfi1.base_wTs = _IQ(100*T);//BASE_FREQ

      // Initialize HPF parameters
      hpf_coeff1.freq = _IQ(18);   // in Hz18
      hpf_coeff1.PiT  = _IQ(PI*T);
      HPF_INIT(&hpf_coeff1);

      // Initialize angle transition parameters
      transition1.spdLo = TRANS_SPD_LO;
      transition1.spdHi = TRANS_SPD_HI;
      transition1.scale = TRANS_SCALE;

      // Initialize NS determination parameters
      ns_id1.cntON     = 0;//5
      ns_id1.cntPRD    = 30;//30
      ns_id1.PWM_ch[0] = 1;       // epwm1
      ns_id1.PWM_ch[1] = 2;       // epwm2
      ns_id1.PWM_ch[2] = 3;       // epwm3
//      ns_id1.PWM_PeriodMax = pwm1.PeriodMax;
      ns_id1.PWM_PeriodMax = SVPWMTBPRD;

      pi_spd.Kp   = _IQ(1.0);  //10.);    //3.0);//_IQ(2);
      pi_spd.Ki   = _IQ(T*20);//_IQ(0.005);  //0.003);  //20
      pi_spd.Umax = _IQ(0.5);
      pi_spd.Umin = _IQ(-0.5);
  // Initialize the PI module for Id
      pi_id.Kp   = _IQ(0.5);//_IQ(4.0);//_IQ(0.5);
      pi_id.Ki   = _IQ(T*2);//_IQ(T/0.01);//_IQ(T/0.04);  //*RS/LS);   ///0.04);//25
      pi_id.Umax = _IQ(0.8);
      pi_id.Umin = _IQ(-0.8);
  // Initialize the PI module for Iq
      pi_iq.Kp   = _IQ(0.5);//0.8
      pi_iq.Ki   = _IQ(T*2);//_IQ(T/0.04);  //*RS/LS);   ///0.04);//25
      pi_iq.Umax = _IQ(0.8);
      pi_iq.Umin = _IQ(-0.8);
	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2806x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2806x_DefaultIsr.c.
	// This function is found in F2806x_PieVect.c.
	InitPieVectTable();

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 		= &cpu_timer0_isr;
	//PieVectTable.ADCINT1 	= &adc_isr;
	PieVectTable.ADCINT2		= &adc_svpwm_isr1;
	//PieVectTable.EPWM4_TZINT = &epwm4_tzint_isr;
    PieVectTable.ECAN0INTA = &ECAN0INTA_ISR;
	EDIS;    // This is needed to disable write to EALLOW protected registers

	// Step 4. Initialize the Device Peripheral. This function can be
	//         found in F2806x_CpuTimers.c
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
//	UART_vConfig(UART_9600_BAUD_VAL,UART_PARITY_NONE,UART_STOP_2);
	Sci_init(&SciaRegs);
//	CanParameter_Init();
	EpwmReg_Init();
	InitAdc();  // For this example, init the ADC
	AdcOffsetSelfCal();


	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 80MHz CPU Freq, 1 second Period (in uSeconds)
	AdcRegs_Init();
	ConfigCpuTimer(&CpuTimer0, 60, 1000);

	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the
	// below settings must also be updated.

	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

	// Step 5. User specific code, enable interrupts:


	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
	IER |= M_INT1;
	IER |= M_INT2;
	IER |= M_INT9;

	// Enable TINT0 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
//	PieCtrlRegs.PIEIER2.bit.INTx4 = 1;
//	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;

	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
	StartCpuTimer0();
	while(1)
	{
		Timer2ms_Fun();
		Timer20ms_Fun();
	}

}

void ECanconfiger(void)
{
    struct ECAN_REGS ECanaShadow;
       EALLOW;      // EALLOW enables access to protected bits

      //配置GPIO引脚工作在eCAN功能
    //   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Disable pull-up for GPIO16 (CANTXB)
    //   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up for GPIO17 (CANRXB)
    //   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch qual for GPIO17 (CANRXB)
    //   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;  // Configure GPIO16 for CANTXB operation
    //   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;  // Configure GPIO17 for CANRXB operation

      //配置eCAN的RX和TX分别为eCAN的接收和发送引脚
        ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
        ECanaShadow.CANTIOC.bit.TXFUNC = 1;
        ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

        ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
        ECanaShadow.CANRIOC.bit.RXFUNC = 1;
        ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

        ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
        ECanaShadow.CANMC.bit.STM = 0;              //工作在正常模式
        ECanaShadow.CANMC.bit.SCB = 1;
        ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
       //初始化所有主设备控制区域为0，MCF所有的位都初始化为0
        ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
        ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

    // TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
    //  as a matter of precaution.

        ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
        ECanaShadow.CANTA.all = 0xFFFFFFFF;
        ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

        ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
        ECanaShadow.CANRMP.all = 0xFFFFFFFF;
        ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;


        ECanaShadow.CANGIF0.all = ECanaRegs.CANGIF0.all;
        ECanaShadow.CANGIF0.all = 0xFFFFFFFF;
        ECanaRegs.CANGIF0.all = ECanaShadow.CANGIF0.all;

        ECanaShadow.CANGIF1.all = ECanaRegs.CANGIF1.all;
        ECanaShadow.CANGIF1.all = 0xFFFFFFFF;
        ECanaRegs.CANGIF1.all = ECanaShadow.CANGIF1.all;

    /* Configure bit timing parameters for eCANA*/

        ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
        ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
        ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

        do
        {
            ECanaShadow.CANES.all = ECanaRegs.CANES.all;
        } while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be  cleared..


        ECanaShadow.CANBTC.all = ECanaRegs.CANBTC.all;
        ECanaShadow.CANBTC.bit.BRPREG = 9;//波特率250K
        /*150/10=15*/
        ECanaShadow.CANBTC.bit.TSEG2REG = 3;
        ECanaShadow.CANBTC.bit.TSEG1REG = 6;
        ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

        ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
        ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
        ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

        do
        {
          ECanaShadow.CANES.all = ECanaRegs.CANES.all;
        } while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..


        ECanaShadow.CANME.all = ECanaRegs.CANME.all;
        ECanaShadow.CANME.all = 0;      // Required before writing the MSGIDs
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;

        /*接收邮箱的ID号*/
        ECanaMboxes.MBOX1.MSGID.all = 0x98FF63A8;
        ECanaMboxes.MBOX0.MSGID.all = 0x98FF61EE;
        ECanaMboxes.MBOX2.MSGID.all = 0x98FF4617;
        ECanaMboxes.MBOX3.MSGID.all = 0x98FF66EC;
        ECanaMboxes.MBOX4.MSGID.all = 0x98F005A8;
        ECanaMboxes.MBOX5.MSGID.all = 0x98EE8880;
        ECanaMboxes.MBOX8.MSGID.all = 0x98FF1111;
        ECanaMboxes.MBOX16.MSGID.all = 0x98888881;
        ECanaMboxes.MBOX17.MSGID.all = 0x98888885;
        ECanaMboxes.MBOX18.MSGID.all = 0x98888886;
        ECanaMboxes.MBOX19.MSGID.all = 0x98888887;
        ECanaMboxes.MBOX20.MSGID.all = 0x98888889;
    //  ECanaMboxes.MBOX3.MSGID.all = 0x98ff0af6;
        /*邮箱0为TX，1为RX*/
        ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
        ECanaShadow.CANMD.bit.MD1 =1;
        ECanaShadow.CANMD.bit.MD0 =0;
        ECanaShadow.CANMD.bit.MD2 =1;
        ECanaShadow.CANMD.bit.MD3 =0;
        ECanaShadow.CANMD.bit.MD4 =1;
        ECanaShadow.CANMD.bit.MD5 =0;
        ECanaShadow.CANMD.bit.MD8 =1;
        ECanaShadow.CANMD.bit.MD16 =0;
        ECanaShadow.CANMD.bit.MD17 =0;
        ECanaShadow.CANMD.bit.MD18 =0;
        ECanaShadow.CANMD.bit.MD19 =0;
        ECanaShadow.CANMD.bit.MD20 =0;
    //    ECanaShadow.CANMD.bit.MD3 =1;
        ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

        /*数据长度 8个BYTE*/
        ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX19.MSGCTRL.bit.DLC = 8;
        ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;
        //设置发送优先级  2009.3.15 Add
        //ECanaMboxes.MBOX16.MSGCTRL.bit.TPL = 0;

        /*没有远方应答帧被请求*/
        ECanaMboxes.MBOX1.MSGCTRL.bit.RTR = 0;
        ECanaMboxes.MBOX2.MSGCTRL.bit.RTR = 0;
        ECanaMboxes.MBOX4.MSGCTRL.bit.RTR = 0;
        ECanaMboxes.MBOX8.MSGCTRL.bit.RTR = 0;
        /*自动应答远程帧*/
    //    ECanaMboxes.MBOX16.MSGID.bit.AAM = 1;
     //   ECanaMboxes.MBOX17.MSGID.bit.AAM = 1;
    //    ECanaMboxes.MBOX18.MSGID.bit.AAM = 1;
    //    ECanaMboxes.MBOX19.MSGID.bit.AAM = 1;
    //    ECanaMboxes.MBOX20.MSGID.bit.AAM = 1;

       //向邮箱RAM区写数据
        ECanaMboxes.MBOX0.MDH.all =0x00;  // 0x89ABCDEF;
        ECanaMboxes.MBOX0.MDL.all = 0x00;//0x12345567;
        ECanaMboxes.MBOX3.MDH.all =0x00;  // 0x89ABCDEF;
        ECanaMboxes.MBOX3.MDL.all = 0x00;//0x12345567;
        ECanaMboxes.MBOX5.MDH.all =0x00;  // 0x89ABCDEF;
        ECanaMboxes.MBOX5.MDL.all = 0x00;//0x12345567;
        ECanaMboxes.MBOX16.MDH.all = 0x00;//0x89ABCDEF;
        ECanaMboxes.MBOX16.MDL.all = 0x00;//0x12345567;

        //邮箱使能Mailbox0
        ECanaShadow.CANME.all = ECanaRegs.CANME.all;
        ECanaShadow.CANME.bit.ME1 =1;
        ECanaShadow.CANME.bit.ME0 =1;
//        ECanaShadow.CANME.bit.ME2 =1;
//      ECanaShadow.CANME.bit.ME3 =1;
//      ECanaShadow.CANME.bit.ME4 =1;
//        ECanaShadow.CANME.bit.ME5 =1;
//      ECanaShadow.CANME.bit.ME8 =1;
//      ECanaShadow.CANME.bit.ME16 =1;
//      ECanaShadow.CANME.bit.ME17 =1;
//      ECanaShadow.CANME.bit.ME18 =1;
//      ECanaShadow.CANME.bit.ME19 =1;
//      ECanaShadow.CANME.bit.ME20 =1;
        ECanaRegs.CANME.all = ECanaShadow.CANME.all;

        ECanaRegs.CANMIM.all = 0xFFFFFFFF;
        //邮箱中断将产生在ECAN0INT
        ECanaRegs.CANMIL.all = 0;
        ECanaRegs.CANGIF0.all = 0xFFFFFFFF;
        //ECAN0INT中断请求线被使能
        ECanaRegs.CANGIM.bit.I0EN = 1;

        EDIS;


}
// INT9.5
interrupt void ECAN0INTA_ISR(void)  // eCAN-A
{
  // Insert ISR Code here

    //while(ECanaRegs.CANRMP.all != 0x00010000 ) ;
struct ECAN_REGS ECanaShadow;
ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
    if(ECanaShadow.CANRMP.bit.RMP1 == 1 )
    {

        can_start_flag=1;
        can_start_flag1=20;
        Rec_l = ECanaMboxes.MBOX1.MDL.all; //锟秸碉拷锟斤拷锟斤拷锟斤拷诮锟斤拷锟斤拷锟斤拷锟組box1
        Rec_h = ECanaMboxes.MBOX1.MDH.all;
        ECanaShadow.CANRMP.bit.RMP1 = 1; //复位RMP标志，同时也复位中断标志
        ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

        switch(Rec_l&0x07000000)
        {
        //case 0x0000000:
        //SpeedRef=_IQ(0.667);Can_start_flag2=1;break;
        case 0x01000000:
        SpeedRef=_IQ(0.04);START=1;break;
        case 0x02000000:
//          GpioDataRegs.GPASET.bit.GPIO18=1;break;
        //SpeedRef=_IQ(0.0);stop_flag=1;Can_start_flag2=0;V_startflag=0;V_startflagcount=0; break;
        SpeedRef=_IQ(0.08);START=1;break;
        case 0x03000000:
//          GpioDataRegs.GPACLEAR.bit.GPIO18=1;break;
        SpeedRef=_IQ(0.1);START=1;break;
        //SpeedRef=_IQ(0.517);Can_start_flag2=1;break;
        case 0x04000000:
//          GpioDataRegs.GPASET.bit.GPIO19=1;break;
//          case 0x05000000:
//          GpioDataRegs.GPACLEAR.bit.GPIO19=1;break;
        SpeedRef=_IQ(0.3);START=1;break;
        case 0x5000000:
        SpeedRef=_IQ(0.567);START=1;break;
        case 0x6000000:
        SpeedRef=_IQ(0.8);START=1;break;
        case 0x7000000:
        SpeedRef=_IQ(1.36);START=1;break;
        case 0x0000000:
        SpeedRef=_IQ(0.0);START=0; break;
        default:
        break;
        }
    }



    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    EINT;





}
