/*
 * cancomm.c
 *
 *  Created on: 8 May 2018
 *      Author: LMD
 */

#include "DSP28x_Project.h"
#include "application.h"
#include "cancomm.h"


ECanParameterMode ECanaParameter;


static void CanRead_fun(ECanParameterMode *ecan,volatile struct MBOX *p);
void ECana_Init(void);



void CanParameter_Init(void)
{
    int i;
    for(i=0;i<sizeof(ECanaParameter);i++)
    {
        ((char*)&ECanaParameter)[i]     = 0;
    }
 //   ECana_Init();
}


void ECANRxProtocol_fun(void)
{
    Uint32 tempECANARMP;

/*********************Configure in initial*************************/

    if(ECanaRegs.CANRMP.all != 0x00000000)
    {
        tempECANARMP = ECanaRegs.CANRMP.all & 0x80000000;
        if(tempECANARMP == 0x80000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX31);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x80000000;      //Clear RMP.31 bit.
        }

        tempECANARMP = ECanaRegs.CANRMP.all & 0x40000000;
        if(tempECANARMP == 0x40000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX30);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x40000000;      //Clear RMP.30 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x20000000;
        if(tempECANARMP == 0x20000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX29);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x20000000;      //Clear RMP.29 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x10000000;
        if(tempECANARMP == 0x10000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX28);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x10000000;      //Clear RMP.28 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x08000000;
        if(tempECANARMP == 0x08000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX27);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x08000000;      //Clear RMP.27 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x04000000;
        if(tempECANARMP == 0x04000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX26);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x04000000;      //Clear RMP.26 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x02000000;
        if(tempECANARMP == 0x02000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX25);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x02000000;      //Clear RMP.25 bit.
        }
        tempECANARMP = ECanaRegs.CANRMP.all & 0x01000000;
        if(tempECANARMP == 0x01000000)
        {
            CanRead_fun(&ECanaParameter,&ECanaMboxes.MBOX24);
            if(ECanaParameter.ReceiveCallBack != 0)
                (*ECanaParameter.ReceiveCallBack)();

            ECanaRegs.CANRMP.all = 0x01000000;      //Clear RMP.24 bit.
        }
    }
}


Uint16  ECANATxDataFromMailBox0(TxFrameMode *sp)
{
    volatile struct ECAN_REGS ECanaShadow;

    if(ECanaParameter.TxContinueFlag == 1)
    {
        ECanaParameter.TxOverTimer++;

        if(ECanaRegs.CANTA.bit.TA0 == 1)
        {
            ECanaRegs.CANTA.bit.TA0             = 1;           // Clear TA1.
            ECanaParameter.TxContinueFlag       = 0;
            ECanaParameter.TxOverTimer          = 0;
        }
    }
    else
    {
        ECanaShadow.CANME.all           = ECanaRegs.CANME.all;
        ECanaShadow.CANME.bit.ME0       = 0;
        ECanaRegs.CANME.all             = ECanaShadow.CANME.all;

        ECanaMboxes.MBOX0.MSGID.all     = sp->tempMassageID;
        ECanaMboxes.MBOX0.MSGID.bit.IDE = sp->tempIDE;
        ECanaMboxes.MBOX0.MSGID.bit.AAM = 0;
        ECanaMboxes.MBOX0.MSGID.bit.AME = 0;

        ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = sp->tempDLC;

        ECanaMboxes.MBOX0.MDL.byte.BYTE0 = sp->tempData[0];
        ECanaMboxes.MBOX0.MDL.byte.BYTE1 = sp->tempData[1];
        ECanaMboxes.MBOX0.MDL.byte.BYTE2 = sp->tempData[2];
        ECanaMboxes.MBOX0.MDL.byte.BYTE3 = sp->tempData[3];
        ECanaMboxes.MBOX0.MDH.byte.BYTE4 = sp->tempData[4];
        ECanaMboxes.MBOX0.MDH.byte.BYTE5 = sp->tempData[5];
        ECanaMboxes.MBOX0.MDH.byte.BYTE6 = sp->tempData[6];
        ECanaMboxes.MBOX0.MDH.byte.BYTE7 = sp->tempData[7];

    //  ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    //  ECanaShadow.CANMD.bit.MD0 = 0;
    //  ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

        ECanaShadow.CANME.all               = ECanaRegs.CANME.all;
        ECanaShadow.CANME.bit.ME0           = 1;
        ECanaRegs.CANME.all                 = ECanaShadow.CANME.all;

        ECanaRegs.CANTRS.bit.TRS0           = 1;          // Set TRS0 for transmit mailboxe1.
        ECanaParameter.TxContinueFlag       = 1;
        return 1;
    }

    if(ECanaParameter.TxOverTimer >= SMC_CANDATASEND_OVERTIME)
    {
        ECanaParameter.TxContinueFlag       = 0;
        ECanaParameter.TxOverTimer          = 0;
//      Fault.type.warning.bit.Communication= 1;
        return 1;
    }
    return 0;

}


static void CanRead_fun(ECanParameterMode *ecan,volatile struct MBOX *p)
{
    ecan->RxDataBuf[0] = p->MDL.byte.BYTE0;
    ecan->RxDataBuf[1] = p->MDL.byte.BYTE1;
    ecan->RxDataBuf[2] = p->MDL.byte.BYTE2;
    ecan->RxDataBuf[3] = p->MDL.byte.BYTE3;
    ecan->RxDataBuf[4] = p->MDH.byte.BYTE4;
    ecan->RxDataBuf[5] = p->MDH.byte.BYTE5;
    ecan->RxDataBuf[6] = p->MDH.byte.BYTE6;
    ecan->RxDataBuf[7] = p->MDH.byte.BYTE7;
    ecan->RxMessageID  = p->MSGID.all & 0x1FFFFFFF;
}




// 初始化ECANA模块
void ECana_Init(void)        // Initialize eCAN-A module
{
    volatile struct ECAN_REGS ECanaShadow;
    Uint16 reBRPREG;

    #ifdef CAN_BUS_SPEED_500KBPS
    reBRPREG = 5;
    #endif

    #ifdef CAN_BUS_SPEED_250KBPS
    reBRPREG = 11;
    #endif

    #ifdef CAN_BUS_SPEED_125KBPS
    reBRPREG = 23;
    #endif

    #ifdef CAN_BUS_SPEED_100KBPS
    reBRPREG = 29;
    #endif

    #ifdef CAN_BUS_SPEED_50KBPS
    reBRPREG = 59;
    #endif

    EALLOW;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     // Enable pull-up for GPIO31 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)

/* Configure eCAN-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA operation

    EDIS;

    ECanaRegs.CANME.all = 0x00000000;        //Disable all Mailboxes

    EALLOW;
    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;     //The CANTX pin is used for the CAN transmit functions.
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;     //The CANRX pin is used for the CAN receive functions.
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;
    EDIS;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
    {
        ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be set.


    EALLOW;

    ECanaShadow.CANBTC.all = ECanaRegs.CANBTC.all;
//    ECanaShadow.CANBTC.bit.BRPREG = 7;
    ECanaShadow.CANBTC.bit.BRPREG = reBRPREG;
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
    ECanaShadow.CANBTC.bit.TSEG1REG = 6;
    ECanaShadow.CANBTC.bit.SAM = 1;     //Three times sample mmode

    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.SUSP = 0;         //SOFT mode.
    ECanaShadow.CANMC.bit.MBCC = 0;         //The time stamp counter is not reset.
    ECanaShadow.CANMC.bit.TCC = 0;          //The time stamp counter is not changed.
    ECanaShadow.CANMC.bit.SCB = 1;          //Select eCAN mode.
    ECanaShadow.CANMC.bit.CCR = 0;          //CPU requests normal operation.
    ECanaShadow.CANMC.bit.PDR = 0;          //The local power-down mode is not requested (normal operation).
    ECanaShadow.CANMC.bit.DBO = 0;          //The data is received or transmitted most significant byte first.
    ECanaShadow.CANMC.bit.WUBA = 0;
    ECanaShadow.CANMC.bit.CDR = 0;          //The CPU requests normal operation.

    ECanaShadow.CANMC.bit.ABO = 1;

    ECanaShadow.CANMC.bit.STM = 0;          //The module is in normal mode.
    ECanaShadow.CANMC.bit.SRES = 0;
    ECanaShadow.CANMC.bit.MBNR = 0;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    EDIS;

//all the bits of the MSGCTRLn registers must first be initialized to zero
//before proceeding to initialize the various bit fields to the desired values.
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

    // Wait until the CPU no longer has permission to change the configuration registers
    do
    {
        ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..

    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.all = 0xFFFF0000;     //Configure mailbox 0-15 is transmit,16-31 is receive.
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

    ECanaRegs.CANTRS.all = 0x00000000;      //Initial transmit-request-set bits,no operation.

    ECanaRegs.CANTRR.all = 0x00000000;      //Initial transmit-request-reset bits,no operation.

    ECanaRegs.CANTA.all = 0xFFFFFFFF;       //Clear all TAn bits.

    ECanaRegs.CANAA.all = 0x00000000;

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;      //Clear all RMPn bits.

    ECanaRegs.CANRML.all = 0x00000000;

    ECanaRegs.CANRFP.all = 0x00000000;

    ECanaRegs.CANGAM.all = 0xFFFFFFFF;

    ECanaRegs.CANES.all = 0x00000000;

    ECanaRegs.CANTEC.all = 0x00000000;

    ECanaRegs.CANREC.all = 0x00000000;

    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;     //Clear all interrupt flag bits.

    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    EALLOW;
    ECanaRegs.CANGIM.all = 0x00000000;
    ECanaRegs.CANMIM.all = 0x00000000;
    EDIS;

    ECanaRegs.CANMIL.all = 0x00000000;      //The mailbox interrupt is generated on interrupt line 0.

    ECanaShadow.CANOPC.all = ECanaRegs.CANOPC.all;
    ECanaShadow.CANOPC.all = 0xFFFFFFFF;    //All Mailboxs overwrite are protected.
    ECanaRegs.CANOPC.all = ECanaShadow.CANOPC.all;

    EALLOW;
    ECanaRegs.CANTSC = 0x00000000;
    EDIS;

    ECanaRegs.CANTOC.all = 0x00000000;

    ECanaRegs.CANTOS.all = 0x00000000;

//Enable extended identifier;Normal transmit mode.
    ECanaMboxes.MBOX0.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX1.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX2.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX3.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX4.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX5.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX6.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX7.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX8.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX9.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX10.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX11.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX12.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX13.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX14.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX15.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX16.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX17.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX18.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX19.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX20.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX21.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX22.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX23.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX24.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX25.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX26.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX27.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX28.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX29.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX30.MSGID.all = 0x80000000;
    ECanaMboxes.MBOX31.MSGID.all = 0x80000000;

//Enable mailbox acceptance mask bit.
//  ECanaMboxes.MBOX0.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX1.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX2.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX3.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX4.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX5.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX6.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX7.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX8.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX9.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX10.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX11.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX12.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX13.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX14.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX15.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX16.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX17.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX18.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX19.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX20.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX21.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX22.MSGID.bit.AME = 1;
//  ECanaMboxes.MBOX23.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX24.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX25.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX26.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX27.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX28.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX29.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX30.MSGID.bit.AME = 1;
    ECanaMboxes.MBOX31.MSGID.bit.AME = 1;

    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX6.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX7.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX10.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX13.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX14.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX15.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX19.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX21.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX22.MSGCTRL.bit.DLC = 8;
//  ECanaMboxes.MBOX23.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX26.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX27.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX28.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX29.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX30.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC = 8;


//Standard and extended frames can be received;
//Enable ID acceptance mask bit.
//Accept a 0 or a 1 (don't care) for the corresponding bit of the received identifier.
    ECanaLAMRegs.LAM0.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM1.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM2.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM3.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM4.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM5.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM6.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM7.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM8.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM9.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM10.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM11.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM12.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM13.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM14.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM15.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM16.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM17.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM18.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM19.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM20.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM21.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM22.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM23.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM24.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM25.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM26.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM27.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM28.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM29.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM30.all = 0x9FFFFFFF;
    ECanaLAMRegs.LAM31.all = 0x9FFFFFFF;

    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME0 = 1;
    ECanaShadow.CANME.bit.ME1 = 1;
    ECanaShadow.CANME.bit.ME2 = 1;
    ECanaShadow.CANME.bit.ME3 = 1;
    ECanaShadow.CANME.bit.ME4 = 1;
    ECanaShadow.CANME.bit.ME5 = 1;
//  ECanaShadow.CANME.bit.ME6 = 1;
//  ECanaShadow.CANME.bit.ME7 = 1;
//  ECanaShadow.CANME.bit.ME8 = 1;
//  ECanaShadow.CANME.bit.ME9 = 1;
//  ECanaShadow.CANME.bit.ME10 = 1;
//  ECanaShadow.CANME.bit.ME11 = 1;
//  ECanaShadow.CANME.bit.ME12 = 1;
//  ECanaShadow.CANME.bit.ME13 = 1;
//  ECanaShadow.CANME.bit.ME14 = 1;
//  ECanaShadow.CANME.bit.ME15 = 1;
//  ECanaShadow.CANME.bit.ME16 = 1;
//  ECanaShadow.CANME.bit.ME17 = 1;
//  ECanaShadow.CANME.bit.ME18 = 1;
//  ECanaShadow.CANME.bit.ME19 = 1;
//  ECanaShadow.CANME.bit.ME20 = 1;
//  ECanaShadow.CANME.bit.ME21 = 1;
//  ECanaShadow.CANME.bit.ME22 = 1;
//  ECanaShadow.CANME.bit.ME23 = 1;
    ECanaShadow.CANME.bit.ME24 = 1;
    ECanaShadow.CANME.bit.ME25 = 1;
    ECanaShadow.CANME.bit.ME26 = 1;
    ECanaShadow.CANME.bit.ME27 = 1;
    ECanaShadow.CANME.bit.ME28 = 1;
    ECanaShadow.CANME.bit.ME29 = 1;
    ECanaShadow.CANME.bit.ME30 = 1;
    ECanaShadow.CANME.bit.ME31 = 1;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
/*
    ECanaRegs.CANME.all = 0xFFFFFFFF;

    EALLOW;
    ECanaRegs.CANMIM.all = 0xFFFFFFFF;      //Mailbox interrput enable
    ECanaRegs.CANMIL.all = 0;               //Mailbox interrput is generated on interrupt line 1.
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;     //Global interrupt flag0 enable.
    ECanaRegs.CANGIM.bit.I0EN = 1;          //The ECAN0INT line is enalbe.
    ECanaRegs.CANGIM.bit.I1EN = 1;          //The ECAN1INT line is enalbe.
    ECanaRegs.CANGIM.all = 0x3ff07;
    EDIS;
*/
}


