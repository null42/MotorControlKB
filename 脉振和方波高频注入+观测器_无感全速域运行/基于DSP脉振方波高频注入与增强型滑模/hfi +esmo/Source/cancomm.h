/*
 * cancomm.h
 *
 *  Created on: 8 May 2018
 *      Author: LMD
 */

#ifndef SOURCE_CANCOMM_H_
#define SOURCE_CANCOMM_H_

#define CAN_BUS_SPEED_250KBPS

#define SMC_CANDATASEND_OVERTIME                10  //20ms


typedef struct{
    Uint32 tempMassageID;
    Uint8 tempIDE;
    Uint8 tempDLC;
    Uint8 tempData[8];

}TxFrameMode;


#define TXFRAMEINCDEMAX         5
typedef struct{
    Uint8 RxDataBuf[8];
    Uint32 RxMessageID;
    void (*ReceiveCallBack)(void);

    Uint16 TxOverTimer;
    Uint16 TxContinueFlag;
    Uint16 TxFrameIndex;
    TxFrameMode TxFrame[TXFRAMEINCDEMAX];

}ECanParameterMode;


extern ECanParameterMode ECanaParameter;


void CanParameter_Init(void);
void ECANRxProtocol_fun(void);
Uint16  ECANATxDataFromMailBox0(TxFrameMode *sp);


#endif /* SOURCE_CANCOMM_H_ */
