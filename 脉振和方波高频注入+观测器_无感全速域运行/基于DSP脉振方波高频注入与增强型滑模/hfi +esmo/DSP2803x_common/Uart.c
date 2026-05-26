/*
 * Uart.c
 *
 *  Created on: 31 Mar 2018
 *      Author: LMD
 */



/****************************************************************************//*
 * Subversion information
 * $URL$
 * $Author$
 * $Date$
 * $Rev$
 ******************************************************************************/

/******************************************************************************\
 * Memory                                                                     *
\******************************************************************************/

#pragma CODE_SECTION(UART_vReceiveInterrupt, "ramfuncs")
#pragma CODE_SECTION(UART_vTransmitInterrupt, "ramfuncs")
#pragma CODE_SECTION(UART_u16ReadData, "ramfuncs")
#pragma CODE_SECTION(UART_u16SendData, "ramfuncs")
#pragma CODE_SECTION(UART_bIsIdle, "ramfuncs")
#pragma CODE_SECTION(UART_u16GetError, "ramfuncs")


/******************************************************************************\
 * Includes                                                                   *
\******************************************************************************/

#include "DSP28x_Project.h"
#include "Uart.h"

/******************************************************************************\
 * Defines                                                                    *
\******************************************************************************/
#define UART_RX_BUF_SIZE           (Uint16)64
#define UART_TX_BUF_SIZE           (Uint16)64


#define UART_DATA_BIT_CNT          7  /* 7 -> 8 Data Bits */

/******************************************************************************\
 * Typedefs                                                                   *
\******************************************************************************/
typedef struct UART_WorkData {
    struct {
        Uint8  au8Buf[UART_RX_BUF_SIZE];
        Uint16 u16WrInd;
        Uint16 u16RdInd;
    } Rx;                            /* Rx buffer working data */
    struct {
        Uint8  au8Buf[UART_TX_BUF_SIZE];
        Uint16 u16WrInd;
        Uint16 u16RdInd;
    } Tx;                            /* Tx buffer working data */
} UART_TSworkData;

/******************************************************************************\
 * Module global variables                                                    *
\******************************************************************************/
/*static */UART_TSworkData UART_Work_m;

/******************************************************************************\
 * Declaration of module global functions                                     *
\******************************************************************************/
static void UART_vSetCallbacks ( void );

/******************************************************************************\
 * Definition of global functions                                             *
\******************************************************************************/

static inline void CLEAR_VARIABLE(void)
{
	int i;
	for(i=0;i<sizeof(UART_Work_m);i++)
	{
		((char*)&UART_Work_m)[i]				= 0;
	}
}

interrupt void UART_vReceiveInterrupt ( void ) {

    /* get current write index */
    Uint16       u16WrInd = UART_Work_m.Rx.u16WrInd;
    /* calculate end position for write index */
    const Uint16 u16RdInd = UART_Work_m.Rx.u16RdInd;
    const Uint16 u16WrEnd = (u16RdInd > 0) ? (u16RdInd - 1) : (UART_RX_BUF_SIZE - 1);

    /* No receive error detected and check receive buffer not full */
    if ((0 == SciaRegs.SCIRXST.bit.RXERROR) && (u16WrEnd != u16WrInd)) {
        /* get one data byte from SCI Rx Reg and write to receive buffer */
        UART_Work_m.Rx.au8Buf[u16WrInd++] = (Uint8)SciaRegs.SCIRXBUF.bit.RXDT;
        /* handle wrap around of write index */
        if (UART_RX_BUF_SIZE <= u16WrInd) {
            u16WrInd = 0;
        }

        /* write back (changed) write index */
        UART_Work_m.Rx.u16WrInd = u16WrInd;
    }
    /* Ack group */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}

interrupt void UART_vTransmitInterrupt ( void ) {

    /* get current read index */
    Uint16       u16RdInd = UART_Work_m.Tx.u16RdInd;
    /* get end position for read index */
    const Uint16 u16RdEnd = UART_Work_m.Tx.u16WrInd;

    /* Tx register empty and data to transmit available ? */
    if (/*(0 != SciaRegs.SCICTL2.bit.TXRDY) && */(u16RdEnd != u16RdInd)) {
        /* get one Byte and write to SCI Tx Reg */
        SciaRegs.SCITXBUF = UART_Work_m.Tx.au8Buf[u16RdInd++];
        /* handle wrap around of read index */
        if (UART_TX_BUF_SIZE <= u16RdInd) {
            u16RdInd = 0;
        }
        /* write back (changed) read index */
        UART_Work_m.Tx.u16RdInd = u16RdInd;
    }
    /* Ack group */
    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}


void UART_vConfig ( Uint16 u16Baudrate, UART_TEparity eParity, UART_TEstopBits eStopBits ) {
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
    EDIS;

    /* Reset UART */
    SciaRegs.SCICTL1.bit.SWRESET = 0;

    /* Clear work data */
    CLEAR_VARIABLE();

    /* configure UART ports */
    InitSciaGpio();

    /* Enable Receiver and Transmitter */
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;

    /* Set Baudrate registers */
    SciaRegs.SCILBAUD = (Uint8)u16Baudrate;
    SciaRegs.SCIHBAUD = (Uint8)(u16Baudrate>>8);
    /* Configure UART */
    SciaRegs.SCICCR.bit.STOPBITS  = eStopBits;
    SciaRegs.SCICCR.bit.PARITYENA = ((Uint8)eParity & (Uint8)0x01u);
    SciaRegs.SCICCR.bit.PARITY    = (((Uint8)eParity >> (Uint8)1u) & (Uint8)0x01u);
    SciaRegs.SCICCR.bit.SCICHAR   = UART_DATA_BIT_CNT;

    SciaRegs.SCICCR.bit.LOOPBKENA =0; // Disable loop back
    SciaRegs.SCIFFTX.all=0xC020;
    SciaRegs.SCIFFRX.all=0x0021;
    SciaRegs.SCIFFCT.all=0x00;

    /* set UART interrupt Callbacks */
    UART_vSetCallbacks();

    SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

    /* Enable UART */
//    SciaRegs.SCICTL1.bit.SWRESET = 1;
}


Uint16 UART_u16ReadData ( Uint8* pu8Data, const Uint16 u16MaxDataCnt ) {
    Uint16 u16RxDataCnt   = 0;
    /* get current read index */
    Uint16       u16RdInd = UART_Work_m.Rx.u16RdInd;
    /* get end position for read index */
    const Uint16 u16RdEnd = UART_Work_m.Rx.u16WrInd;

    /* while Data accepted and receive data available */
    while ((u16MaxDataCnt > u16RxDataCnt) && (u16RdEnd != u16RdInd)) {
        /* get one Byte and pass to caller */
        *pu8Data++ = UART_Work_m.Rx.au8Buf[u16RdInd++];
        /* handle wrap around of read index */
        if (UART_RX_BUF_SIZE <= u16RdInd) {
            u16RdInd = 0;
        }
        /* one byte handled */
        u16RxDataCnt++;
    }
    /* write back (changed) read index */
    UART_Work_m.Rx.u16RdInd = u16RdInd;

    /* return number of handled bytes */
    return(u16RxDataCnt);
}


Uint16 UART_u16SendData ( const Uint8* pu8Data, const Uint16 u16MaxDataCnt ) {

	volatile Uint16 u16TxDataCnt   = 0;
    /* get current write index */
    Uint16       u16WrInd = UART_Work_m.Tx.u16WrInd;
    /* calculate end position for write index */
    const Uint16 u16RdInd = UART_Work_m.Tx.u16RdInd;
    const Uint16 u16WrEnd = (u16RdInd > 0) ? (u16RdInd - 1) : (UART_TX_BUF_SIZE - 1);

    /* while Data to send available and transmit buffer not full */
    while ((u16MaxDataCnt > u16TxDataCnt) && (u16WrEnd != u16WrInd)) {
        /* copy one data byte */
        UART_Work_m.Tx.au8Buf[u16WrInd++] = *pu8Data++;
        /* handle wrap around of write index */
        if(UART_TX_BUF_SIZE <= u16WrInd) {
            u16WrInd = 0;
        }
        /* one byte handled */
        u16TxDataCnt++;
    }
    /* write back (changed) write index */
    UART_Work_m.Tx.u16WrInd = u16WrInd;

    /* Trigger new SCI Tx interrupt via PIE */
    PieCtrlRegs.PIEIFR9.bit.INTx2 = 1;

    /* return number of handled bytes */
    return( u16TxDataCnt);
}


boolean UART_bIsIdle ( void ) {
    boolean bIsIdle = FALSE;

    /* First check if the Tx buffer is empty */
    if ( UART_Work_m.Tx.u16RdInd == UART_Work_m.Tx.u16WrInd ) {
        /* then check if the UART has no Tx data */
        if (0 != SciaRegs.SCICTL2.bit.TXEMPTY) {
            /* UART is Idle */
            bIsIdle = TRUE;
        }
    }
    return(bIsIdle);
}


Uint16 UART_u16GetError ( void ) {
    Uint16 u16ErrorFlags = UART_NO_ERROR;

    /* Receive error active? */
    if (0 != SciaRegs.SCIRXST.bit.RXERROR) {
        /* Check for Break error flag */
        if (0 != SciaRegs.SCIRXST.bit.BRKDT) {
            u16ErrorFlags |= UART_RX_BREAK_ERR_FLAG;
        }
        /* Check for Frame error flag */
        if (0 != SciaRegs.SCIRXST.bit.FE) {
            u16ErrorFlags |= UART_RX_FRAME_ERR_FLAG;
        }
        /* Check for Overflow error flag */
        if (0 != SciaRegs.SCIRXST.bit.OE) {
            u16ErrorFlags |= UART_RX_OVL_ERR_FLAG;
        }
        /* Check for Parity error flag */
        if (0 != SciaRegs.SCIRXST.bit.PE) {
            u16ErrorFlags |= UART_RX_PARITY_ERR_FLAG;
        }
        /* Reset UART to clear the error flag(s) */
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        __asm(" RPT #10 || NOP");
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }
    /* return detected UART error information */
    return( u16ErrorFlags);
}


static void UART_vSetCallbacks ( void ) {
    EALLOW;

    /* Disable PIE interrupt 1 and 2 at group 9 */
    PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
    PieCtrlRegs.PIEIER9.bit.INTx2 = 0;
    __asm(" rpt #4 || nop");
    /* Clear CPU interrupt flag 9 */
    IFR &= ~M_INT9;
    /* Acknowledge PIE interrupt group 9 */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    /* Set ISR vectors */
    PieVectTable.SCIRXINTA = &UART_vReceiveInterrupt;
    PieVectTable.SCITXINTA = &UART_vTransmitInterrupt;
    /* Enable UART interupts */
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;//1;		//adon
    SciaRegs.SCICTL2.bit.TXINTENA = 0;//1;
    /* Enable PIE interrupt 1 and 2 at group 9 */
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
    /* Enable CPU interrupt 9 */
    IER |= M_INT9;

    EDIS;
}


/*----------------------------------------------------------------------------*\
  END OF FILE
\*----------------------------------------------------------------------------*/










