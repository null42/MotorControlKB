/*
 * Uart.h
 *
 *  Created on: 31 Mar 2018
 *      Author: LMD
 */

#ifndef UART_H_
#define UART_H_

/*----------------------------------------------------------------------------*\
  INCLUDES
\*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*\
  DEFINES
\*----------------------------------------------------------------------------*/
/* Return values of function UART_isError */
#define UART_NO_ERROR            0
#define UART_RX_BREAK_ERR_FLAG   1
#define UART_RX_FRAME_ERR_FLAG   2
#define UART_RX_OVL_ERR_FLAG     4
#define UART_RX_PARITY_ERR_FLAG  8

#define SYS_CLK_HZ					(Uint32)60000000/4

#define UART_BAUD_TO_TICKS(BAUD)   (Uint16)((SYS_CLK_HZ/((BAUD)*8UL))-1UL)
#define UART_4800_BAUD_VAL         UART_BAUD_TO_TICKS(4800UL)
#define UART_9600_BAUD_VAL         UART_BAUD_TO_TICKS(9600UL)
#define UART_19200_BAUD_VAL        UART_BAUD_TO_TICKS(19200UL)
#define UART_38400_BAUD_VAL        UART_BAUD_TO_TICKS(38400UL)
#define UART_57600_BAUD_VAL        UART_BAUD_TO_TICKS(57600UL)
#define UART_115200_BAUD_VAL       UART_BAUD_TO_TICKS(115200UL)

/*----------------------------------------------------------------------------*\
  TYPEDEFS
\*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*\
  ENUMS
\*----------------------------------------------------------------------------*/
typedef enum UART_Parity
{
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD  = 1,
    UART_PARITY_EVEN = 3
} UART_TEparity;

typedef enum UART_StopBits
{
    UART_STOP_1      = 0,
    UART_STOP_2
} UART_TEstopBits;

/*----------------------------------------------------------------------------*\
  VARIABLES
\*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*\
  FUNCTION PROTOTYPES
\*----------------------------------------------------------------------------*/
interrupt void UART_vReceiveInterrupt ( void );
interrupt void UART_vTransmitInterrupt ( void );

extern void    UART_vConfig ( Uint16 u16Baudrate, UART_TEparity eParity, UART_TEstopBits eStopBits );
extern Uint16  UART_u16ReadData ( Uint8* pu8Data, const Uint16 u16MaxDataCnt );
extern Uint16  UART_u16SendData ( const Uint8* pu8Data, const Uint16 u16MaxDataCnt );
extern boolean UART_bIsIdle ( void );
extern Uint16  UART_u16GetError ( void );

#endif /* UART_H_ */

/*----------------------------------------------------------------------------*\
  END OF FILE
\*----------------------------------------------------------------------------*/
