/*
 * communication.h
 *
 *  Created on: 2 Apr 2018
 *      Author: LMD
 */

#ifndef SOURCE_COMMUNICATION_H_
#define SOURCE_COMMUNICATION_H_


#define Slave_addr      0x02


#define SYS_CLK_HZ					(Uint32)60000000/4

#define UART_BAUD_TO_TICKS(BAUD)   (Uint16)((SYS_CLK_HZ/((BAUD)*8UL))-1UL)
#define UART_4800_BAUD_VAL         UART_BAUD_TO_TICKS(4800UL)
#define UART_9600_BAUD_VAL         UART_BAUD_TO_TICKS(9600UL)
#define UART_19200_BAUD_VAL        UART_BAUD_TO_TICKS(19200UL)
#define UART_38400_BAUD_VAL        UART_BAUD_TO_TICKS(38400UL)
#define UART_57600_BAUD_VAL        UART_BAUD_TO_TICKS(57600UL)
#define UART_115200_BAUD_VAL       UART_BAUD_TO_TICKS(115200UL)

#define tx_max_size        64

struct SciParameterModel{
	Uint8 tx_buffer[tx_max_size];
	Uint8 tx_counter;
	Uint8 tx_index;
	Uint8 tx_index1;
	Uint8 rx_buffer[64];
	Uint8 rx_index;
};


enum ModbusFunction{
	ModbusRead = 3,
	ModbusWrite = 6,
	ModbusWrites = 16
};


typedef struct{
	Uint16 Rec_finish;
	Uint16 Char_Switch_time;
	Uint16 PublicReg[60];
	enum ModbusFunction ModFun;
	Uint16 RS485_time;
	void (*ModbusCallBack)(void);
}ModbusParameterModel;



extern ModbusParameterModel ModbusParameter;
extern volatile struct SciParameterModel  SciaParameter;


void Sci_init(volatile struct SCI_REGS *p);
void ModbusParameter_Init(void);
void ModbusTask_Fun(void);

#endif /* SOURCE_COMMUNICATION_H_ */
