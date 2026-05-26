/*
 * communication.c
 *
 *  Created on: 2 Apr 2018
 *      Author: LMD
 */
#include "DSP28x_Project.h"
#include "communication.h"

#define LED202		GpioDataRegs.GPATOGGLE.bit.GPIO14


volatile struct SciParameterModel  SciaParameter;
ModbusParameterModel ModbusParameter;


static void ModbusReadTask_Fun(void);
static void ModbusWriteTask_Fun(void);
static void ModbusWritesTask_Fun(void);
static void UART_vSetCallbacks ( void ) ;
static void UART_u16SendData(Uint8 c);
static Uint16 CRC16 (volatile Uint8 *arr_buff, Uint8 len);
void ModbusTask_Fun(void);

void ModbusParameter_Init(void)
{
	int i;
	for(i=0;i<sizeof(ModbusParameter);i++)
	{
		((char*)&ModbusParameter)[i]		= 0;
	}
	for(i=0;i<sizeof(SciaParameter);i++)
	{
		((char*)&SciaParameter)[i]			= 0;
	}
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO14			= 0;
	GpioCtrlRegs.GPADIR.bit.GPIO14			= 1;
	EDIS;
}


#pragma CODE_SECTION(ModbusTask_Fun,"ramfuncs");
void ModbusTask_Fun(void)
{
	if(ModbusParameter.Rec_finish == 1)
	{
		switch(ModbusParameter.ModFun)
		{
			case ModbusRead :
			{
				ModbusReadTask_Fun();
			}break;
			case ModbusWrite :
			{
				ModbusWriteTask_Fun();
				if(ModbusParameter.ModbusCallBack != 0)
					(*ModbusParameter.ModbusCallBack)();
			}break;
			case ModbusWrites :
			{
				ModbusWritesTask_Fun();
				if(ModbusParameter.ModbusCallBack != 0)
					(*ModbusParameter.ModbusCallBack)();
			}break;
			default:break;
		}
		LED202							= 1;
		ModbusParameter.Rec_finish		= 0;
	}
    if(++ModbusParameter.RS485_time >= 2500)
    {
        Sci_init(&SciaRegs);
        ModbusParameter.RS485_time  = 0;
    }
}


#pragma CODE_SECTION(ModbusReadTask_Fun,"ramfuncs");
static void ModbusReadTask_Fun(void)
{
	Uint16 crc,crc_h,crc_l,i,j;
	Uint16 reg_start_add,reg_counter,send_size;
	Uint8 SendBuffer[80];
	crc = CRC16(SciaParameter.rx_buffer,6);
	crc_h = crc/256;
	crc_l = crc%256;
	if((crc_l != SciaParameter.rx_buffer[6])||(crc_h !=SciaParameter.rx_buffer[7]))
	{
		/* bu zheng que hui fu*/
		SendBuffer[0] = Slave_addr;
		SendBuffer[1] = 0x83;
		SendBuffer[2] = 0x04;
		crc							  = CRC16(SendBuffer,3);
		SendBuffer[3] = crc%256;
		SendBuffer[4] = crc/256;
		for(i=0;i<5;i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
	else
	{
		/*zheng que du chu li*/
		reg_start_add					= (Uint16)SciaParameter.rx_buffer[2]*256 + SciaParameter.rx_buffer[3];
		reg_counter 					= (Uint16)SciaParameter.rx_buffer[4]*256 + SciaParameter.rx_buffer[5];
		SendBuffer[0] 	= Slave_addr;
		SendBuffer[1] 	= ModbusRead;
		SendBuffer[2] 	= reg_counter*2;
		send_size 						= 3 + SendBuffer[2];
		for(i=0; i<reg_counter; i++)
		{
			j = 3 + i*2;
			SendBuffer[j] 	= ModbusParameter.PublicReg[reg_start_add+i]/256;
			SendBuffer[j+1] = ModbusParameter.PublicReg[reg_start_add+i]%256;
		}
		crc = CRC16(SendBuffer,send_size);
		SendBuffer[send_size]	 = crc%256;
		SendBuffer[send_size+1]	 = crc/256;
		send_size = send_size + 1;
		for(i=0;i<=send_size;i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
}


#pragma CODE_SECTION(ModbusWriteTask_Fun,"ramfuncs");
static void ModbusWriteTask_Fun(void)
{
	Uint16 crc,crc_h,crc_l,i;
	Uint16 reg_start_add;
	Uint8 SendBuffer[64];
	crc = CRC16(SciaParameter.rx_buffer,6);
	crc_h = crc/256;
	crc_l = crc%256;
	if((crc_l != SciaParameter.rx_buffer[6])||(crc_h !=SciaParameter.rx_buffer[7]))
	{
		/* bu zheng chang xiang ying */
		SendBuffer[0] = Slave_addr;
		SendBuffer[1] = 0x86;
		SendBuffer[2] = 0x04;
		crc = CRC16(SendBuffer,3);
		SendBuffer[3] = crc%256;
		SendBuffer[4] = crc/256;
		for(i=0;i<5;i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
	else
	{
		/*zheng que xie dan ge ji cun qi */
		reg_start_add = (Uint16)SciaParameter.rx_buffer[2]*256 + SciaParameter.rx_buffer[3];
		ModbusParameter.PublicReg[reg_start_add] = (Uint16)SciaParameter.rx_buffer[4]*256 + SciaParameter.rx_buffer[5];
		SendBuffer[0] = Slave_addr;
		SendBuffer[1] = ModbusWrite;
		SendBuffer[2] = SciaParameter.rx_buffer[2];
		SendBuffer[3] = SciaParameter.rx_buffer[3];
		SendBuffer[4] = SciaParameter.rx_buffer[4];
		SendBuffer[5] = SciaParameter.rx_buffer[5];
		SendBuffer[6] = SciaParameter.rx_buffer[6];
		SendBuffer[7] = SciaParameter.rx_buffer[7];
		for(i=0;i<8;i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
}


#pragma CODE_SECTION(ModbusWritesTask_Fun,"ramfuncs");
static void ModbusWritesTask_Fun(void)
{
	Uint16 crc,i,j;
	Uint16 reg_start_add,reg_counter;
	Uint8 SendBuffer[80];
	crc = CRC16(SciaParameter.rx_buffer, (9 + SciaParameter.rx_buffer[6]));
	if (crc != 0)
	{
		/* bu zheng chang xiang ying */
		SendBuffer[0] 	= Slave_addr;
		SendBuffer[1] 	= 0x90;
		SendBuffer[2]	= 0x04;
		crc								= CRC16(SendBuffer, 3);
		SendBuffer[3] 	= crc % 256;
		SendBuffer[4] 	= crc / 256;
		for (i = 0; i < 5; i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
	else
	{
		/*zheng que chu li*/
		reg_start_add	= (Uint16)SciaParameter.rx_buffer[2]*256 + SciaParameter.rx_buffer[3];
		reg_counter		= (Uint16)SciaParameter.rx_buffer[4]*256 + SciaParameter.rx_buffer[5];
		for(i = 0; i < reg_counter; i++)
		{
			j			= 7 + i*2;
			ModbusParameter.PublicReg[reg_start_add + i] = (Uint16)SciaParameter.rx_buffer[j]*256 + SciaParameter.rx_buffer[j+1];
		}
		SendBuffer[0]	= Slave_addr;
		SendBuffer[1]	= ModbusWrites;
		SendBuffer[2]	= SciaParameter.rx_buffer[2];
		SendBuffer[3]	= SciaParameter.rx_buffer[3];
		SendBuffer[4]	= SciaParameter.rx_buffer[4];
		SendBuffer[5]	= SciaParameter.rx_buffer[5];
		crc								= CRC16(SendBuffer,6);
		SendBuffer[6]	= crc%256;
		SendBuffer[7]	= crc/256;
		for (i = 0; i < 8; i++)
		{
			UART_u16SendData(SendBuffer[i]);
		}
	}
}


#pragma CODE_SECTION(UART_u16SendData,"ramfuncs");
static void UART_u16SendData(Uint8 c)
{
//	RS485_control = 1;//0;

	while(SciaParameter.tx_counter >= tx_max_size);

	DINT;
//	if(ScibParameter.tx_counter || (ScibRegs.SCIFFTX.bit.TXFFST >= 1))
//	{
		SciaParameter.tx_buffer[SciaParameter.tx_index] = c;
		if(++SciaParameter.tx_index >= tx_max_size)
			SciaParameter.tx_index = 0;
		++SciaParameter.tx_counter;
		SciaRegs.SCIFFTX.bit.TXFFIENA = 1;
//	}
//	else
//	{
//		ScibRegs.SCITXBUF = c;
//	}
	EINT;

}


#pragma CODE_SECTION(sciaTxFifoIsr,"ramfuncs");
interrupt void sciaTxFifoIsr(void)
{
	if(SciaParameter.tx_counter)
	{
		--SciaParameter.tx_counter;
		SciaRegs.SCITXBUF = SciaParameter.tx_buffer[SciaParameter.tx_index1];
		if(++SciaParameter.tx_index1 >= tx_max_size)
		SciaParameter.tx_index1 = 0;
	}
	else
	{
		SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
	}
    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
}


#pragma CODE_SECTION(sciaRxFifoIsr,"ramfuncs");
interrupt void sciaRxFifoIsr(void)
{
	SciaParameter.rx_buffer[SciaParameter.rx_index] = SciaRegs.SCIRXBUF.bit.RXDT;
	if(SciaParameter.rx_buffer[0] == Slave_addr)
	{
		SciaParameter.rx_index++;
	}
	if((SciaParameter.rx_buffer[1] == 0x03) &(SciaParameter.rx_index == 0x08))
	{
		ModbusParameter.Rec_finish 	= 1;
		ModbusParameter.ModFun		= ModbusRead;
	}
	else if((SciaParameter.rx_buffer[1] == 0x06) &(SciaParameter.rx_index == 0x08))
	{
		ModbusParameter.Rec_finish 	= 1;
		ModbusParameter.ModFun		= ModbusWrite;
	}
	else if((SciaParameter.rx_buffer[1] == 0x10) &(SciaParameter.rx_index == (9 + SciaParameter.rx_buffer[6])))
	{
		ModbusParameter.Rec_finish 	= 1;
		ModbusParameter.ModFun		= ModbusWrites;
	}
	ModbusParameter.Char_Switch_time = 0;
	ModbusParameter.RS485_time = 0;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}


void Sci_init(volatile struct SCI_REGS *p)
{
	Uint16 u16Baudrate = UART_BAUD_TO_TICKS(19200);
	InitSciaGpio();
	p->SCICCR.all =0x0007;   // 1 stop bit,  No loopback
								  // No parity,8 char bits,
								  // async mode, idle-line protocol
	p->SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
								  // Disable RX ERR, SLEEP, TXWAKE
	p->SCICTL2.bit.TXINTENA =1;
	p->SCICTL2.bit.RXBKINTENA =1;
	p->SCIHBAUD = (Uint8)(u16Baudrate>>8);
	p->SCILBAUD = (Uint8)u16Baudrate;
	p->SCICCR.bit.LOOPBKENA =0; // Disable loop back
	p->SCIFFTX.all=0xC020;
	p->SCIFFRX.all=0x0021;
	p->SCIFFCT.all=0x00;

	UART_vSetCallbacks();

	p->SCICTL1.all =0x0023;     // Relinquish SCI from Reset
	p->SCIFFTX.bit.TXFIFOXRESET=1;
	p->SCIFFRX.bit.RXFIFORESET=1;
}

static void UART_vSetCallbacks ( void )
{
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
    PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
    PieVectTable.SCITXINTA = &sciaTxFifoIsr;
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


#pragma CODE_SECTION(CRC16,"ramfuncs");
static Uint16 CRC16 (volatile Uint8 *arr_buff, Uint8 len)
{
	Uint16 cr = 0xFFFF;
	Uint8 i, j;
	for (j = 0; j < len; j++)
	{
		cr=cr ^*arr_buff++;
		for ( i=0; i<8; i++)
		{
			if( ( cr&0x0001) >0)
			{
				cr=cr>>1;
				cr=cr^ 0xa001;
			}
      		else
      		{
				cr=cr>>1;
			}
		}
	}
	return (cr);
}







