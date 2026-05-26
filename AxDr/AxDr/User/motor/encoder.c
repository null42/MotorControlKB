#include "encoder.h"
#include "main.h"
#include "math.h"
#include "foc_drv.h"
#include "spi.h"

#include "stdio.h"

MT6816_SPI_Signal_Typedef	mt6816_spi;

void REIN_MT6816_SPI_Signal_Init(void)
{
	mt6816_spi.sample_data = 0;
	mt6816_spi.angle = 0;
}

void RINE_MT6816_SPI_Get_AngleData(void)
{
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	for(uint8_t i=0; i<3; i++){
		//???SPI????
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
		MT6816_SPI_CS_H();
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
		MT6816_SPI_CS_H();
		mt6816_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		//???§µ??
		h_count = 0;
		for(uint8_t j=0; j<16; j++){
			if(mt6816_spi.sample_data & (0x0001 << j))
				h_count++;
		}
		if(h_count & 0x01){
			mt6816_spi.pc_flag = false;
		}
		else{
			mt6816_spi.pc_flag = true;
			break;
		}
	}
	if(mt6816_spi.pc_flag){
		mt6816_spi.angle = mt6816_spi.sample_data >> 2;
		mt6816_spi.no_mag_flag = (bool)(mt6816_spi.sample_data & (0x0001 << 1));
	}
}

MT6816_Typedef	mt6816;

float REIN_MT6816_Get_AngleData()
{
	RINE_MT6816_SPI_Get_AngleData();
	mt6816.angle_data = mt6816_spi.angle;   
}

void MT6816_Calc_Elec_Angle(uint8_t pole_pairs)
{
	float angle_diff;
	
	RINE_MT6816_SPI_Get_AngleData();
	mt6816.angle_data = mt6816_spi.angle;
	mt6816.mech_angle = (float)mt6816.angle_data * (2.0f * 3.14159265358f) / 16384.0f;
	
	angle_diff = mt6816.mech_angle - mt6816.last_angle;
	if (angle_diff > 3.14159265358f)
		angle_diff -= 2.0f * 3.14159265358f;
	else if (angle_diff < -3.14159265358f)
		angle_diff += 2.0f * 3.14159265358f;
	
	mt6816.speed = angle_diff * 20000.0f;
	mt6816.last_angle = mt6816.mech_angle;
	
	mt6816.elec_angle = mt6816.mech_angle * (float)pole_pairs;
	while(mt6816.elec_angle > 2.0f * 3.14159265358f)
	{
		mt6816.elec_angle -= 2.0f * 3.14159265358f;
	}
}
