//=========================================电源接线================================================//
// OLED模块               STM32单片机
//   VCC         接       DC 5V/3.3V      //OLED屏电源正
//   GND         接          GND          //OLED屏电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为4线制SPI
// OLED模块               STM32单片机
//   D1          接          PB15        //OLED屏SPI写信号
//=======================================液晶屏控制线接线==========================================//
// OLED模块               STM32单片机
//   CS          接          PB11        //OLED屏片选控制信号
//   RES         接          PB12        //OLED屏复位控制信号
//   DC          接          PB10        //OLED屏数据/命令选择控制信号
//   D0          接          PB13        //OLED屏SPI时钟信号
//=========================================触摸屏接线=========================================//
//本模块不带触摸功能，所以不需要触摸屏接线
/***************************************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
****************************************************************************************************/		
#ifndef __GUI_H__
#define __GUI_H__

#include "sys.h"

void GUI_DrawPoint(u8 x, u8 y, u8 color);
void GUI_Fill(u8 sx,u8 sy,u8 ex,u8 ey,u8 color);
void GUI_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2,u8 color);
void GUI_DrawRectangle(u8 x1, u8 y1, u8 x2, u8 y2,u8 color);
void GUI_FillRectangle(u8 x1, u8 y1, u8 x2, u8 y2,u8 color);
void GUI_DrawCircle(u8 xc, u8 yc, u8 color, u8 r);
void GUI_FillCircle(u8 xc, u8 yc, u8 color, u8 r);
void GUI_DrawTriangel(u8 x0,u8 y0,u8 x1,u8 y1,u8 x2,u8 y2,u8 color);
void GUI_FillTriangel(u8 x0,u8 y0,u8 x1,u8 y1,u8 x2,u8 y2,u8 color);
void GUI_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size,u8 mode);
void GUI_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 Size,u8 mode);    								//数字显示
void GUI_ShowString(u8 x,u8 y,char *chr,u8 Char_Size,u8 mode);   							//字符串显示
void GUI_ShowFont12(u8 x,u8 y,u8 *s,u8 mode);
void GUI_ShowFont16(u8 x,u8 y,u8 *s,u8 mode);
//void GUI_ShowFont24(u8 x,u8 y,u8 *s,u8 mode);
//void GUI_ShowFont32(u8 x,u8 y,u8 *s,u8 mode);
void GUI_ShowCHinese(u8 x,u8 y,u8 hsize,u8 *str,u8 mode);     							 	//显示汉字
void GUI_DrawBMP(u8 x,u8 y,u8 width, u8 height, u8 BMP[], u8 mode);     			//显示图片
void GUI_ShowNum_F(u8 x,u8 y,float num,u8 len,u8 len_f,u8 Size,u8 mode);			//显示带小数数字
void GUI_ShowNum_FU(u8 x,u8 y,float num,u8 len,u8 len_f,u8 Size,u8 mode);			//显示无符号小数
void GUI_ShowNum_Int(u8 x,u8 y,s16 num,u8 len,u8 Size,u8 mode);								//显示带符号整数
#endif

