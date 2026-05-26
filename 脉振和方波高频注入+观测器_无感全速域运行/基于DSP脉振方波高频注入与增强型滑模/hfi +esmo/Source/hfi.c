/* ==============================================================================
System Name:  	Zero and low Speed Position Detection for PMSM

File Name:	  	HFI.C

Description:	High Frequency Injection for zero and low speed position detection
                for Three Phase Permanent-Magnet Synchronous Motor (PMSM)
=================================================================================  */

#include "hfi.h"

#define  EALLOW asm(" EALLOW")
#define  EDIS   asm(" EDIS")

#define  SIGN(A)  (A < 0 ? -1 : 1)

#define  SUCCESS   1
#define  FAILURE   0

volatile struct EPWM_REGS *ePWM_lib[] =
 				  { &EPwm1Regs,			//intentional: (ePWM[0] not used)
				  	&EPwm1Regs,
					&EPwm2Regs,
					&EPwm3Regs,
					&EPwm4Regs,
					&EPwm5Regs,
					&EPwm6Regs,
					&EPwm7Regs
				  };

void  HPF_INIT(HPF_COEFF_handle k);
void  HPF_MODULE(HPF_handle f, HPF_COEFF_handle k)  ;
void  HPF_RESET(HPF_handle f);
void  HFI_RESET(HFI_handle h);
void  ZLSPD(HPF *iq, HPF_COEFF *k, HFI *h, NS_ID *n, CLARKE *i, PHASEVOLTAGE *v);
void  NS_Determination(NS_ID *n, HFI *h, CLARKE *i);

/**************************************************************
 * High Pass Filter - setting up coefficients
 * ************************************************************
 * vo(1) = N*(vi(1)-vi(0)) + D*vo(0)
 *   where,
 *     N =         2/(2+w0.Ts)   -->            1/(1+w0.Ts/2)
 *     D = (2-w0.Ts)/(2+w0.Ts)   -->  (1-w0.Ts/2)/(1+w0.Ts/2)
 **************************************************************/
 //omega=2 π* freq
 //N = 1/(1+omega*T/2)
 //D = (1-omega*T/2)/(1+omega*T/2)
void HPF_INIT(HPF_COEFF_handle k)/*初始化高通滤波器系数*/
{
	k->wTby2 = _IQmpy(k->PiT, k->freq);
	k->N     = _IQdiv(_IQ(1.0), _IQ(1.0)+k->wTby2);
	k->D     = _IQmpy(_IQ(1.0)-k->wTby2, k->N);
}

/**************************************************************
 * High Pass Filter - implementation
 * ************************************************************
 * vo(1) = N*(vi(1)-vi(0)) + D*vo(0)
 * vi(0) = vi(1)
 ***************************************************************/
 //高通滤波器
 //f->out位滤波器输出，f->In1 当前输入f->In0 前一次输入 k 滤波器系数结构体
 //Out = N*(In1-In0)+D*Out
 //In0= In1
 //N=2/(2+omega*Ts)
 //d=(2-omega*Ts)(2+omega*Ts)
void  HPF_MODULE(HPF_handle f, HPF_COEFF_handle k)
{
	f->Out = _IQmpy(k->N, f->In1-f->In0) + _IQmpy(k->D, f->Out);
	f->In0 = f->In1;
}

/***************************************************************
 * High Pass Filter - Reset
 * ************************************************************/
void  HPF_RESET(HPF_handle f)
{
	f->Out = 0;/*输出: 0*/
	f->In0 = 0;/*前一个输入: 0*/
}


/***************************************************************
 * High Frequency Injection - Module Reset
 * ************************************************************/
void HFI_RESET(HFI_handle h)
{
    h->speedEst    = 0;/*速度估计*/
	h->thetaEst    = 0;/*角度估计*/
	h->duty        = 0;/*占空比*/
	h->HFI_Status  = HFI_RESET_STATE;
}

/***************************************************************
 * High Frequency Injection - Implementation
 高频注入主函数
 ***************************************************************/
void  ZLSPD(HPF *iq, HPF_COEFF *k, HFI *h, NS_ID *n, CLARKE *i, PHASEVOLTAGE *v)
{
	static _iq  	 kp,
     	 	 	 	 dIq,
     	 	 	 	 IqOld,
     	 	 	 	 thetaErr,
     	 	 	 	 volt_hfi;

	static Uint16 	 Squ_PRD,
       	   	   	   	 squ_cnt,
       	   	   	   	 NS_cnt;

	/********************************************
	 * Reset State
	 ********************************************/
	if(h->HFI_Status == HFI_RESET_STATE)
	{
		NS_cnt    = 0;
		thetaErr  = 0;
		IqOld     = 0;
		dIq	      = 0;
		squ_cnt   = 0;
		Squ_PRD   = h->Squ_PRD_set;//设定方波周期
		volt_hfi  = h->volt_ipd;//设定HFI电压
		kp        = h->Kp_IPD;//设定IPD期间的PLL的Kp值

//		HPF_RESET(iq);                    // reset HPF
		h->HFI_Status = HFI_IPD_STATE;    // Start HFI
	}

	/********************************************
	 * NS state
	 ********************************************/
	else if(h->HFI_Status == HFI_NSID_STATE)
	{
		NS_Determination(n,h,i);//NS磁极判断
	}

	/********************************************
	 * HFI IPD and RUN states
	 ********************************************/
	else
	{
//		HPF_MODULE(id, k);    /*  HPF on phase current to extract HF component   */
		HPF_MODULE(iq, k);    /*  HPF on phase current to extract HF component   */

		/*******************************
		 * Square wave period control
		 脉宽计数，多个开关周期组成一个正向方波激励
		 *******************************/
		if (Squ_PRD != h->Squ_PRD_set)/*判断方波周期是否等于设定的方波周期*/
		{
			if (Squ_PRD < h->Squ_PRD_set)/*小于设定值 +1*/
				Squ_PRD++;
			else/*大于设定值 -1*/
				Squ_PRD--;
		}

		/****************************************************
		 * Square wave polarity control + angle error calc
		 记录脉冲电流顶点和底点两处的Iq，中间略去，将顶点和底点的电流
		 变化量作为方波激励产生的电流变化量。也就是对应角度偏移所产生的q轴脉冲电流误差
		 同时翻转方波极性
		 ****************************************************/
		if (++squ_cnt >= Squ_PRD)
		{
			squ_cnt = 0;
			dIq     = SIGN(h->dutyMax) * (iq->Out - IqOld);     /* angle error 此处应该是电流变化量*/
			IqOld   = iq->Out;
			h->dutyMax = -h->dutyMax;/* Square wave polarity control翻转方波极性 */

			/******************************
			 * Initial HFI for IPD state
			 ******************************/
			if (NS_cnt < h->HFI_Time2)
			{
				/* Initial alignment estimation */
				if (++NS_cnt < h->HFI_Time1)
				{
					volt_hfi = h->volt_ipd;
					kp = h->Kp_IPD;
				}

				/* clear motor currents to prep for NSID */
				else if (NS_cnt < h->HFI_Time2)
				{
					volt_hfi = 0;
				}

				/* set state to NSID */
				else
					h->HFI_Status = HFI_NSID_STATE;
			}

			/*********************************
			 * Post IPD HFI for RUN state
			 *********************************/
			else
			{
				volt_hfi = h->volt_run;
				kp = h->Kp_RUN;
			}
		}

		/************************************************************/
		/*   Angle estimation  */
		//q轴电流分量直接与角度线性相关，有了角度误差，可以通过PLL或者PI调节器估算出速度，
		//速度的积分可以估算出角度
		/************************************************************/
		thetaErr  = dIq;
		h->speedEst  = _IQmpy(kp,       thetaErr);
		h->thetaEst += _IQmpy(h->speedEst, h->base_wTs);
		ANGLE_WRAP(h->thetaEst);

		/************************************************************/
		/*   Duty cycle (HFI Square wave mag) estimation    
		当BUS母线电压低于高频注入电压时，需要限制注入电压幅值*/
		/************************************************************/
		/*   volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);   */
		if (v->DcBusVolt < volt_hfi)
			h->duty = h->dutyMax;
		else
			h->duty = _IQdiv(_IQmpy(h->dutyMax, volt_hfi), v->DcBusVolt);
	}

	return;
}


/***************************************************************
 * Magnet NS polarity determination through pulse injection
 ***************************************************************/
void NS_Determination(NS_ID *n, HFI *h, CLARKE *i)
{
	Uint16  ch1 = n->PWM_ch[0],
			ch2 = n->PWM_ch[1],
			ch3 = n->PWM_ch[2],

	        PeakIndex,
	        PeakSct;

	_iq     angHi,
	        angLo,
	        angle,
	        PeakCur,
	        pulseDuty = _IQ(0.8),
	        pulseCnt  = _IQmpy(pulseDuty, n->PWM_PeriodMax),

	        curA = i->As,
	        curB = i->Bs,
	        curC = -curA-curB;

	const _iq SectorAngle[8] = {
			_IQ(5.0/6),
			_IQ(0.0),
			_IQ(1.0/6),
			_IQ(2.0/6),
			_IQ(3.0/6),
			_IQ(4.0/6),
			_IQ(5.0/6),
			_IQ(6.0/6)
	};

	const Uint16 vt[7][3] = {
			{0,0,0},    // intentional zero vector slot for sequence priming
			{1,0,0},
			{1,1,0},
			{0,1,0},
			{0,1,1},
			{0,0,1},
			{1,0,1}
	};

	static _iq  PeakCurTable[6],
	            PeakCurMax[2];

	static Uint16 PeakSector[2],
	              cnt,
	              vector = 0,
	              NS_Status;

	/**************************************************************************
	 * generate pulses based on sw count based pwm
	 * - period of sw PWM = cntPRD
	 * - duty cycle of sw PWM = cntON
	 *
	 * when cnt < cntON ---> when cnt == 0, generate pulse outputs for 3 phases
	 * when cnt >= cntON --> when cnt == cntON, disable PWM and set output = 0,
	 *                       sample the dominant phase current, stack it in an
	 *                       array and when all samples are available, determine
	 *                       the alignment of magnetic poles and rotor angle
	 * vector = 0 --> zero vector, used to just prime the pulse sequence so
	 *                that the first active pulse shows up properly. Without
	 *                this, there was some ambiguity about the visibility of
	 *                first active pulse after a cpu reset.
	 **************************************************************************/

	if (++cnt >= n->cntPRD)
	{
		cnt = 0;
		if (vector == 0)
			vector = 1;
		else if (vector == 7)  // vector = 7 means NSID complete
		{
			vector = 0;                           // reinitialize for next NSID call
			if(NS_Status == SUCCESS)
				h->HFI_Status = HFI_RUN_STATE;    // NS determination success, change to RUN_STATE
			else
				h->HFI_Status = HFI_RESET_STATE;  // NS determination fail, change to RESET_STATE
		}
	}

	if (cnt == 0)
	{
		/* set up PWM duty cycles */
		(*ePWM_lib[ch1]).CMPA.half.CMPA = vt[vector][0] * pulseCnt;
		(*ePWM_lib[ch2]).CMPA.half.CMPA = vt[vector][1] * pulseCnt;
		(*ePWM_lib[ch3]).CMPA.half.CMPA = vt[vector][2] * pulseCnt;

		/* PWM outputs - Enable */
		EALLOW;
		(*ePWM_lib[ch1]).TZCLR.bit.OST=1;
		(*ePWM_lib[ch2]).TZCLR.bit.OST=1;
		(*ePWM_lib[ch3]).TZCLR.bit.OST=1;
		EDIS;
	}

	else if (cnt == n->cntON)
	{
		// reset PWM duty cycles to zero
		(*ePWM_lib[ch1]).CMPA.half.CMPA = 0;
		(*ePWM_lib[ch2]).CMPA.half.CMPA = 0;
		(*ePWM_lib[ch3]).CMPA.half.CMPA = 0;

		/* PWM outputs set to high impedance */
		EALLOW;
		(*ePWM_lib[ch1]).TZFRC.bit.OST=1;
		(*ePWM_lib[ch2]).TZFRC.bit.OST=1;
		(*ePWM_lib[ch3]).TZFRC.bit.OST=1;
		EDIS;

		switch (vector)
		{
		case 1: PeakCurTable[0] = _IQabs(curA); vector = 4; break;
		case 2: PeakCurTable[1] = _IQabs(curC); vector = 5; break;
		case 3: PeakCurTable[2] = _IQabs(curB); vector = 6; break;
		case 4: PeakCurTable[3] = _IQabs(curA); vector = 2; break;
		case 5: PeakCurTable[4] = _IQabs(curC); vector = 3; break;
		case 6: PeakCurTable[5] = _IQabs(curB);

				/*****************************************
				 * Determination of peak current vectors *
				 *****************************************/
				PeakCurMax[0] = 0; PeakCurMax[1] = 0;
				for(vector = 1; vector < 7; vector ++)
				{
					PeakCur = PeakCurTable[vector-1];
					if(PeakCurMax[0] < PeakCur)
					{
						PeakCurMax[1] = PeakCurMax[0];
						PeakSector[1] = PeakSector[0];
						PeakCurMax[0] = PeakCur;
						PeakSector[0] = vector;
					}
					else if(PeakCurMax[1] < PeakCur)
					{
						PeakCurMax[1] = PeakCur;
						PeakSector[1] = vector;
					}
				}

				/*****************************************************
				 * Angle Qualification based on peak current vector  *
				 *****************************************************/
				PeakIndex = 0;  // chose the peak most vector
				while(1)
				{
					PeakSct = PeakSector[PeakIndex];
					angLo = SectorAngle[PeakSct - 1];
					angHi = SectorAngle[PeakSct + 1];
					angle = h->thetaEst;

					/*********************************************************
					 * NOTE:
					 *   Vector == 1 is a special case, where the angle rolls
					 *   between 0 deg and 360 deg. Hence some delimitation
					 *   on the angle is needed before comparing
					 *********************************************************/
					if (PeakSct == 1)
					{
						if (angle < angHi)
							angle   += _IQ(1.0);
						angHi += _IQ(1.0);
					}

					/***********************************************
					 * Check if angle lies within sector boundaries
					 ***********************************************/
					if ( (angle >= angLo) && (angle <= angHi) )
					{
						 h->thetaEst = angle;
						 NS_Status = SUCCESS;			/* success */
						 break;
					}
					else
					{
						angle += _IQ(0.5);

						/*********************************************************
						 * NOTE:
						 *   Vector == 1 is a special case, where the angle rolls
						 *   between 0 deg and 360 deg. Hence some delimitation
						 *   on the angle is needed before comparing
						 *********************************************************/
						if (PeakSct != 1)
						{
							ANGLE_WRAP(angle);
						}

						/***********************************************
						 * Check if angle lies within sector boundaries
						 ***********************************************/
						if ( (angle >= angLo) && (angle <= angHi) )
						{
							h->thetaEst = angle;
							NS_Status = SUCCESS;		/* success */
							break;
						}
						else
						{
							/*************************************************
							 * Since the max vector could not qualify the
							 * estimated angle, check and see if the next max
							 * vector could qualify the angle
							 *************************************************/
							PeakIndex ++;           //

							/*************************************************
							 * if both max vectors fail to qualify the angle
							 * estimate, then declare FAILURE status
							 * ***********************************************/
							if(PeakIndex >= 2)
							{
								NS_Status = FAILURE;		/* failure */
								break;
							}
						}
					}
				} // end while(1)

				/**************************************************
				 * clean up angle and signal end of determination
				 * ************************************************/
				ANGLE_WRAP(h->thetaEst);    // angle wrap up
				vector = 7;					// signal end of NS determination
		default: break;
		} // end switch(vector)
	} // end if (cnt == cntON)

	return;
}

//===========================================================================
// End of file
//===========================================================================
