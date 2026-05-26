/* =================================================================================
File name:       HFI.H
===================================================================================*/


#ifndef __HFI_H__
#define __HFI_H__

#include "angle_math.h"       	// Include header for the ANGLE MATH group of objects
#include "IQmathlib.h"
#include "PeripheralHeaderIncludes.h"

#include "volt_calc.h"
#include "clarke.h"
//#include "VOLT_CALC_CLA.h"
//#include "CLARKE_CLA.h"
/***********************************
 * HFI state machine states
 * 0 - RESET state
 * 1 - HFI @ IPD state
 * 2 - NS determination
 * 3 - HFI @ RUN state
 ***********************************/
#define  HFI_RESET_STATE  0
#define  HFI_IPD_STATE    1
#define  HFI_NSID_STATE   2
#define  HFI_RUN_STATE    3

//=========================
// High Pass Filter Object
//=========================
typedef struct
{
	_iq In1,
		In0,
		Out;
} HPF;

typedef HPF *HPF_handle;

/*-----------------------------------------------------------------------------
	Default initializer for the HPF object.
-----------------------------------------------------------------------------*/
#define HPF_DEFAULTS     \
{                        \
	0,                   \
	0,                   \
    0,                   \
}

//======================================
// High Pass Filter Coefficients Object
//======================================
/**************************************************************
 * High Pass Filter - setting up coefficients
 * ************************************************************
 * vo(1) = N*(vi(1)-vi(0)) + D*vo(0)
 *   where,
 *     N =         2/(2+w0.Ts)   -->            1/(1+w0.Ts/2)
 *     D = (2-w0.Ts)/(2+w0.Ts)   -->  (1-w0.Ts/2)/(1+w0.Ts/2)
 **************************************************************/
typedef struct
{
	_iq 	freq,
	        PiT,
			wTby2,
			N,
		    D;
} HPF_COEFF;

typedef HPF_COEFF *HPF_COEFF_handle;

/*-----------------------------------------------------------------------------
	Default initializer for the HPF_COEFF object.
-----------------------------------------------------------------------------*/
#define  HPF_COEFF_DEFAULTS          \
{                                    \
        _IQ(18.0),                   \
        _IQ(0.0),    /* dummy */     \
        _IQ(0.0),		             \
        _IQ(0.0),		             \
        _IQ(0.0)		             \
}

//=======================================================================
// High Frequency Injection (HFI) Object
//=======================================================================
typedef struct
{
	_iq  Kp_IPD,			// Kp during IPD
	     Kp_RUN,			// Kp during RUN
	     base_wTs,			// base_wTs = BASE_FREQ * T ,for lib
	     dutyMax,           // max duty cycle
	     volt_run,          // minimum regulation Vdc during RUN
	     volt_ipd,          // minimum regulation Vdc during IPD

	     speedEst,          // estimated speed
	     thetaEst,          // estimated angle
	     duty;              // operating duty cycle

	Uint16 Squ_PRD_set,     // square wave period
	       HFI_Time1,       // end of initial alignment completion
	       HFI_Time2,       // end of post alignment no current (PAUSE) time

	       HFI_Status;      // state machine state
}HFI;

typedef HFI *HFI_handle;

/*-----------------------------------------------------------------------------
	Default initializer for the HFI object.
-----------------------------------------------------------------------------*/
#define HFI_DEFAULTS                \
{	                                \
    _IQ(8.0),                       \
    _IQ(20.0),                      \
    _IQ(0.0),                       \
    _IQ(0.0),                       \
    _IQ(0.0),                       \
    _IQ(0.0),                       \
                                    \
    _IQ(0.0),                       \
    _IQ(0.0),                       \
    _IQ(0.0),                       \
    								\
    16,                             \
    400,                            \
    450,        					\
                                    \
    0								\
}

/*-------------------------------------------
 * Position identifier object
 *-----------------------------------------*/
typedef struct {
Uint16  cntON,
		cntPRD,
		PWM_PeriodMax,
		PWM_ch[3];				// pwm channel for lib use
} NS_ID;

typedef NS_ID *NS_ID_handle;

/*----------------------------------------------------------
 * Default initializer for position identifier object
 *----------------------------------------------------------*/
#define  NS_ID_DEFAULTS				\
{									\
	5,								\
	30,								\
	0,								\
	{1,2,3}							\
}

/* ===================================================
 * Module definitions
 * ===================================================*/
void  HPF_INIT(HPF_COEFF_handle k);  // initialize HPF constants
void  HPF_RESET(HPF_handle f);       // reset HPF
void  HFI_RESET(HFI_handle h);       // reset HFI
void  ZLSPD(HPF *iq, HPF_COEFF *k, HFI *h, NS_ID *n, CLARKE *i, PHASEVOLTAGE *v);  // Zero and low speed position detection

#endif // __HFI_H__

