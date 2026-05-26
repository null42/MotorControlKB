/* ==============================================================================
System Name:  	HVPM_Enhanced_Sensorless

File Name:		HVPM_Enhanced_Sensorless.h

Description:	Primary system header file for the Real Implementation of Sensorless  
          		Field Orientation Control for a Three Phase Permanent-Magnet
          		Synchronous Motor 
=================================================================================  */

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/

#include "smopos.h"       		// Include header for the SMOPOS object
#include "smopos_const.h"       // Include header for the SMOPOS object
#include "park.h"       		// Include header for the PARK object
#include "ipark.h"       		// Include header for the IPARK object
#include "pi.h"       			// Include header for the PIDREG3 object 
#include "clarke.h"         	// Include header for the CLARKE object 
#include "svgen.h"		       	// Include header for the SVGENDQ object 
#include "rampgen.h"        	// Include header for the RAMPGEN object 
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object 
#include "volt_calc.h"      	// Include header for the PHASEVOLTAGE object 
#include "speed_est.h"          // Include header for the SPEED_ESTIMATION object 
#include "speed_fr.h"			// Include header for the SPEED_MEAS_QEP object
#include "speed_est.h"
#include "esmopos.h"
#include "transition.h"
#include "HFI.h"
#include "angle_math.h"
//#include "f2803xpwm.h"
//#if (DSP2803x_DEVICE_H==1)
//#include "f2803xileg_vdc.h" 	// Include header for the ILEG2DCBUSMEAS object
//#include "f2803xpwm.h"        	// Include header for the PWMGEN object
//#include "f2803xpwmdac.h"       // Include header for the PWMGEN object
//#include "f2803xqep.h"        	// Include header for the QEP object
//#include "DSP2803x_EPwm_defines.h" // Include header for PWM defines
//#endif

//#include "dlog4ch-HVPM_Enhanced_Sensorless.h"			// Include header for the DLOG_4CH object

//===========================================================================
// No more.
//===========================================================================
typedef struct {
        Uint16 PeriodMax;   // Parameter: PWM Half-Period in CPU clock cycles (Q0)
        Uint16 HalfPerMax;  // Parameter: Half of PeriodMax                   (Q0)
        Uint16 Deadband;    // Parameter: PWM deadband in CPU clock cycles    (Q0)
        _iq MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (Q24)
        _iq MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (Q24)
        _iq MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (Q24)
        } PWMGEN1 ;

#define F2803X_FC_PWM_GEN   {  1000,     \
                                 100,    \
                              0x4000,    \
                              0x4000,    \
                              0x4000,    \
                             }

#define PWMGEN_DEFAULTS     F2803X_FC_PWM_GEN
        typedef struct {
                _iq  Kdtc,                // constant K
                     Ith,                 // phase current threshold
                     scale,               // ratio of deabdand count to current threshold
                     gain;                // scale to adjust dead time compensation
                } DBC;


#define  DBC_DEFAULTS {              \
        _IQ(0.0),                    \
        _IQ(0.001),                  \
        _IQ(0.0),                    \
        _IQ(0.0)  /* _IQ(0.5) */     \
     }

#define  RESET_STATE      0
#define  IPD_STATE        1
#define  RUN_STATE        2

#define RESET_PI(v)                                             \
    v.Out = 0;                                                  \
    v.i1  = 0;                                                  \
    v.ui  = 0;

#define CURRENT_LOOP  1
#define IPD_LOOP  2
#define ESMO_LOOP  3

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   STEP_BUILD IPD_LOOP
