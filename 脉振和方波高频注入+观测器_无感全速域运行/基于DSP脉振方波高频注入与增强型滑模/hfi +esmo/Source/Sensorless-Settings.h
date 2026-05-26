/* =================================================================================
File name:  HVPM_eSensorless-Settings.H                     
                    
Description: Incremental Build Level control file.
=================================================================================  */


#ifndef PROJ_SETTINGS_H
#define PROJ_SETTINGS_H

#if (BUILDLEVEL  == LEVEL8) || (BUILDLEVEL == LEVEL9)
//#error  *** Cannot compile, contact TI sales for HFI/IPD library !! ***
#endif  // BUILDLEVEL

//------------------------------------------------------------------------------

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)

#define SYSTEM_FREQUENCY 60


// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 8

// Define the offset filter corner frequency (in Hz)
#define  OFFSET_FILTER_CORNER_FREQ    20    /*  in Hz  */


// Define the electrical motor parametes (Estun Servomotor)
#define RS 		1.2		    	    // Stator resistance (ohm) 2.35//0.59
#define RR   			               	// Rotor resistance (ohm) 
#define LS   	0.024				// Stator inductance (H)45//3
#define LR   			  				// Rotor inductance (H) 	
#define LM   			   				// Magnatizing inductance (H)
#define POLES  	10						// Number of poles

// Define the base quantites
#define BASE_VOLTAGE    1000        // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_CURRENT    20            // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_TORQUE     		      // Base torque (N.m)
#define BASE_FLUX       			  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	100           // Base electrical frequency (Hz)

#define  TRANS_SPD_HI     _IQ(0.12)//0.12
#define  TRANS_SPD_LO     _IQ(0.05)//0.06
#define  TRANS_SCALE      12    //10      /* must be < 1/(HI-LO) such that max k2 ~= 0.8 */

#define  INDEX_CNT        8
#define  VOLT_PU(A)       _IQ( A/(sqrt(3)*600) )//BASE_VOLTAGE
#define  HFI_VOLT_RUN     50.0        /* max 50duty cycle below this Vdc (in V) @ RUN */
#define  HFI_VOLT_IPD     80.0        /* max 80duty cycle below this Vdc (in V) @ IPD */


#endif

