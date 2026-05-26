
#ifndef  __F2803xDRVLIB_H__
#define __F2803xDRVLIB_H__


// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] = 
 				  { &EPwm1Regs,			//intentional: (ePWM[0] not used)
				  	&EPwm1Regs,
					&EPwm2Regs,
					&EPwm3Regs,
					&EPwm4Regs,
					&EPwm5Regs,
					&EPwm6Regs,
					&EPwm7Regs,
					
				  };



				  
#endif	// __F2803xDRVLIB_H__




