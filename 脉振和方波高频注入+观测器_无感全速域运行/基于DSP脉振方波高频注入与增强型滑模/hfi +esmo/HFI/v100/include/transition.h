/* =================================================================================
File name:       TRANSITION.H
==================================================================================*/
#ifndef __TRANSITION_H__
#define __TRANSITION_H__

#include "IQmathlib.h"

typedef struct {
	_iq spd,
	    spdLo,
	    spdHi,
		angleHFI,
		angleSMO,
		angle,
		scale;
} TRANSITION;

typedef TRANSITION * TRANSITION_handle;

/*-----------------------------------------------------------------------------
	Default initializer for TRANSITION object.
-----------------------------------------------------------------------------*/
#define TRANSITION_DEFAULTS     \
{                               \
	0,                          \
	_IQ(0.06),                  \
	_IQ(0.10),                  \
	0,                          \
	0,                          \
	0,                          \
    20                          \
}

void  ANGLE_TRANSIT(TRANSITION_handle v) ;

#endif
