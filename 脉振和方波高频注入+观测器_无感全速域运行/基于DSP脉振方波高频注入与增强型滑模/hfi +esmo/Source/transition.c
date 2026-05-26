/* =================================================================================
File name:       TRANSITION.C
==================================================================================*/
#include "transition.h"

/*------------------------------------------------------------------------------
Prototypes for the function
------------------------------------------------------------------------------*/

void  ANGLE_TRANSIT(TRANSITION * v)      {
	_iq Ksmo, error, spd = _IQabs(v->spd);

	if (spd >= v->spdHi)
		v->angle = v->angleSMO;

	else if (spd >= v->spdLo)
	{
		error = v->angleHFI - v->angleSMO;
		if      (error < -_IQ(0.5))  v->angleHFI += _IQ(1.0);
		else if (error >  _IQ(0.5))  v->angleSMO += _IQ(1.0);
		Ksmo = (spd - v->spdLo)*v->scale;
		/*Ksmo = _IQdiv(_IQmpy((spd - v->spdLo), v->scale), (v->spdHi - v->spdLo));     */
		v->angle = _IQmpy(v->angleHFI, _IQ(1.0) - Ksmo) +
                  _IQmpy(v->angleSMO, Ksmo);
		if (v->angle  > _IQ(1.0))
		    v->angle -= _IQ(1.0);
	}

	else
	    v->angle = v->angleHFI;
}

