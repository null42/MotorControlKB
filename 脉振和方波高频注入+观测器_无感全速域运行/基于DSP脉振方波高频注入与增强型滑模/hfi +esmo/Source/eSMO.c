/* ==============================================================================
System Name:  	Enhanced Sliding Mode Observer for PMSM

File Name:	  	eSMO.C

Description:	
=================================================================================  */

#include "esmopos.h"
#include "angle_math.h"
#include "pi.h"

#define PI 3.14159265358979

/********************************************
 * eSMO module
 * ******************************************/
void eSMO_MODULE(ESMOPOS *v)
{
    /*	Sliding mode current observer	*/
	v->EstIalpha = _IQmpy(v->Fsmopos,v->EstIalpha) + _IQmpy(v->Gsmopos,(v->Valpha-v->Ealpha-v->Zalpha));
	v->EstIbeta  = _IQmpy(v->Fsmopos,v->EstIbeta)  + _IQmpy(v->Gsmopos,(v->Vbeta -v->Ebeta -v->Zbeta ));

	/*	Current errors	*/
    v->IalphaError = v->EstIalpha - v->Ialpha;
    v->IbetaError  = v->EstIbeta  - v->Ibeta;

	/*  Sliding control calculator	*/
	/* v->Zalpha=v->IalphaError*v->Kslide/v->E0) where E0=0.5 here*/
	v->Zalpha = _IQmpy(_IQsat(v->IalphaError,v->E0,-v->E0),_IQmpy2(v->Kslide));
	v->Zbeta  = _IQmpy(_IQsat(v->IbetaError ,v->E0,-v->E0),_IQmpy2(v->Kslide));

	/*	Sliding control filter -> corner frequency calculator	*/
	v->smoFreq   = _IQabs(_IQmpy2(v->runSpeed));
	if (v->smoFreq < _IQ(0.2)) v->smoFreq = _IQ(0.2);
	v->Kslf = _IQmpy(v->smoFreq, _IQmpy(_IQ(2*PI),v->base_wTs));      // Need 2.pi.F for phase shift

	/*	Sliding control filter -> back EMF calculator	*/
    v->Ealpha = v->Ealpha + _IQmpy(v->Kslf,(v->Zalpha-v->Ealpha));
    v->Ebeta  = v->Ebeta  + _IQmpy(v->Kslf,(v->Zbeta -v->Ebeta));

	/*	Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta) + SMO angle comp + DIR angle comp	*/
	v->smoShift = _IQatan2PU(v->runSpeed, v->smoFreq);                 /* smo filter angle comp */
	v->Theta = _IQatan2PU(-v->Ealpha,v->Ebeta) + v->smoShift;
	if (v->cmdSpeed < 0) v->Theta -= _IQ(0.5);                         /* direction angle comp  */
	ANGLE_WRAP(v->Theta);

	return;
}

/******************************************
 * Angle filter - PI controller
 ******************************************/
void anglePI(PI_CONTROLLER *v)
{
	/* proportional term */
	v->up = v->Ref - v->Fbk;
	ERROR_ANGLE_WRAP(v->up); 	        /* roll in the error */

	/* integral term */
	v->up = _IQmpy(v->Kp, v->up);
	v->ui = (v->Out == v->v1)?(_IQmpy(v->Ki, v->up)+ v->i1) : v->i1;
	v->i1 = v->ui;

	/* control output */
	v->v1 = v->up + v->ui;
	v->Out= _IQsat(v->v1, v->Umax, v->Umin);

	return;
}

/*****************************
 * Angle filter
 *****************************/
_iq angleFilter(PI_CONTROLLER *v, ESMOPOS *s)
{
	v->Ref  = s->Theta;
	v->Fbk += _IQmpy(v->Out, s->base_wTs);    /* Fbk = integral of speed     */
	ANGLE_WRAP(v->Fbk);                       /* roll "Fbk" within -pi to pi */
	anglePI(v);

	return(v->Fbk);
}

//===========================================================================
// End of file
//===========================================================================
