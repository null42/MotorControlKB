/*
 * interrupt.c
 *
 *  Created on: 28 Mar 2018
 *      Author: LMD
 */

#include "DSP28x_Project.h"
#include "application.h"
#include "device.h"
#include "communication.h"
#include "control.h"
#include "alarm.h"
#include "esmohfi.h"
//#include "voliate.h"
//#pragma DATA_SECTION(TestDataBuf,"testdata");
//Uint16 TestDataBuf[2048];
_iq Vbus = 0,ipd_position[4]={0,0,0,0,};
_iq SpeedRef=_IQ(0.15),id_target=0;
extern _iq  startCnt,smoK_set;   // sliding mode gain - final value//0.5
Uint16 lsw=0,speed_state=0,SpeedLoopCount=0,relay_start_flag=0,ipd_fail=0;
extern Uint8 START;
int transition_count=0,SpeedLoopPrescaler=5,ipd_count=0,ipd_again=0,dingwei_count=0,relay_count=0;
extern int16 can_start_flag,can_start_flag1;
extern float32 T;
Uint16 start_flag=0;
extern RMPCNTL rc1;
extern SPEED_ESTIMATION speed3;
extern ESMOPOS  esmo1;
extern SMOPOS_CONST smo1_const;
extern TRANSITION  transition1;
extern HFI hfi1;
// Instance a high pass filter
extern HPF  hpf_Iq;
// Instance a high pass filter coefficients structure
extern HPF_COEFF   hpf_coeff1;
// Instance a High Frequency Injection controller
// Instance a magnetic polarity identifier
extern NS_ID  ns_id1;
extern PWMGEN1 pwm1;
extern PI_CONTROLLER pi_spd;
extern PI_CONTROLLER pi_id;
extern PI_CONTROLLER pi_iq;
extern RAMPGEN rg1;
extern CLARKE clarke1;
extern PARK park1;
extern IPARK ipark1;
extern PHASEVOLTAGE volt1;
extern DBC dbc1;
extern SVGEN svgen1;


#pragma CODE_SECTION(cpu_timer0_isr,"ramfuncs");
interrupt void cpu_timer0_isr(void)
{
	SystemParameter.Timer.Timer1msIndex++;
	SystemParameter.Timer.Timer20msIndex++;
	SystemParameter.Timer.Timer1SIndex++;
	ModbusParameter.Char_Switch_time++ ;
	if(ModbusParameter.Char_Switch_time >= 5)
	{
	   ModbusParameter.Char_Switch_time		= 0;
	   SciaParameter.rx_index				= 0;
	}

	EINT;
	FaultProtection_Fun();
	StateMachine_Fun();

	DINT;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



#pragma CODE_SECTION(adc_svpwm_isr1,"ramfuncs");
interrupt void adc_svpwm_isr1(void)
{// ------------------------------------------------------------------------------
    //  Connect inputs of the RMP module and call the ramp control macro
    // ------------------------------------------------------------------------------
    //  Measure the DC bus voltage in _IQ format
    // ------------------------------------------------------------------------------
        //  Measure the DC bus voltage in _IQ format
        // ------------------------------------------------------------------------------
        //  volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);
         //   volt1.DcBusVolt = ((AdcResult.ADCRESULT3)*_IQ(-0.000341))+_IQ(1.2631);
        // ------------------------------------------------------------------------------
        //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
        //  Connect inputs of the CLARKE module and call the clarke transformation macro
        // ------------------------------------------------------------------------------
        ADValue.Iu                          = AdcResult.ADCRESULT0;
        ADValue.Iv                          = AdcResult.ADCRESULT1;
        ADValue.Iw                          = AdcResult.ADCRESULT2;
        ADValue.Vbus                        = AdcResult.ADCRESULT3;

        clarke1.As                 = _IQmpyI32(ACCURRENTTRANSFE,AdcResult.ADCRESULT0 - ADCalibration.AdOffset.Iu);       //
        clarke1.Bs                 = _IQmpyI32(ACCURRENTTRANSFE,AdcResult.ADCRESULT1 - ADCalibration.AdOffset.Iv);       //
        clarke1.Cs                 = _IQmpyI32(ACCURRENTTRANSFE,AdcResult.ADCRESULT2 - ADCalibration.AdOffset.Iw);
//        clarke1.As =_IQmpy((_IQ12toIQ(ADValue.Iu -ADCalibration.AdOffset.Iu)),_IQ(1.225));//-2048//_IQ(2.49795)//20
//        clarke1.Bs=_IQmpy((_IQ12toIQ(ADValue.Iv -ADCalibration.AdOffset.Iv)),_IQ(1.225));//-2048//_IQ(2.49795)
//        clarke1.Cs =_IQmpy((_IQ12toIQ(ADValue.Iw -ADCalibration.AdOffset.Iw)),_IQ(1.225));//-2048//_IQ(2.49795)

        volt1.DcBusVolt = _IQ(0.33);
        Vbus                                = _IQmpyI32(DCVOLAGETRANSFE,ADValue.Vbus);
        SvpwmParameter.Vbus                 = SvpwmParameter.Vbus + _IQmpy(_IQ(0.02),(Vbus - SvpwmParameter.Vbus));
//        volt1.DcBusVolt = SvpwmParameter.Vbus;

         if(( _IQabs(clarke1.As)>_IQ(1.2))|( _IQabs(clarke1.Bs)>_IQ(1.2))|( _IQabs(clarke1.Cs)>_IQ(1.2))&&(start_flag==1))
             {
                 Fault.type.error.bit.IoutOverIns    = TRUE;
             }


         if(0 != Fault.all)
            {
                DisablePwm();
                SystemParameter.State.PwmState = pwm_off;
            }

        CLARKE_MACRO(clarke1)
        // ------------------------------------------------------------------------------
        //  Connect inputs of the PARK module and call the park trans. macro
        // ------------------------------------------------------------------------------

#if (STEP_BUILD == CURRENT_LOOP)

    if(SystemParameter.State.PwmState == pwm_on)
    {
        if(lsw == 0)
        {
            rc1.TargetValue = 0;
            rc1.SetpointValue = 0;
            park1.Angle = 0;
            // reset all PI regulators
            dbc1.gain  = 0;
            pi_iq.Ref = _IQ(0.1);
            pi_iq.Fbk = park1.Qs;
            PI_MACRO(pi_iq);

            pi_id.Ref = _IQ(0.0);
            pi_id.Fbk = park1.Ds;
            PI_MACRO(pi_id);

        }else
        {
            pi_iq.Ref = _IQ(0.1);
            pi_iq.Fbk = park1.Qs;
            PI_MACRO(pi_iq);

            pi_id.Ref = _IQ(0.0);
            pi_id.Fbk = park1.Ds;
            PI_MACRO(pi_id);
            RC_MACRO(rc1)
            rc1.TargetValue = SpeedRef;
            RG_MACRO(rg1)
            rg1.Freq = rc1.SetpointValue;
            ANGLE_WRAP(rg1.Out);
            park1.Angle=rg1.Out;
        }




    }else
    {
         RESET_PI(pi_id);
         RESET_PI(pi_iq);
         rc1.TargetValue = 0;
         rc1.SetpointValue = 0;

    }

        if(rc1.SetpointValue>_IQ(0.15))
        rc1.RampDelayMax=5;
        else
        rc1.RampDelayMax=5;
        RESET_PI(pi_spd)
    // ------------------------------------------------------------------------------

    // ------------------------------------------------------------------------------
    // State Machine for motor control
    // -----------------------------------------------------------------------------
    // ------------------------------------------------------------------------------
    //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
    // ------------------------------------------------------------------------------

#endif

#if (STEP_BUILD == IPD_LOOP)

        if(lsw != RUN_STATE)
        {
            rc1.TargetValue = 0;
            rc1.SetpointValue = 0;
            speed_state=0;
            id_target =_IQ(0);
            pi_id.Ref=_IQ(0);
        }
        if(lsw == RUN_STATE)
        {
            if(SpeedRef>_IQ(0.12)&&speed_state==0)
              {
                rc1.TargetValue = _IQ(0.1);
                transition_count++;
                if(transition_count>20000)
                   {
                       rc1.TargetValue = SpeedRef;
                       transition_count=0;
                       speed_state=1;
                   }
                 }
               else
            rc1.TargetValue = SpeedRef;
        }

        RC_MACRO(rc1)    //¼Ó¼õËÙ

        if(rc1.SetpointValue>_IQ(0.15))
        rc1.RampDelayMax=5;
        else
        rc1.RampDelayMax=5;

        if(lsw==0) park1.Angle = 0;
        else park1.Angle  = speed3.EstimatedTheta;



    if(SystemParameter.State.PwmState == pwm_on)
    {
        if(lsw == 0)
        {
            rc1.TargetValue = 0;
            rc1.SetpointValue = 0;
            park1.Angle = 0;

            dbc1.gain  = 0;
            pi_iq.Ref = _IQ(0.1);
            pi_iq.Fbk = park1.Qs;
            PI_MACRO(pi_iq);

            pi_id.Ref = _IQ(0.0);
            pi_id.Fbk = park1.Ds;
            PI_MACRO(pi_id);

            HFI_RESET(&hfi1);          // reset HFI module
            HPF_RESET(&hpf_Iq);        // reset HPF
            HPF_INIT(&hpf_coeff1);

            RESET_PI(pi_spd)
            ipd_count=0;
        }
        else if (lsw == IPD_STATE)
        {
            ZLSPD(&hpf_Iq, &hpf_coeff1, &hfi1, &ns_id1, &clarke1, &volt1);
            pi_id.Out = hfi1.duty;
            pi_iq.Out = 0;
            RESET_PI(pi_spd);

            if(hfi1.HFI_Status == HFI_RUN_STATE)
            {
                lsw = RUN_STATE;
                pi_id.Out = 0;
            }
            else if(hfi1.HFI_Status == HFI_RESET_STATE)
            {
                lsw = RESET_STATE;
                GpioDataRegs.GPASET.bit.GPIO24=1;
                ipd_fail=1;
            }
            if(hfi1.HFI_Status == HFI_NSID_STATE)
            {
                ipd_count++;
                if(ipd_count>500)
                {
                    //hfi1.HFI_Status=HFI_RUN_STATE;
                    ipd_again++;
                    if(ipd_again<1)
                    {
                        //low_Vflagcount=201;V_lowcount=3001;V_able=0;low_Vflag=1;
                        ipd_position[ipd_again-1]=hfi1.thetaEst;
                        lsw=0;
                    }
                    else
                    {
                        hfi1.HFI_Status=HFI_RUN_STATE;
                        //hfi1.Squ_PRD_set  = 8;//INDEX_CNT;        /* 16 IPD // 8 low speed for Bosch // 1 for E-bike *///16
                        //hfi1.dutyMax  = _IQ(0.9);
                        ipd_position[ipd_again-1]=hfi1.thetaEst;
                        if(hfi1.thetaEst>_IQ(0.5))
                            hfi1.thetaEst-=_IQ(0.5);
                        ipd_position[ipd_again]=hfi1.thetaEst;
                        ipd_again=0;
                        rc1.SetpointValue=_IQ(0.000);
                        pi_spd.Out=_IQ(0.4);
                        pi_spd.ui=_IQ(0.4);
                        pi_spd.i1=_IQ(0.4);
                    }
                    ipd_count=0;
                    rg1.Out=hfi1.thetaEst;
                }
            }
        }
        else if (lsw == RUN_STATE)
        {
            // ------------------------------------------------------------------------------
            //    Connect inputs of the PI module and call the PID speed controller macro
            // ------------------------------------------------------------------------------
            if (++SpeedLoopCount >= SpeedLoopPrescaler)
            {
                pi_spd.Ref = rc1.SetpointValue;
                pi_spd.Fbk = speed3.EstimatedSpeed;
                PI_MACRO(pi_spd);
                SpeedLoopCount=0;
            }
            // ------------------------------------------------------------------------------
            //    Connect inputs of the PI module and call the PID IQ controller macro
            // ------------------------------------------------------------------------------
            pi_iq.Ref = pi_spd.Out;
            pi_iq.Fbk = park1.Qs;
            PI_MACRO(pi_iq);
            // ------------------------------------------------------------------------------
            //    Connect inputs of the PI module and call the PID ID controller macro
            // ------------------------------------------------------------------------------
      //    if(rc1.SetpointValue>_IQ(0.2))
      //    {
      //        pi_id.Ref =pi_iq.Ref*_IQ(-0.2) ;
      //    }else

            if(rc1.SetpointValue< _IQ(0.15))
      //    pi_id.Ref = 0;
            id_target =_IQ(0);
            else
            id_target=_IQmpy(pi_iq.Ref,_IQ(-0.226));

            if(_IQabs(id_target-pi_id.Ref)>0.00003&&rc1.SetpointValue<_IQ(0.55))
            {
                 if(id_target>pi_id.Ref)pi_id.Ref+=_IQ(0.00002);
                 if(id_target<pi_id.Ref)pi_id.Ref-=_IQ(0.00002);
            }
            else
            {
                 pi_id.Ref = id_target;
            }

            pi_id.Fbk = park1.Ds;
            PI_MACRO(pi_id);

            // Zero and low speed position detection
            if ( _IQabs(rc1.SetpointValue) <= transition1.spdHi )
            {
              ZLSPD(&hpf_Iq, &hpf_coeff1, &hfi1, &ns_id1, &clarke1, &volt1);
              pi_id.Out += hfi1.duty;
            }
            dbc1.gain  = _IQ(0.5);             // comment to tune in debug environment
        }
    }else
    {
        DisablePwm();


        EPwm1Regs.CMPA.half.CMPA            = 500;
        EPwm2Regs.CMPA.half.CMPA            = 500;
        EPwm3Regs.CMPA.half.CMPA            = 500;

    }

#endif

    // ------------------------------------------------------------------------------
    //    Connect inputs of the VOLT_CALC module and call the phase voltage macro
    // ------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------
    //    Connect inputs of the SPEED_EST module and call the estimated speed macro
    // ------------------------------------------------------------------------------
        transition1.spd      = rc1.SetpointValue;
        transition1.angleHFI = hfi1.thetaEst;
        transition1.angleSMO = esmo1.Theta;
        ANGLE_TRANSIT(&transition1);

        speed3.EstimatedTheta = transition1.angle;

        if (lsw !=2)
            speed3.OldEstimatedTheta = speed3.EstimatedTheta;
        SE_MACRO(speed3)

    // ------------------------------------------------------------------------------


        park1.Alpha  = clarke1.Alpha;
        park1.Beta   = clarke1.Beta;
        park1.Sine   = _IQsinPU(park1.Angle);
        park1.Cosine = _IQcosPU(park1.Angle);
        PARK_MACRO(park1)
        hpf_Iq.In1 = park1.Qs;

        ipark1.Ds     = pi_id.Out;
        ipark1.Qs     = pi_iq.Out;
        ipark1.Sine   = park1.Sine;
        ipark1.Cosine = park1.Cosine;
        IPARK_MACRO(ipark1)

        svgen1.Ualpha = ipark1.Alpha;
        svgen1.Ubeta  = ipark1.Beta;
        SVGENDQ_MACRO(svgen1)

        //  volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);
        volt1.MfuncV1 = svgen1.Ta;
        volt1.MfuncV2 = svgen1.Tb;
        volt1.MfuncV3 = svgen1.Tc;
        PHASEVOLT_MACRO(volt1)
    // ------------------------------------------------------------------------------
    //    Connect inputs of the eSMO_POS module and call the eSMO module
    // ------------------------------------------------------------------------------
        if ((lsw == RUN_STATE)&&(rc1.SetpointValue>transition1.spdHi) && (esmo1.Kslide  < smoK_set))
    //  if ((lsw == RUN_STATE) && (esmo1.Kslide  < smoK_set))
        esmo1.Kslide += _IQ(0.00005);//_IQ(0.00001);
        esmo1.Ialpha   = clarke1.Alpha;
        esmo1.Ibeta    = clarke1.Beta;
        esmo1.Valpha   = volt1.Valpha;
        esmo1.Vbeta    = volt1.Vbeta;
        esmo1.runSpeed = speed3.EstimatedSpeed;
        esmo1.cmdSpeed = rc1.SetpointValue;
        eSMO_MODULE(&esmo1);



 //       if (lsw != RUN_STATE)   dbc1.gain = 0;
    //  else          dbc1.gain = _IQ(0.5);         // comment line to tune in debug
        pwm1.MfuncC1 = svgen1.Ta;
        pwm1.MfuncC2 = svgen1.Tb;
        pwm1.MfuncC3 = svgen1.Tc;

        if (hfi1.HFI_Status != HFI_NSID_STATE)      // not NSID state
        {
            /***********************************************/
  //          PWMwDBC_MACRO(1,2,3,pwm1,clarke1,dbc1);
            EPwm1Regs.CMPA.half.CMPA = _IQmpy((EPwm1Regs.TBPRD>>1),pwm1.MfuncC1)+ (EPwm1Regs.TBPRD>>1);
            EPwm2Regs.CMPA.half.CMPA = _IQmpy((EPwm2Regs.TBPRD>>1),pwm1.MfuncC2)+ (EPwm2Regs.TBPRD>>1);
            EPwm3Regs.CMPA.half.CMPA = _IQmpy((EPwm3Regs.TBPRD>>1),pwm1.MfuncC3)+ (EPwm3Regs.TBPRD>>1);


        }else
        {
            dingwei_count++;
            if(dingwei_count>20000)
                dingwei_count=0;
    //      GpioDataRegs.GPACLEAR.bit.GPIO22=1;
            pwm1.MfuncC1 = 0;
            pwm1.MfuncC2 = 0;
            pwm1.MfuncC3 = 0;
            dbc1.scale = _IQdiv(_IQdiv2(pwm1.Deadband), dbc1.Ith);  // uncomment to tune in debug
            dbc1.Kdtc  = _IQmpy(dbc1.scale, dbc1.gain);
            EPwm1Regs.CMPA.half.CMPA = _IQmpy((EPwm1Regs.TBPRD>>1),pwm1.MfuncC1)+ (EPwm1Regs.TBPRD>>1);
            EPwm2Regs.CMPA.half.CMPA = _IQmpy((EPwm2Regs.TBPRD>>1),pwm1.MfuncC2)+ (EPwm2Regs.TBPRD>>1);
            EPwm3Regs.CMPA.half.CMPA = _IQmpy((EPwm3Regs.TBPRD>>1),pwm1.MfuncC3)+ (EPwm3Regs.TBPRD>>1);   // uncomment to tune  "  "

        }

//        CurrentOverRead_Fun();

                   if(can_start_flag==0)
                      {
            //            start_flag=0;
                         // speed_able=0;
                        //  if(SpeedRef>_IQ(0.1))SpeedRef-=_IQ(0.00003);
                        //  if(SpeedRef<_IQ(0.1))
                         SpeedRef=_IQ(0.0);
            //            rc1.RampDelayMax=2;
                          //else  speed_able=0;
            //            if(lsw1==1)
                          Fault.type.error.bit.TigbtOver=1;
            //            if(fabs(speed3.EstimatedSpeed)<_IQ(0.1))
            //                {
            //                   speed_able=0;
            //                }
                      }else
                          Fault.type.error.bit.TigbtOver=0;


        AdcRegs.ADCINTFLGCLR.bit.ADCINT2        = 1;        //Clear ADCINT2 flag reinitialize for next SOC
        PieCtrlRegs.PIEACK.all                  = PIEACK_GROUP1;   // Acknowledge interrupt to PIE




}








