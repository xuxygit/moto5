#include "motoctrl.h"
#include "motoparam.h"
#include <common.h>

extern MotoCtrl_t MotoCtrl;

/******************************************************************************
*  Calc the root of input, Using Newton way;
*  Since the input is max 0xFFFF, and only inter output need, take easy way;
*******************************************************************************/
uint16_t MinSqrt(uint16_t in)
{
	uint16_t rootlast,rootnew;
	
	rootlast = 256;
	
	for(;;)
	{
		rootnew = (rootlast + in/rootlast) / 2;
		if((rootlast - rootnew) > 1)
			rootlast = rootnew;
		else 	
			return ( rootnew );
	}
}

/******************************************************************************
*  Speed Loop KPI control;
*  input reauired speed and feedback speed;
*  output pwd duty cycle
*******************************************************************************/

int16 SpeedKPI(MotoCtrl_t MotoCtrl,uint16_t SpeedFb, uint16_t SpeedReq)
{
	static int32 delta, proportion,proportionlast, integral,integralsum,kpiout;
	float lowpassfactor = 0.7;
	
	integralsum = MotoCtrl.Integral_Sum;
	delta = SpeedFb - SpeedReq;
	
	/* TODO low pass fliter */ 
	proportion = (1 - lowpassfactor) * delta + lowpassfactor * proportionlast;
	proportionlast = proportion;

	proportion = MotoCtrl.KP * delta;
	integral = MotoCtrl.KI * delta;
	integralsum += integral;
	
	if( integralsum > (KI_DIV_VM * UPPER_OUT_LIMIT_VM) )
		integralsum = KI_DIV_VM * UPPER_OUT_LIMIT_VM;
	else if( integralsum < (- KI_DIV_VM * UPPER_OUT_LIMIT_VM))
		integralsum = - KI_DIV_VM * UPPER_OUT_LIMIT_VM;
	
	MotoCtrl.Integral_Sum = integralsum;
	kpiout = (proportion / KP_DIV_VM) + (integralsum / KI_DIV_VM);
	
	if(kpiout > UPPER_OUT_LIMIT_VM)
		kpiout = UPPER_OUT_LIMIT_VM;
	else if (kpiout < LOWER_OUT_LIMIT_VM)
		kpiout = LOWER_OUT_LIMIT_VM;
	
	return( (int16_t)kpiout);
	

}
