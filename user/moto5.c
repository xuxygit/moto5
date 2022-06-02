/**********************************************************************

************************************************************************/

#include "common.h"
#include "uart.h"
#include "sysinit.h"
#include "hwconfig.h"
#include "statemachine.h"

int main(void);

/*****************************************************************************
* system initialization
* MCU init, PWM,CMT,ADC,PIT, KBI and UART
* App vatiaable initialzation
* run in state machine loop
	-init state-Stop-Alignment-Start Up-Open loop-Shift vector-Run-Fault-Restart
*****************************************************************************/
int main(void)
{
	sysinit();
	MotorInit();
	HWConfig();

	while (1)
	{
		// StateMachine();
		BtnPolling();
	}
}
