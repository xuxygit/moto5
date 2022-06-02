
/******************************************************************************
 *
 * @brief callback function
 * @brief   Interrupt Service Routines.
 ******************************************************************************/
#include "hwconfig.h"
#include "motoctrl.h"
#include "statemachine.h"
#include "motoparam.h"

extern SysStatus_t status;
extern Flags_t flags;
extern Motor_Ctrl_t motor_ctrl;
extern const PWMCtrl_t CMTTable[8];

uint8_t key_push_times = 0;

/**********************************************************************
 *  KBI1_Task
 *	response to user button push
 *	set system Power On/Off and motor speed as perdefine
 **********************************************************************/
void KBI1Task(void)
{
	key_push_times++;

	if (key_push_times >= 6)
		key_push_times = 0;
	switch (key_push_times)
	{
	case 1:
		//			PowerOn();
		break;
	case 2:
		GPIO_PinToggle(LED1);
		MotorSetSpeed(TARGET_SPEED25);
		MotorStart();
		break;
	case 3:
		GPIO_PinToggle(LED1);
		GPIO_PinToggle(LED2);
		MotorSetSpeed(TARGET_SPEED30);
		break;
	case 4:
		GPIO_PinToggle(LED2);
		GPIO_PinToggle(LED3);
		MotorSetSpeed(TARGET_SPEED35);
		break;
	case 5:
		GPIO_PinToggle(LED3);
		GPIO_PinToggle(LED4);
		MotorSetSpeed(TARGET_SPEED40);
		break;
	case 6:
		GPIO_PinSet(LED1);
		GPIO_PinSet(LED2);
		GPIO_PinSet(LED3);
		GPIO_PinSet(LED4);
		MotorStop();
		//			PowerOff();
		break;
	default:
		GPIO_PinSet(LED1);
		GPIO_PinSet(LED2);
		GPIO_PinSet(LED3);
		GPIO_PinSet(LED4);
	}
}

/**********************************************************************
 *  BtnPolling
 *	response to user button push
 *	set system Power On/Off and motor speed as perdefine
 **********************************************************************/
void BtnPolling(void)
{
	if (GPIO_BitRead(GPIO_PTD0))
	{
		key_push_times++;

		if (key_push_times >= 6)
			key_push_times = 0;
		GPIO_PinClear(GPIO_PTD0);
	}
	switch (key_push_times)
	{
	case 1:
		//			PowerOn();
		break;
	case 2:
		GPIO_PinToggle(LED1);
		MotorSetSpeed(TARGET_SPEED25);
		MotorStart();
		break;
	case 3:
		GPIO_PinToggle(LED1);
		GPIO_PinToggle(LED2);
		MotorSetSpeed(TARGET_SPEED30);
		break;
	case 4:
		GPIO_PinToggle(LED2);
		GPIO_PinToggle(LED3);
		MotorSetSpeed(TARGET_SPEED35);
		break;
	case 5:
		GPIO_PinToggle(LED3);
		GPIO_PinToggle(LED4);
		MotorSetSpeed(TARGET_SPEED40);
		break;
	case 6:
		GPIO_PinSet(LED1);
		GPIO_PinSet(LED2);
		GPIO_PinSet(LED3);
		GPIO_PinSet(LED4);
		MotorStop();
		//			PowerOff();
		break;
	default:
		GPIO_PinSet(LED1);
		GPIO_PinSet(LED2);
		GPIO_PinSet(LED3);
		GPIO_PinSet(LED4);
	}
}

/**********************************************************************
 * save phase change timer
 * PWM UVW phase change
 * calc BEMF sensor delay
 * set next phase change time
 * set ADC BEMF channel
 * Next sector;
 * in startup or open loop time force Motor PWM commutation
 **********************************************************************/
void CMTTimerTask(void)
{

	int16_t kpiout, duty;
	//	ETM_ClrChannelFlag(CMTTIMER,ETM_CHANNEL_CHANNEL0);
	ETM_ClrOverFlowFlag(CMTTIMER);

	motor_ctrl.CMT_step_count++;

	motor_ctrl.phase_sector = NextSector(motor_ctrl.phase_sector);
	SetPWMPhase(motor_ctrl.phase_sector);

	// LED just for test
	if (motor_ctrl.CMT_step_count == 1000)
	{
		GPIO_PinToggle(LED1);
		motor_ctrl.CMT_step_count = 0;
	}

	//	speed feedback and adjust
	motor_ctrl.speed_fb = MotorGetmSpeed();
	kpiout = SpeedKPI(motor_ctrl.speed_fb, motor_ctrl.speed_mech);
	duty = motor_ctrl.duty_cycle + kpiout / 100; // TODO: coefficients?
	SetPWMDutyCycle(duty);
}

/**********************************************************************
 * read BEMF value
 * calc Non_engined phase cross zero point
 * save ZCT event timer
 * Calculate time for commute
 * set ADC channel
 * set and save second AD value
 *
 **********************************************************************/
void ADCTask(void)
{
	/* heartbeat counting*/
	motor_ctrl.PWM_step_count++; // +1 in every PWM period

	// every PWM period check state machine;
	StateMachine();

	// every 2 * PWM period check zero cross point;
	if ((status == SPIN || status == OPENLOOP || status == RUN) &&
		(motor_ctrl.PWM_step_count % 2 == 1))
	{
		if (motor_ctrl.Bemf_volt_index >= 3)
			MotorZCDetecting();
	}

	// every 1ms @ PWM period @16Khz; check over current
	if ((status == SPIN || status == RUN) &&
		(motor_ctrl.PWM_step_count % 16 == 2))
	{
		MotorOverCurrent();
	}

	// every 16ms @ PWM period @16Khz; check speed
	if ((status == RUN) && (motor_ctrl.PWM_step_count % 128 == 3))
	{
		MotorSpeedKPI();
	}

	//  about 25% of CMT_period from demag delay  status != OPENLOOP || status != RUN ||
	if (CMTTIMER->CNT < (CMTTIMER->MOD >> 3)) // no need run adc ISR
	{
		if (NVIC_GetPendingIRQ(ADC0_IRQn))
			NVIC_ClearPendingIRQ(ADC0_IRQn);
		ADC->SC1 |= ADC_SC1_AIEN_MASK;
		ADC->SC2 |= ADC_SC2_ADTRG_MASK;
		NVIC_EnableIRQ(ADC0_IRQn);
		return;
	}

	// Read out one of three phase BEMF value
	/* NOTE!
	Since sample on PWM ON slot, the BEMF readint out  are Vbemf+1/2*Vdbus;
	if POW1P1N, bemf_volt is BEMF+Vdbus/2,
	if POW1P2N, bemf_volt is BEMF+Bdbus/3; */
	motor_ctrl.Bemf_volt[motor_ctrl.Bemf_volt_index] = ADC->R; // V no-power phase

	// Disable interupt and hardware trig,Set second ADC for  powered phase
	ADC->SC1 &= ~ADC_SC1_AIEN_MASK;
	ADC->SC1 |= CMTTable[motor_ctrl.phase_sector].swap;
	while (!(ADC->SC1 & ADC_SC1_COCO_MASK))
	{
	};
	motor_ctrl.Bemf_volt[motor_ctrl.Bemf_volt_index] -= ADC->R;
	motor_ctrl.Bemf_volt[motor_ctrl.Bemf_volt_index] += 4096; // 4096=2^12, 12bits;

	motor_ctrl.Bemf_volt_index++;
	if (motor_ctrl.Bemf_volt_index > 5)
		motor_ctrl.Bemf_volt_index = 0;

	// clear, if other process take more time than one pwm period;
	if (NVIC_GetPendingIRQ(ADC0_IRQn))
		NVIC_ClearPendingIRQ(ADC0_IRQn);
	// set next ADC channel and  enable HW trig and interupt
	ADC->SC1 |= CMTTable[motor_ctrl.phase_sector].adchannel;
	ADC->SC2 |= ADC_SC2_ADTRG_MASK;
	ADC->SC1 |= ADC_SC1_AIEN_MASK;
	NVIC_EnableIRQ(ADC0_IRQn);

	if (!(motor_ctrl.PWM_step_count >> 10)) // NOTE:~1s@16khz, just for debug show the adc runing;
	{
		GPIO_PinToggle(LED3);
	}
}

/**********************************************************************
 * Calc motor speed as per ZC
 * Motor speed and/o current loop PI regulate
 * Set PWM duty cycle
 * open loop speed ramp
 * trigger ADC
 * fault detect
 **********************************************************************
void PITTask(void)
{
	PIT_ChannelClrFlags(PIT_CHANNEL0);
	NVIC_EnableIRQ(PIT_CH0_IRQn);
	msTicks++;
	if (msTicks > 100)
	{
		msTicks = 0;
		GPIO_PinToggle(LED3);
	}
}
*/
/**********************************************************************
 * 1 ms loop control
 * Motor speed and/o current loop PI regulate
 * fault detect
 * control PWMTIMER dutycycle for Motor Speed control
 * as PIT could not work with ETM0&ETM1, using RTC
 ***********************************************************************/
void RTCTask(void)
{
	int16_t kpiout, duty;

	motor_ctrl.Ms_count++;
	if (motor_ctrl.Ms_count > 1000)
	{
		GPIO_PinToggle(LED2);
		motor_ctrl.Ms_count = 0;
	}

	//	speed feedback and adjust
	motor_ctrl.speed_fb = MotorGetmSpeed();
	kpiout = SpeedKPI(motor_ctrl.speed_fb, motor_ctrl.speed_mech);
	duty = motor_ctrl.duty_cycle + kpiout / 100; // TODO: coefficients?
	SetPWMDutyCycle(duty);
}
