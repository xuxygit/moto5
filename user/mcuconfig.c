/*****************************************************************************
 * system initialization
 * MCU init, PWM,CMT,ADC,PIT, KBI and UART
 *
 *****************************************************************************/

#include "hwconfig.h"
#include "motoctrl.h"
/*
* MOSFET MASK, SWAP,ADC map in HW as:
* phase U,	V,	W output adc
	0	+   -	/         ch2
	1	+	/	-         ch1
	2	/	+	-         ch0
	3	-	+	/         ch2
	4	-	/	+         ch1
	5	/	-	+         ch0
	6	+	-	-
*/

const PWMCtrl_t CMTTable1P1N[8] = {
	// mask, swap,adcchnnel
	{0x30, 0x02, 0x04}, // [0] - sector 0
	{0x0C, 0x04, 0x02}, // [1] - sector 1
	{0x03, 0x04, 0x01}, // [2] - sector 2
	{0x30, 0x01, 0x04}, // [3] - sector 3
	{0x0C, 0x01, 0x02}, // [4] - sector 4
	{0x03, 0x02, 0x01}, // [5] - sector 5
	{0x00, 0x06, 0x00}, // [6] - alignment vector (combination of sectors 0 & 1)
	{0x3F, 0x00, 0x00}, // [7] - PWM Off
};

const PWMCtrl_t CMTTable1P2N[8] = {
	{0x1A, 0x10, 0x04}, // [0] - sector 0
	{0x16, 0x14, 0x04}, // [1] - sector 1
	{0x26, 0x04, 0x04}, // [2] - sector 2
	{0x25, 0x05, 0x04}, // [3] - sector 3
	{0x29, 0x01, 0x02}, // [4] - sector 4
	{0x19, 0x11, 0x01}, // [5] - sector 5
	{0x1A, 0x02, 0x00}, // [6] - alignment vector (combination of sectors 0 & 1)
	{0x3F, 0x00, 0x00}, // [7] - PWM Off
};

extern Motor_Ctrl_t motor_ctrl;

/*****************************************************************************
 *
 *  main initialization
 *  - Calls initialization functions of peripherals required for motor
 *  control functionality
 *
 *******************************************************************************/
void HWConfig(void)
{

	//	PMC_ConfigType  PMC_Config={0};

#ifdef UARTCOM
	/* UART1 init */
	UART1Init();
#endif
	/* keyboard  and LED init */
	KbLedInit();

	/* init 6 phase mosfet driver */
	PWMTimerInit();

	/* init communation driver */
	CMTTimerInit();

	/* Init PIT driver
		PITInit();
	*/
	/* ADC for BEMF sensor driver */
	ADC0Init();

	/* RTC for timer count
	RTCInit();
	*/
	/* PMC init */
	/*	PMC_Config.sCtrlstatus.bits.bBandgapEn = 0;
		PMC_Config.sCtrlstatus.bits.bLvdStopEn = 0; //????????????
		PMC_Config.sCtrlstatus.bits.bLvdRstEn = 0;  //????????
		PMC_Init(PMC, &PMC_Config);
		PMC_DisableLVWInterrupt(PMC);
		PMC_SetMode(PMC, PmcModeWait);
	*/
}

/**********************************************************************
	* initial UART1
	* baud rate 115200
	* send in poll way
	* Rev in interputer set call back -> UART1_RxTask

 **********************************************************************/
void UART1Init(void)
{

	UART_ConfigType uartConfig;
	uartConfig.u32SysClkHz = BUS_CLK_HZ;
	uartConfig.u32Baudrate = 115200;
	UART_Init(UART1, &uartConfig);

	UART_EnableRxBuffFullInt(UART1);
	NVIC_EnableIRQ(UART1_IRQn);
	NVIC_SetPriority(UART1_IRQn, 3);
	UART_SetCallback(UART1RxTask);
}

/**********************************************************************
 *  initial LED1-LED4 and Keyboard0
 *	LED0-4 show Speed Low,MiddleLow, Middle and High
 *  KBI for input , scan KBI power On/Off system and set speed
 *	function callback -> BI1_Task() scan keyboard.
 *	function BtnPolling scan keyboard as No ISP way.
 **********************************************************************/
void KbLedInit(void)
{

	/* ISP way */
	/*
	KBI_ConfigType sKBIConfig;
	uint8_t i;

	for (i = 0; i < KBI_MAX_PINS_PER_PORT; i++)
	{
		sKBIConfig.sPin[i].bEn = 0;
	}

	sKBIConfig.sBits.bMode = 0;
	sKBIConfig.sPin[0].bEdge = KBI_FALLING_EDGE_LOW_LEVEL;
	sKBIConfig.sBits.bIntEn = 1;
	sKBIConfig.sPin[0].bEn = 1;

	KBI_Init(KBI1, &sKBIConfig);
	NVIC_EnableIRQ(KBI1_IRQn);
	NVIC_SetPriority(KBI1_IRQn, 3);
	KBI_SetCallback(KBI1, &KBI1Task);
	*/

	/* polling way */
	GPIO_PinInit(GPIO_PTD0, GPIO_PinInput_InternalPullup);
	PORT->IOFLT = 0xe4960000; // TODO: io filter on PTD0.

	/* LEDs init */
	GPIO_PinInit(LED1, GPIO_PinOutput);
	GPIO_PinInit(LED2, GPIO_PinOutput);
	GPIO_PinInit(LED3, GPIO_PinOutput);
	GPIO_PinInit(LED4, GPIO_PinOutput);
}

/******************************************************************************
 *  initial motor MOSFET  six phase PWM driver
 *  Using PWMTIMER timer
 *	 3-phase center-aligned PWM
 *	 PWM frequency 16 khz.
 *	 50 % of duty cycle
 *	 complementary mode
 *   Enable PWM output by ETM2OUTMASK = 0
 *	Disable PWM output by ETM2OUTMASK = 0x3F;
 *******************************************************************************/
void PWMTimerInit(void)
{
	uint8_t i;

	/* Enable ETM2 module */
	SIM->SCGC |= SIM_SCGC_ETM2_MASK;
	SIM->SOPT &= ~SIM_SOPT_NMIE_MASK; // Enable ETM2_CH4;

	PWMTIMER->MODE |= ETM_MODE_WPDIS_MASK | ETM_MODE_ETMEN_MASK;

	/* Setting for center aligned PWM in Combine Mode*/
	PWMTIMER->CNTIN = (unsigned)-PWM_PERIOD / 2;
	PWMTIMER->MOD = PWM_PERIOD / 2 - 1; // 16 kHz
	// Enable generation of trigger when ETM counter is equal to CNTIN value

	// ELSnB:ELSnA = 1:0 set channel to generate positive PWM
	for (i = 0; i < 6; i++)
		PWMTIMER->CONTROLS[i].CnSC = ETM_CnSC_MSB_MASK | ETM_CnSC_ELSB_MASK;
	/* in combine mode, PWM period = (MOC-CNTIN+1), PWM width=C(n+1)v - C(n)V;*/
	/* set 50% of duty cycle*/
	for (i = 0; i < 6; i += 2)
		PWMTIMER->CONTROLS[i].CnV = (unsigned)-PWM_PERIOD / 4;
	for (i = 1; i < 6; i += 2)
		PWMTIMER->CONTROLS[i].CnV = PWM_PERIOD / 4;

	/*	setting	softwareupdate	*/
	PWMTIMER->SYNC = ETM_SYNC_CNTMAX_MASK | ETM_SYNC_SYNCHOM_MASK; // output mask updated on PWM synchronization (not on rising edge of clock)

	PWMTIMER->COMBINE = ETM_COMBINE_SYNCEN2_MASK | ETM_COMBINE_DTEN2_MASK | ETM_COMBINE_COMP2_MASK | ETM_COMBINE_COMBINE2_MASK |
						ETM_COMBINE_SYNCEN1_MASK | ETM_COMBINE_DTEN1_MASK | ETM_COMBINE_COMP1_MASK | ETM_COMBINE_COMBINE1_MASK |
						ETM_COMBINE_SYNCEN0_MASK | ETM_COMBINE_DTEN0_MASK | ETM_COMBINE_COMP0_MASK | ETM_COMBINE_COMBINE0_MASK;

	PWMTIMER->DEADTIME = ETM_PWM_DEAD_TIME;

	// selecting the FTM2 init trigger for starting of the AD conversion, NOTE! trigger when PWM ON!
	PWMTIMER->EXTTRIG = ETM_EXTTRIG_INITTRIGEN_MASK;
	SIM->SOPT |= SIM_BUSDIV_BUSDIV_MASK | SIM_SOPT_BUSREF(0);
	SIM->SOPT |= SIM_SOPT_ADHWT(10); // TODO: 10:2? ADC trig by ETM2 , (2)->ETM2 init trigger;
	SIM->SOPT |= SIM_SOPT_DELAY(32); // delay 32 bus count after reload. for demag delay;

	/* Following line configures:
	   - enhanced PWM synchronization, FTM counter reset on SW sync
	   - output SW control / polarity registers updated on PWM synchronization (not on rising edge of clock)
	   - output SW control/inverting(swap)/mask registers updated from buffers on SW synchronization */
	PWMTIMER->SYNCONF = ETM_SYNCONF_SYNCMODE_MASK | ETM_SYNCONF_SWRSTCNT_MASK |
						ETM_SYNCONF_SWOC_MASK | ETM_SYNCONF_INVC_MASK |
						ETM_SYNCONF_SWSOC_MASK | ETM_SYNCONF_SWINVC_MASK | ETM_SYNCONF_SWOM_MASK;

	PWMTIMER->OUTMASK = ETM_OUTMASK_CH0OM_MASK | ETM_OUTMASK_CH1OM_MASK | ETM_OUTMASK_CH2OM_MASK |
						ETM_OUTMASK_CH3OM_MASK | ETM_OUTMASK_CH4OM_MASK | ETM_OUTMASK_CH5OM_MASK;

	PWMTIMER->CNT = 0x00; // update of FTM settings

	PWMTIMER->SC = ETM_SC_CLKS(1); // no ISR, counting up, system clock, divide by 1
	PWMTIMER->SYNC |= ETM_SYNC_SWSYNC_MASK;
	// Enable the loading of MOD,CNTIN and CnV from buffers
	PWMTIMER->PWMLOAD |= ETM_PWMLOAD_LDOK_MASK;
}

/******************************************************************************
 *  initial motor six step commutation driver
 *  Using CMTTIMER timer
 *  period set by ADC zero cross value, change as per motor speed
 *******************************************************************************/
void CMTTimerInit(void)
{
	/* Enable ETM1 module */
	SIM->SCGC |= SIM_SCGC_ETM1_MASK;
	/*	CMTTIMER->SC = 0;
		//system clock, divide by 128, 6.4us @ 20 MHz clock,time overflower interupt enable
		// Enable Output Compare interrupt, output Compare, Software Output Compare only \
		(ELSnB:ELSnA = 0:0, output pin is not controlled by FTM)

		CMTTIMER->CONTROLS[0].CnSC = ETM_CnSC_MSA_MASK | ETM_CnSC_CHIE_MASK;
		CMTTIMER->MOD = 0x4ff;
		CMTTIMER->CONTROLS[0].CnV = CMTTIMER->MOD >>2;
		CMTTIMER->CNT = 0;

		CMTTIMER->SC |= ETM_SC_CLKS(1) | ETM_SC_PS(7);
	*/
	ETM_PWMInit(CMTTIMER, ETM_PWMMODE_EDGEALLIGNED, ETM_PWM_HIGHTRUEPULSE);
	ETM_SetModValue(CMTTIMER, 0xfff);
	ETM_ClockSet(CMTTIMER, ETM_CLOCK_SYSTEMCLOCK, ETM_CLOCK_PS_DIV128);
	ETM_EnableOverflowInt(CMTTIMER);
	/* Set priority to interrupt
	 total 0,1,2,3, and 0x00 is highest priority*/
	NVIC_SetPriority(ETM1_IRQn, 0);
	NVIC_EnableIRQ(ETM1_IRQn);
	/* ISR for commutation sectors */
	ETM_SetCallback(CMTTIMER, CMTTimerTask);
}

/******************************************************************************
 *  initial PIT module for triger ADC
 *  PLEASE NOTE: Navate MCU could not using 2*ETM and PIT same time, not support
 *  set Triger at central point of phase width  ( SystemCoreClock / MT_PWM_FREQ)/ 4
 ******************************************************************************/
/*
void PITInit(void)
{

	SIM->SCGC |= SIM_SCGC_PIT_MASK;

	PIT->MCR = 0x00;

	// for  1ms timing, systecoreclock is 40Mhz,
	PIT->CHANNEL[0].LDVAL = SystemCoreClock / 1000 - 1;
	PIT->CHANNEL[0].TCTRL = 0x03;

	NVIC_EnableIRQ(PIT_CH0_IRQn);
	PIT_ChannelEnableInt(PIT_CHANNEL0);

	NVIC_SetPriority(PIT_CH0_IRQn, 3);

	PIT_SetCallback(PIT_CHANNEL0, PITTask);
}
*/
/******************************************************************************
 *   a 1 ms slow loop for speed KPI
 *******************************************************************************/
void RTCInit(void)
{

	SIM->SCGC |= SIM_SCGC_RTC_MASK;

	//	RTC->SC = 0;
	RTC->MOD = 15;
	//  bus clock 8MHz, divied by 1024,128us per tick
	RTC->SC |= RTC_SC_RTCLKS(RTC_CLKSRC_BUS) | RTC_SC_RTCPS(RTC_CLK_PRESCALER_1024);
	RTC->SC |= RTC_SC_RTIE_MASK;

	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 2);
	RTC_SetCallback(RTCTask);
}

/******************************************************************************
*  initial ADC module for BEMF sensor

*  ADC0_SE0 = BEMF_U, ADC0_SE1 = BEMF_V, ADC0_SE2 = BEMF_W
*  ADC0_SE3 = DC Bus I sensor
* Set ADC clock run in 10Mhz, 12bit, hardware trig and interrupt enable
*******************************************************************************/
void ADC0Init(void)
{

	/* enable ADC clock */
	SIM->SCGC |= SIM_SCGC_ADC_MASK;

	/* 12bit,input clock 10M */
	ADC->SC3 = ADC_SC3_MODE(2) | ADC_SC3_ADIV(2);

	/* input pin */

	ADC->APCTL1 = ADC_CHANNEL_AD0 | ADC_CHANNEL_AD1 | ADC_CHANNEL_AD2 | ADC_CHANNEL_AD3;
	/* Hardware trig */
	ADC->SC2 |= ADC_SC2_ADTRG_MASK;
	/* set channel */
	ADC_SetChannel(ADC, BEMF_W);
	ADC_VrefSelect(ADC, ADC_VREF_VDDA);

	NVIC_SetPriority(ADC0_IRQn, 1);
	ADC_IntEnable(ADC);
	NVIC_EnableIRQ(ADC0_IRQn);
	ADC_SetCallBack(ADCTask);
}

/*************************************************************************
 * Function Name: SetDutyCycle
 * Parameters: duty cycle value  is number of percent, 40 -> 40%;
* duty cycle:
	period = (Mod - CNTIN + 1);
	duty (On time ) = (CnV（N+1） - CnV(n) );
	when: ETM config as: ETMEN = 1, DECAPEN = 0, COMBINE = 1, CPWMS = 0;
 * Return: none
 * Description: Set PWM dutycycle of PWMTIMER change phase voltage
 *************************************************************************/
void SetPWMDutyCycle(uint8_t duty)
{
	uint16_t i, setduty;
	int16 FirstEdge, SecondEdge;

	if (duty > 90)
		duty = 90;
	//	duty = (induty-50) / 100 * (PWM_PERIOD / 2) ;
	setduty = (duty - 50) * 10;
	FirstEdge = -PWM_PERIOD / 4 - setduty;
	SecondEdge = PWM_PERIOD / 4 + setduty;

	for (i = 0; i < 6; i += 2)
		PWMTIMER->CONTROLS[i].CnV = FirstEdge;
	for (i = 1; i < 6; i += 2)
		PWMTIMER->CONTROLS[i].CnV = SecondEdge;
	// PWMLOAD LDOK update MOD,CNTIN,CnV;
	PWMTIMER->PWMLOAD |= ETM_PWMLOAD_LDOK_MASK;
}

/*************************************************************************
 * Function Name: SetPwmOutput
 * Parameters: sector number
 * Return: none
 * Description: set PWM output configuration based on selected sector
 *************************************************************************/
void SetPWMPhase(uint8_t sector)
{
	PWMTIMER->OUTMASK = CMTTable1P1N[sector].mask;
	PWMTIMER->INVCTRL = CMTTable1P1N[sector].swap;
	// PWM SYNV software sync output and invent;
	PWMTIMER->SYNC |= ETM_SYNC_SWSYNC_MASK;
}

/*************************************************************************
 * Function Name: Sector from  0 to 5 in running state;
 * Parameters: sector number
 *************************************************************************/
uint8_t NextSector(uint8_t sector)
{
	sector++;
	sector %= 6;
	motor_ctrl.phase_sector = sector;

	return (sector);
}

/*************************************************************************
 * Function Name: Set CMTTIMER period;
 * Parameters: period
 *************************************************************************/
void SetCMTPeriod(uint16_t period)
{
	CMTTIMER->MOD = period;
	ETM_SetModValue(CMTTIMER, period);
}
