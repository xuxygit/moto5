/******************************************************************************
 *
 * @brief provide header files of hardware config.
 * UI com, LED,BTN
 * ETM2, ETM0, PIT
 * ADC
 *
 *******************************************************************************/

#ifndef _HWCONFIG_H_
#define _HWCONFIG_H_

#include "common.h"
#include "ics.h"
#include "sysinit.h"

#include "etm.h"
//#include "pit.h"
#include "adc.h"
#include "rtc.h"

#include "gpio.h"
#include "kbi.h"
#include "pmc.h"

#include "uart.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>

/**********************************************************************
 *   in the HW board
 *	KEY = PTD0
 * LED1 = PTB3, LED2 = PTB2, LED3 = PTB1, LED4 = PTB0
 *
 **********************************************************************/
#define LED1 GPIO_PTB3
#define LED2 GPIO_PTB2
#define LED3 GPIO_PTB1
#define LED4 GPIO_PTB0

void KbLedInit(void);
void KBI1Task(void);
void BtnPolling(void);
void PowerOn(void);
void PowerOff(void);

/**********************************************************************
 *  in the HW board
 *	ETM2_CH0 = PWM_UH, ETM2_CH1 = PWM_UL,
 *	ETM2_CH2 = PWM_VH, ETM2_CH3 = PWM_VL,
 *	ETM2_CH4= PWM_WH, ETM2_CH5 = PWM_WL,
 *
 **********************************************************************/
#define PWMTIMER ETM2
#define CMTTIMER ETM1
#define LOOPTIMER ETM0
#define ADCTRIGER PIT

#define ETM_PWM_DEAD_TIME (40)                /* dead time 1us = PWM_DEAD_TIME/20Mhz */
#define MT_PWM_FREQ (16000)                   /* PWM frequency - 16kHz */
#define PWM_PERIOD (BUS_CLK_HZ / MT_PWM_FREQ) /* 3000 on this MCU run at 20Mhz */
#define MT_CTRL_LOOP_FREQ (1000)              /* Control loop frequency */

#define BEMF_U ADC_CHANNEL_AD0
#define BEMF_V ADC_CHANNEL_AD1
#define BEMF_W ADC_CHANNEL_AD2
#define IDcbus ADC_CHANNEL_AD3
#define VREFH (5) // MCU Vdd=Vrefh=5.0v

void HWConfig(void);

void PWMTimerInit(void);

void CMTTimerInit(void);
void CMTTimerTask(void);

void LoopTimerInit(void);
void LoopTimerTask(void);

/* PIT for trig ADC */
/*
void PITInit(void);
void PITTask(void);
*/
/* ADC for 3*BEMF and current sensor */
void ADC0Init(void);
void ADCTask(void);

/* RTC for timer count */
void RTCInit(void);
void RTCTask(void);

void PowerOn(void);
void PowerOff(void);

void UART1RxTask(UART_Type *pUART);
void UART1Init(void);

typedef struct
{
  char mask;
  char swap;
  char adchannel;
} PWMCtrl_t;

void SetPWMDutyCycle(uint8_t induty);
void SetPWMPhase(uint8_t sector);
uint8_t NextSector(uint8_t sector);
void SetCMTPeriod(uint16_t period);

#endif
