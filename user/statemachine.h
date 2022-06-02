/******************************************************************************
 *
 * @brief provide header files of motor state machine.
 *
 *******************************************************************************/
#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include <common.h>

void StateMachine(void);
void MotorInit(void);
void MotorReset(void);
void MotorStop(void);
void MotorStart(void);
void MotorAlign(void);
void MotorOpenLoop(void);
void MotorRun(void);
void MotorFault(void);
uint8_t MotorZCDetecting(void);
void MotorSpeedKPI(void);

#endif
