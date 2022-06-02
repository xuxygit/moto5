/**********************************************************************
 *
 Using the UART for debug
 the hardware link UART1 as comm port
 UART1 is initied in HWConfig.c file
 define list of commands as bellow:

 ************* COMMANDS ***************
 <STARTM> -- Start Motor\n\r";
 <STOPMT> -- Stop Motor\n\r";
 <SETSPD> -- Set the Motor Speed\n\r";
 <GETSPD> -- Get the Motor Speed\n\r";
 <STATUS> -- Get the Status of system \n\r";

 ************* MAIN MOTOR PARAMETERS ***************
 <ACCELL> -- Motor acceleration value during startup \n\r";

 ****** PI REGULATOR PARAMETERS - SPEED LOOP  ******\n\r
 <KP-PRM> -- Set the PI proportional term \n\r";
 <KI-PRM> -- Set the PI integral term \n\r\n\r >";

 *
 ************************************************************************/

//#include "uiuartcom.h"
#include "hwconfig.h"
#include "motoctrl.h"
#include "uartcom.h"
#include "statemachine.h"

#ifdef UARTCOM

extern Motor_Ctrl_t motor_ctrl;
extern SysStatus_t status;
uint16_t uart_cmd_set_value_flag = 0;

const CMD_T CmdTable[] = {
    {"START", CMD_STARTM},
    {"STOP", CMD_STOPMT},
    {"SETSPD", CMD_SETSPD},
    {"STATUS", CMD_STATUS},
    {"GETSPD", CMD_GETSPD},
    {"ALIGN", CMD_ALIGN},
    {"ACCEL", CMD_ACCEL},
    {"OPLOOP", CMD_OPLOOP},
    {"KP_PRM", CMD_KP_PRM},
    {"KI_PRM", CMD_KI_PRM},
    {"HELP", CMD_HELP},
    {"", NULL}};

/** @defgroup MC_UI_INIT    MC_UI_INIT
 *  @{
 * @brief UART User Interface init
 */
void MC_UI_INIT()
{
  UART_SendWait(UART1, row0TxBuf, (COUNTOF(row0TxBuf) - 1));
  UART_SendWait(UART1, comTxBuf, (COUNTOF(comTxBuf) - 1));
  UART_SendWait(UART1, startMtTxBuf, (COUNTOF(startMtTxBuf) - 1));
  UART_SendWait(UART1, stopMtTxBuf, (COUNTOF(stopMtTxBuf) - 1));
  UART_SendWait(UART1, setMtDirTxBuf, (COUNTOF(setMtDirTxBuf) - 1));
  UART_SendWait(UART1, setMtSpdTxBuf, (COUNTOF(setMtSpdTxBuf) - 1));
  UART_SendWait(UART1, getMtSpdTxBuf, (COUNTOF(getMtSpdTxBuf) - 1));
  UART_SendWait(UART1, getMtStuTxBuf, (COUNTOF(getMtStuTxBuf) - 1));
  UART_SendWait(UART1, setMtPtmTxBuf, (COUNTOF(setMtPtmTxBuf) - 1));
  UART_SendWait(UART1, msgHelpTxBuf, (COUNTOF(msgHelpTxBuf) - 1));
  UART_SendWait(UART1, setMtIRefTxBuf, (COUNTOF(setMtIRefTxBuf) - 1));
  UART_SendWait(UART1, setMtAccTxBuf, (COUNTOF(setMtAccTxBuf) - 1));
  UART_SendWait(UART1, setMtPolTxBuf, (COUNTOF(setMtPolTxBuf) - 1));
  UART_SendWait(UART1, msgPILoopTxBuf, (COUNTOF(msgPILoopTxBuf) - 1));
  UART_SendWait(UART1, setMtKPTxBuf, (COUNTOF(setMtKPTxBuf) - 1));
  UART_SendWait(UART1, setMtKITxBuf, (COUNTOF(setMtKITxBuf) - 1));

  UART1RxTask(UART1);
  //   motor_ctrl.Button_ready = TRUE;
}

/** @defgroup UART_num_decode    UART_num_decode
 *  @{
 * @brief UART Value decoding function
 */
uint32_t UART_num_decode()
{
  uint8_t i;

  static char Value_Buf[RXBUFFERSIZE];

  for (i = 0; i < BUFF_RCV; i++)
  {
    Value_Buf[i] = aRxBuf[i];
  }

  return (atoi(Value_Buf));
}

/** @defgroup UART_Set_Value    UART_Set_Value
 *  @{
 * @brief UART Main function
 */
void UART_Set_Value()
{

  if (uart_cmd_set_value_flag == 1)
  {

    motor_ctrl.uart_set_value = UART_num_decode();

    switch (motor_ctrl.uart_set_cmd)
    {
    case SETSPD_CMD: /*!<  Set the new speed value command received */
      motor_ctrl.speed_mech = motor_ctrl.uart_set_value;
      break;
    case ACCEL_CMD: /*!<  Set the Accelleration for Start-up of the motor command received */
      motor_ctrl.accel = motor_ctrl.uart_set_value;
      break;
    case KP_PRM_CMD: /*!<  Set the KP PI param command received */
      motor_ctrl.KP = motor_ctrl.uart_set_value;
      break;
    case KI_PRM_CMD: /*!<  Set the KI PI param command received */
      motor_ctrl.KI = motor_ctrl.uart_set_value;
      break;
    }

    BUFF_RCV = RXBUFFERSIZE;
    uart_cmd_set_value_flag = 0;
    UART_SendWait(UART1, msgOKTxBuf, (COUNTOF(msgOKTxBuf) - 1));
    UART_FLAG_RECEIVE = TRUE;
  }
}

/** @defgroup CMD_MSG    CMD_MSG
 *  @{
 * @brief UART Transmit standard message
 */
void CMD_MSG()
{
  UART_SendWait(UART1, msgInsTxBuf, (COUNTOF(msgInsTxBuf) - 1));
  BUFF_RCV = RXBUFFERSIZE - 1;
  uart_cmd_set_value_flag = 1;
  UART_FLAG_RECEIVE = TRUE;
}

void CMD_ALIGN()
{
  MotorAlign();
}

/** @defgroup CMD_STARTM    CMD_STARTM
 *  @{
 * @brief UART Transmit Start motor message
 */
void CMD_STARTM()
{
  /*!<  Start Motor command received */

  UART_SendWait(UART1, msgStartMtAckTxBuf, (COUNTOF(msgStartMtAckTxBuf) - 1));
  MotorStart();
  UART_FLAG_RECEIVE = TRUE;
}

/** @defgroup CMD_OPLOOP
 *  @{
 * @brief UART Transmit open loop run message
 */
void CMD_OPLOOP()
{
  UART_SendWait(UART1, oploopMtTxBuf, (COUNTOF(oploopMtTxBuf) - 1));
  MotorOpenLoop();
  UART_FLAG_RECEIVE = TRUE;
}
/** @defgroup CMD_STOPMT    CMD_STOPMT
 *  @{
 * @brief UART Transmit Stop motor message
 */
void CMD_STOPMT()
{
  /*!<  Stop Motor command received */
  UART_SendWait(UART1, msgStopMtAckTxBuf, (COUNTOF(msgStopMtAckTxBuf) - 1));
  MotorStop();
  UART_FLAG_RECEIVE = TRUE;
}

/** @defgroup CMD_SETSPD    CMD_SETSPD
 *  @{
 * @brief UART Change the motor speed
 */
void CMD_SETSPD()
{
  /*!<  Set the new speed value command received */
  CMD_MSG();
  motor_ctrl.uart_set_cmd = SETSPD_CMD;
  uart_cmd_set_value_flag = 1;
}

/** @defgroup CMD_GETSPD    CMD_GETSPD
 *  @{
 * @brief UART Get the motor speed
 */
void CMD_GETSPD()
{
  /*!<  Get Mechanical Motor Speed command received */
  static char strLineMessage[40];

  sprintf(strLineMessage, "-- The Motor Speed is: %d RPM -- \r\n >", motor_ctrl.speed_fb);
  UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
  UART_FLAG_RECEIVE = TRUE;
}

/** @defgroup CMD_HELP    CMD_HELP
 *  @{
 * @brief UART Help command
 */
void CMD_HELP()
{
  /*!<  Help command received */
  MC_UI_INIT();
  UART_FLAG_RECEIVE = TRUE;
}

/** @defgroup CMD_ACCEL    CMD_ACCEL
 *  @{
 * @brief UART Set the accelleration of the motor at start-up
 */
void CMD_ACCEL()
{
  /*!<  Set the Accelleration for Start-up of the motor command received */
  CMD_MSG();
  motor_ctrl.uart_set_cmd = ACCEL_CMD;
  uart_cmd_set_value_flag = 1;
}

/** @defgroup CMD_KP_PRM    CMD_KP_PRM
 *  @{
 * @brief UART Set the  KP PI param
 */
void CMD_KP_PRM()
{
  /*!<  Set the KP PI param command received */
  CMD_MSG();
  motor_ctrl.uart_set_cmd = KP_PRM_CMD;
  uart_cmd_set_value_flag = 1;
}

/** @defgroup CMD_KI_PRM    CMD_KI_PRM
 *  @{
 * @brief UART Set the KI PI param
 */
void CMD_KI_PRM()
{
  /*!<  Set the KI PI param command received */
  CMD_MSG();
  motor_ctrl.uart_set_cmd = KI_PRM_CMD;
  uart_cmd_set_value_flag = 1;
}

/** @defgroup CMD_STATUS    CMD_STATUS
 *  @{
 * @brief UART View the STATUS
 */

void CMD_STATUS()
{
  static char strLineMessage[30];

  switch (status)
  {
  case START:
    sprintf(strLineMessage, " -- The status is: STARTUP --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  case STOP:
    sprintf(strLineMessage, " -- The status is: STOP --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  case RUN:
    sprintf(strLineMessage, " -- The status is: RUN --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    sprintf(strLineMessage, "-- The CMT=: %d -- \r\n >", CMTTIMER->MOD);
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    sprintf(strLineMessage, "-- The BEMF=: %d -- \r\n >", motor_ctrl.Bemf_sum / 5);
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  case ALIGNMENT:
    sprintf(strLineMessage, " -- The status is: ALIGNMENT --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  case OPENLOOP:
    sprintf(strLineMessage, " -- The status is: OPENLOOP --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  case FAULT:
    sprintf(strLineMessage, " -- The status is: FAULT --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  default:
    sprintf(strLineMessage, " -- The status is: IDLE --\r\n >");
    UART_SendWait(UART1, (uint8_t *)strLineMessage, sizeof(strLineMessage));
    break;
  }
  UART_FLAG_RECEIVE = TRUE;
}

/** @defgroup CMD_Parser    CMD_Parser
 *  @{
 * @brief UART String parser
 */
void CMD_Parser(char *pCommandString)
{
  static uint8_t CmdListIndex;
  char sCmd[8];

  strcpy(sCmd, pCommandString);
  strtok(sCmd, TOKEN);

  /* Command Callback identification */
  for (CmdListIndex = 0; CmdTable[CmdListIndex].pCmdFunc != NULL; CmdListIndex++)
  {
    if (strcmp(CmdTable[CmdListIndex].name, sCmd) == 0)
    {
      break;
    }
  }
  if (CmdListIndex < CMD_NUM)
  {
    // CMD OK --> extract parameters
    /* Check for valid callback */
    if (CmdTable[CmdListIndex].pCmdFunc != NULL)
    {
      CmdTable[CmdListIndex].pCmdFunc();
    }
  }
  else
  {
    UART_SendWait(UART1, msgErrTxBuf, (COUNTOF(msgErrTxBuf) - 1));
    BUFF_RCV = RXBUFFERSIZE;
    aRxBufindex = 0;
    uart_cmd_set_value_flag = 0;
    UART_FLAG_RECEIVE = TRUE;
    return;
  }
}

/**********************************************************************
*  UART1 Rx_ISR
*	get char from uart1 and put data to aRxBuf
*	check if char is '/r/ means a input command or digital is done.
*	call cmd_prese

**********************************************************************/

void UART1RxTask(UART_Type *pUART)
{
  volatile uint8_t rcvd;

  if (UART_CheckFlag(pUART, UART_FlagOR))
  {
    rcvd = UART_ReadDataReg(UART1);
  }

  else if (UART_IsRxBuffFull(UART1))
  {
    rcvd = UART_ReadDataReg(UART1);

    if (aRxBufindex == RXBUFFERSIZE)
      aRxBufindex = 0;
    else
      aRxBuf[aRxBufindex++] = rcvd;

    // check if rcvd is '\r'	, \r\ = 0x0D in ascii table
    if (rcvd == 0x0d)
    {
      aRxBufindex = 0;
      if (uart_cmd_set_value_flag == 1)
      {
        UART_Set_Value();
        return;
      }

      CMD_Parser((char *)aRxBuf);
    }
  }
}

#endif
