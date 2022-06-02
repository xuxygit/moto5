#ifndef _UARTCOM_H_
#define _UARTCOM_H_
#include <common.h>

/*
 *	UI UARTCOM commands list
 */

#define SETSPD_CMD 2 /*!<  Set the new speed value command received */
#define GETSPD_CMD 3 /*!<  Get Mechanical Motor Speed command received */
#define ACCEL_CMD 4  /*!<  Set the Accelleration for Start-up of the motor command received */
#define KP_PRM_CMD 5 /*!<  Set the KP PI param command received */
#define KI_PRM_CMD 6 /*!<  Set the KI PI param command received */
#define STATUS_CMD 8 /*!<  Get the Status of the system command received */

uint8_t BUFF_RCV = 8;
uint8_t aRxBuf[8] = {'0'}; /*!< Buf used for reception */
uint8_t aRxBufindex = 0;
uint8_t row0TxBuf[] = "\033[2J"; /*!< Buf used for transmission */
uint8_t comTxBuf[] = " List of commands:\n\r\n\r";
uint8_t startMtTxBuf[] = "  <STARTM> -- Start Motor\n\r";
uint8_t stopMtTxBuf[] = "  <STOPMT> -- Stop Motor\n\r";
uint8_t oploopMtTxBuf[] = "  <OPLOOP> -- openloop run Motor\n\r";
uint8_t setMtDirTxBuf[] = "  <DIRECT> -- Set the Motor direction CW or CCW \n\r";
uint8_t setMtSpdTxBuf[] = "  <SETSPD> -- Set the Motor Speed\n\r";
uint8_t getMtSpdTxBuf[] = "  <GETSPD> -- Get the Motor Speed\n\r";
uint8_t getMtStuTxBuf[] = "  <STATUS> -- Get the Status of system \n\r";
uint8_t setMtPtmTxBuf[] = "  <POTENZ> -- Enable/Disable the potentiometer\n\r";
uint8_t msgHelpTxBuf[] = "  <HELP>   -- Show the help menu \n\r";
uint8_t msgInsTxBuf[] = " >>> Insert the value: ";
uint8_t msgEnTxBuf[] = " >>> ENABLE <1> DISABLE <0>: ";
uint8_t msgCWTxBuf[] = " >>> CW <0> CCW <1>: ";
uint8_t msgStartMtAckTxBuf[] = " >>> START MOTOR COMMAND RECEIVED ! <<< \n\r >";
uint8_t msgStopMtAckTxBuf[] = " >>> STOP MOTOR COMMAND RECEIVED ! <<< \n\r >";
uint8_t msgErrTxBuf[] = " >>> ERROR - PLEASE TYPE AGAIN ! <<< \n\r >";
uint8_t setMtIRefTxBuf[] = "  <INIREF> -- Start-up current reference (0-4095) \n\r";
uint8_t setMtAccTxBuf[] = "  <ACCELE> -- Motor acceleration value during startup \n\r";
uint8_t setMtPolTxBuf[] = "  <POLESP> -- Set the Motor pole pairs \n\r";
uint8_t msgPILoopTxBuf[] = "          ****** PI REGULATOR PARAMETERS - SPEED LOOP  ******\n\r";
uint8_t setMtKPTxBuf[] = "  <KP-PRM> -- Set the PI proportional term \n\r";
uint8_t setMtKITxBuf[] = "  <KI-PRM> -- Set the PI integral term \n\r\n\r >";
uint8_t msgOKTxBuf[] = " --- OK ---\n\r >";
uint8_t UART_FLAG_RECEIVE;

/******************************************************************************
 *  UART define and functions
 *******************************************************************************/

#define TOKEN "\r"
#define CMD_NUM 13
typedef struct
{
    char name[10];
    void (*pCmdFunc)(void);
} CMD_T;

#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE (COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE 8

void CMD_STARTM(void);
void CMD_STOPMT(void);
void CMD_SETSPD(void);
void CMD_GETSPD(void);
void CMD_OPLOOP(void);
void CMD_STATUS(void);
void CMD_ALIGN(void);
void CMD_ACCEL(void);
void CMD_HELP(void);
void CMD_KP_PRM(void);
void CMD_KI_PRM(void);
void CMD_Parser(char *pCommandString);

#endif
