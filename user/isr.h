/******************************************************************************
*
* @brief Define interrupt service routines referenced by the vector table.
*
* Note: Only "vectors.c" should include this header file.
*
*******************************************************************************
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 

/******************************************************************************
* Macros
******************************************************************************/


#undef  VECTOR_036
#define VECTOR_036 RTC_Isr
extern void  RTC_Isr(void);

#undef  VECTOR_041
#define VECTOR_041 KBI1_Isr
extern void KBI1_Isr(void);

#undef  VECTOR_029
#define VECTOR_029	UART1_Isr         
extern void UART1_Isr(void);

#undef  VECTOR_033
#define VECTOR_033 ETM0_Isr
extern void ETM0_Isr(void);

#undef  VECTOR_034
#define VECTOR_034 ETM1_Isr
extern void ETM1_Isr(void);


#undef  VECTOR_035
#define VECTOR_035 ETM2_Isr
extern void ETM2_Isr(void);

/*
#undef  VECTOR_038
#define VECTOR_038 PIT_Ch0Isr
extern void PIT_Ch0Isr(void);
*/
#undef  VECTOR_031
#define VECTOR_031 ADC_Isr
extern void ADC_Isr(void);

#endif  //__ISR_H
/* End of "isr.h" */
