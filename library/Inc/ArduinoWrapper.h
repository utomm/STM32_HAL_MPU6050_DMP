/*
 * ArduinoWrapper.h
 *
 *  Created on: Mar 27, 2013
 *      Author: zoellner
 */

#ifndef ARDUINOWRAPPER_H_
#define ARDUINOWRAPPER_H_



//Standard Libraries
#include <stm32f1xx_hal.h>
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

//TODO functions that need wrapper: millis(), Serial.print

#define PI 3.1416f
#define millis() HAL_GetTick()
#define I2CDEV_DEFAULT_WRITE_TIMEOUT     100
#define delay(x) HAL_Delay(x)



float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE;

#ifdef __cplusplus
}
#endif




#endif /* ARDUINOWRAPPER_H_ */
