//
// Created by hu on 2020/7/11.
//
#include "ArduinoWrapper.h"

float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max) {
    return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);

    return ch;
}

