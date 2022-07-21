#ifndef USART_H
#define USART_H

#include <avr/io.h>

void USART_Init(void); //инициализация usart`a
void USART_PutChar(char sym); //положить символ в буфер
void USART_SendStr(char * data); //послать строку по usart`у

#endif //USART_H
