//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: WINAVR
//
//  Description.: драйвер USART/UART с кольцевым буфером
//
//  Data........: 11.01.10 
//
//***************************************************************************
#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>

#define SIZE_TBUF 10       //и размер кольцевых буферов - <255
#define SIZE_RBUF 10

void USART_Init(void); //инициализация usart`a
uint8_t USART_GetTxCount(void); //взять число символов передающего буфера
void USART_FlushTxBuf(void); //очистить передающий буфер
void USART_PutChar(char sym); //положить символ в буфер
void USART_SendStr(char * data); //послать строку по usart`у
uint8_t USART_GetRxCount(void); //взять число символов в приемном буфере
void USART_FlushRxBuf(void); //очистить приемный буфер
uint8_t USART_GetChar(void); //прочитать приемный буфер usart`a

#endif //USART_H
