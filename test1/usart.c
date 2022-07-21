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
#include "usart.h"
#include "subs.h"
#include "io.h"

//#define USART_BAUDRATE 38400L
#define USART_BAUDRATE 19200L
#define UBRRL_value (F_CPU/(USART_BAUDRATE*16))-1

//инициализация usart`a
void USART_Init(void)
{
    UBRRL = UBRRL_value;       //Младшие 8 бит UBRRL_value
    UBRRH = (UBRRL_value) >> 8;  //Старшие 8 бит UBRRL_value
    UCSRB |=(1<<TXEN);         //Бит разрешения передачи
    UCSRC |=(1<< URSEL)|(1<< UCSZ0)|(1<< UCSZ1); //Устанавливем формат 8 бит данных
//	USART_RxTx485(1);
    set_dir_out(OUT_UART_TX);
}

//помещает символ в буфер, инициирует начало передачи
void USART_PutChar(char sym)
{
    while(!( UCSRA & (1 << UDRE)))
	delay(1);
//    while ( !( UCSRA & (1<<5)) ) {}
    UDR = sym;
}

//функция посылающая строку по usart`у
void USART_SendStr(char * data)
{
    uint8_t sym;
    while (*data) {
	sym = *data++;
	USART_PutChar(sym);
    }
}

