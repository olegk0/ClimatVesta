#include "usart.h"
#include "subs.h"
#include "io.h"

#define USART_BAUDRATE 38400
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 

void USART_Init(void)
{
    UBRR0L = (uint8_t) (BAUD_PRESCALE & 0xff);
    UBRR0H = (uint8_t) (BAUD_PRESCALE >> 8);
    //UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0B = (1 << TXEN0);

    set_dir_out(OUT_UART_TX);
}

//______________________________________________________________________________
//помещает символ в буфер, инициирует начало передачи
void USART_PutChar(char sym)
{
    while (!( UCSR0A & (1 << UDRE0)))
	delay(1);
    UDR0 = sym;
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

