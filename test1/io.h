/*
 * io.h
 *
 *  Created on: 14 окт. 2019 г.
 *      Author: olegvedi@gmail.com
 */

#ifndef IO_H_
#define IO_H_

#include <avr/io.h>


#define SWITCH_BIT(var,bit)     {(var) = (var) ^ (1<<bit);}
#define SET_BIT(var,bit)        {(var) = (var) | (1<<bit);}
#define CLR_BIT(var,bit)        {(var) = (var) & ~(1<<bit);}
#define TST_BIT(var,bit)        ((var) & (1<<bit))

#define _CAT(a, b)                      a ## b
#define CAT(a, b)                       _CAT(a, b)

#define TO_PORT(prt,pn) CAT(PORT,prt)
#define TO_PIN(prt,pn) CAT(PIN,pn)
#define TO_DDR(prt,pn) CAT(DDR,prt)

#define TO_PORT_NUM(prt,pn) CAT(CAT(PORT,prt),pn)
#define TO_PIN_NUM(prt,pn) CAT(PIN,pn)
#define TO_DDR_NUM(prt,pn) CAT(DDR,prt)







#define BIT(p,b)                (b)

#define PORT(p,b)               (PORT ## p)
#define PIN(p,b)                (PIN ## p)
#define DDR(p,b)                (DDR ## p)


#define Set_Port_Bit(p,b)       ((p) |= _BV(b))
#define Clr_Port_Bit(p,b)       ((p) &= ~_BV(b))
#define Tgl_Port_Bit(p,b)       ((p) ^= _BV(b))

#define Get_Port_Bit(p,b)       (((p) & _BV(b)) != 0)


//user functions:
#define bit(io)	                BIT(io)
#define port(io)                PORT(io)

#define pin_high(io)            Set_Port_Bit(PORT(io),BIT(io))
#define pin_low(io)             Clr_Port_Bit(PORT(io),BIT(io))
#define pin_toggle(io)          Tgl_Port_Bit(PORT(io),BIT(io))

#define get_output(io)          Get_Port_Bit(PORT(io),BIT(io))
#define get_input(io)           Get_Port_Bit(PIN(io),BIT(io))

#define set_dir_in(io)          Clr_Port_Bit(DDR(io),BIT(io))
#define set_dir_out(io)         Set_Port_Bit(DDR(io),BIT(io))
#define dir_toggle(io)          Tgl_Port_Bit(DDR(io),BIT(io))





//*****************SPI Display***********************

#define SPI_MISO	B,4
#define SPI_MOSI	B,3
#define SPI_SS		B,2
#define SPI_SCK		B,5

//*****************Input***********************
#define IN_BTN_HI_MODE	C,4 //long press

#define IN_BTN_T_DWN	D,2 //int0
#define IN_BTN_T_UP	D,3 //int1

#define IN_BTN_V_DWN	B,0 //int
#define IN_BTN_V_UP	D,7 //int

#define IN_ACC		C,3 //1-on
#define IN_FROST_BTN	B,4 //0-on, long press - setup
#define IN_NIGHT_BR	C,2 //1-night

#define IN_ADC_T_FROST	ADC6
#define IN_ADC_T_SALON	ADC7

//*****************Output***********************
#define OUT_FROST_SW	C,5 //1-on
#define OUT_FROST_IND	C,1 //1-on

#define OUT_BTN_T_DWN	D,4
#define OUT_BTN_T_UP	D,5

#define OUT_BTN_V_DWN	B,1
#define OUT_BTN_V_UP	D,6

#define OUT_BTN_PREF_MODE       C,0 // любимый режим обдува (на кнопку режима)
#define OUT_BTN_Recycle	D,0 // Recycle Btn,  0 - pressed

#define OUT_UART_TX	D,1

//*****************Temp***********************


#define IN_BTN_1        C,2 //long press
#define IN_BTN_2        C,3 //

//*****************Output***********************
#define OUT_CH_1        C,4
#define OUT_CH_2        C,1

#define OUT_CH_LED_1    C,0
#define OUT_CH_LED_2    C,5

#define OUT_UART_TX     D,1


#endif /* IO_H_ */
