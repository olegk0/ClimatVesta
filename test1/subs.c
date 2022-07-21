#include "subs.h"
#include <avr/pgmspace.h>
#include "io.h"

void delay_ms0(uint8_t ms)
{
    while (ms > 0) {
	delay(1000);
	ms--;
    }
//	wdt_reset();
}

void delay_ms(uint16_t ms)
{
    while (ms > 0) {
	delay(1000);
	ms--;
    }
//	wdt_reset();
}

void delay_s(uint8_t s)
{
    while (s > 0) {
	delay_ms(1000);
	s--;
    }
//	wdt_reset();
}

int16_t check_range(int16_t min, int16_t in, int16_t max)
{
    if (in > max)
	in = max;
    if (in < min)
	in = min;
    return in;
}

float check_rangef(float min, float in, float max)
{
    if (in > max)
	in = max;
    if (in < min)
	in = min;
    return in;
}

int8_t delay_btn_check(int8_t *delay_var)
{
    if (*delay_var) {
	if (*delay_var > BTN_LONG_DELAY) {
	    *delay_var = 0;
	    return 1;
	}
    } else {
	*delay_var = 1;
    }
    return 0;
}

void num_to_str(struct m2bytes *nbuf)
{
    uint8_t tmp, num;
//m2bytes nbuf;

    num = nbuf->one;
    if (num > 99)
	num = 99;
    tmp = num / 10;
    nbuf->one = tmp + '0';
    num -= tmp * 10;
    nbuf->two = num + '0';
}
/*
 void num_to_lcd(uint8_t fs,uint8_t num)
 {
 struct m2bytes nbuf;

 nbuf.one = num;
 num_to_str(&nbuf);
 LcdChr(fs,nbuf.one,0);
 LcdChr(fs,nbuf.two,0);
 }
 */

void SPI_SendByte(int8_t byte)
{
    SPDR = byte;
    while (!(SPSR & (1 << SPIF)))
	;
}

void SPI_init(void)
{
//    PORTB &= ~((1 << PORTB3) | (1 << PORTB5)); //низкий уровень
//    PORTB |= (1 << PORTB2);
    pin_high(SPI_SS);
    pin_low(SPI_MOSI);
    pin_low(SPI_SCK);

    //    DDRB = DDRB & ~((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4) | (1 << PORTB5));
    //    DDRB |= ((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB5)); //ножки SPI на выход
    set_dir_in(SPI_MISO);
    set_dir_out(SPI_MOSI);
    set_dir_out(SPI_SS);
    set_dir_out(SPI_SCK);

    //SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0); //включим шину, объ¤вим ведущим, делитель 16
    /*разрешение spi,старший бит вперед,мастер, режим 0*/
    SPCR = (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0);
    SPSR = 0;    //(0 << SPI2X);
}

void SPI_deInit()
{
    SPCR = 0;
    set_dir_in(SPI_MISO);
    set_dir_in(SPI_MOSI);
    set_dir_in(SPI_SS);
    set_dir_in(SPI_SCK);
}

void ADC_Init()
{
    ADCSRA = (1 << ADEN) // Включаем АЦП
    | (1 << ADPS1) | (1 << ADPS0);    // устанавливаем предделитель преобразователя на 8
    ADMUX = (1 << REFS1) | (1 << REFS0); //Подключен внутренний ион 1.1В, с внешним конденсатором на AREF пине
}

void ADC_Mux(adc_in_type input)
{
    ADMUX &= ~(0b1111);
    ADMUX |= (input & 0b1111);
    delay_ms(1); // TODO
}

void ADC_Start()
{
    ADCSRA |= (1 << ADSC);    // Начинаем преобразование
}

uint16_t ADC_GetData()
{
    return (ADCL | ADCH << 8); // Считываем  полученное значение
}

