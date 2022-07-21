#include "subs.h"
#include <avr/pgmspace.h>
#include "disp.h"
#include "io.h"
#include <stdlib.h>

void Send_7219(int8_t rg, int8_t dt)
{
    pin_low(SPI_SS);
    delay(1);
    SPI_SendByte(rg);
    SPI_SendByte(dt);
    delay(1);
    pin_high(SPI_SS);
}

void Clear_7219(void)
{
    int8_t i = DISP_LEN;
    // Loop until 0, but don't run for zero
    do {
	// Set each display in use to blank
	Send_7219(i, 0xF); //int8_t BLANK
    } while (--i);
}

/*static uint8_t bar_conv_singl[9] = {
	0, 0b1000, 0b10000000, 0b100, 0b10000, 1, 0b100000, 0b10, 0b1000000 };
	*/
static uint8_t bar_conv_full[] = {
	0, 0b1000, 136, 140, 156, 157, 189, 191, 255 };
void Disp_Bars(uint8_t dt)
{
    if (dt < 9) {
	dt = bar_conv_full[dt];
	Send_7219(BAR1_SEG, 0);
    } else {
	if (dt > 10)
	    dt = 10;
	Send_7219(BAR1_SEG, bar_conv_full[dt - 8]);
	dt = 255;
    }
    Send_7219(BAR0_SEG, dt);
}
// |

static uint8_t num_conv_singl[10] = {
	0b10111101, 0b10000001, 0b11111000, 0b11101001, 0b11000101, 0b1101101, 0b1111101, 0b10001001, 0b11111101,
	0b11101101 };
void Disp_Num_Seg(uint8_t seg, uint8_t num, uint8_t dot)
{
    if (num < 10) {
	num = num_conv_singl[num];
    } else {
	switch (num) {
	case 'S':
	    num = num_conv_singl[5];
	    break;
	case 'A':
	case 'a':
	    num = 0b11011101;
	    break;
	case 'b':
	case 'B':
	    num = 0b01110101;
	    break;
	case 'C':
	    num = 0b00111100;
	    break;
	case 'c':
	    num = 0b01110000;
	    break;
	case 'D':
	case 'd':
	    num = 0b11110001;
	    break;
	case 'E':
	case 'e':
	    num = 0b01111100;
	    break;
	case 'F':
	case 'f':
	    num = 0b01011100;
	    break;
	case 'h':
	    num = 0b01010101;
	    break;
	case 'H':
	    num = 0b11010101;
	    break;
	case 'n':
	    num = 0b01010001;
	    break;
	case 'o':
	    num = 0b01110001;
	    break;
	case 't':
	    num = 0b01110100;
	    break;
	case 'L':
	    num = 0b00110100;
	    break;
	case 'i':
	    num = 0b00000001;
	    break;
	case 'I':
	    num = 0b00010100;
	    break;
	case 'r':
	    num = 0b01010000;
	    break;
	case 'P':
	    num = 0b11011100;
	    break;
	case '?':
	    num = 0b11011000;
	    break;
	default:
	    num = 0b01000000;
	}
    }
    if (dot)
	num |= 2;
    Send_7219(seg, num);
}

void Disp_Num(int8_t num, uint8_t dot)
{
    uint8_t dot1 = 0;
    uint8_t st, ml;
    if (num < 0) {
	dot1 = 1;
	num = abs(num);
    }
    if (num > 99) {
	st = 'h';
	ml = 'i';
    } else {
	st = num / 10;
	ml = num - (st * 10);
    }

    Disp_Num_Seg(NUM0_SEG, st, dot1);
    Disp_Num_Seg(NUM1_SEG, ml, dot);
}
