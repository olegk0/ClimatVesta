//#include <avr/wdt.h>
#define XTAL F_CPU
#include <stdint.h>
#include <avr/interrupt.h>

#define BTN_LONG_DELAY	2

typedef enum {
    ADC0 = 0,
    ADC1 = 1,
    ADC2 = 2,
    ADC3 = 3,
    ADC4 = 4,
    ADC5 = 5,
    ADC6_Temp = 6,
    ADC7_U12v = 7
} adc_in_type;

/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
 *************************************************************************/
static inline void _delayFourCycles(uint16_t __count)
{
    if (__count == 0)
	__asm__ __volatile__( "rjmp 1f\n 1:" );
    // 2 cycles
    else
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"                              // 4 cycles/loop
		: "=w" (__count)
		: "0" (__count)
	);

}

/************************************************************************* 
 delay for a minimum of <us> microseconds
 the number of loops is calculated at compile-time from MCU clock frequency
 *************************************************************************/
#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/4000) )*us)/1000 )

void delay_ms0(uint8_t ms);
void delay_ms(uint16_t ms);
void delay_s(uint8_t s);

struct m2bytes {
    uint8_t one;
    uint8_t two;
};

void num_to_str(struct m2bytes *nbuf);

void num_to_lcd(uint8_t fs, uint8_t num);

void SPI_SendByte(int8_t byte);
void SPI_init(void);

void ADC_Init(void);
void SPI_deInit(void);
void ADC_Mux(adc_in_type input);
void ADC_Start(void);
//((ADCSRA & (1 << ADIF)) == 0)
uint16_t ADC_GetData(void);
#define ADCIsRun() (ADCSRA & (1 << ADSC))

int8_t delay_btn_check(int8_t *delay_var);
int16_t check_range(int16_t min, int16_t in, int16_t max);
float check_rangef(float min, float in, float max);

uint8_t eeprom_read_byte1(uint16_t addr);
void eeprom_write_byte1(uint16_t addr, uint8_t data);
void eeprom_read_buf(uint8_t *data, uint16_t addr, uint16_t len);
void eeprom_update_buf(uint16_t addr, uint8_t *data, uint16_t len);
