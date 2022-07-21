//#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
//#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>

#include "io.h"
#include "usart.h"

#include "vars.h"
#include "subs.h"

void int_to_uart(char *id, int16_t num)
{
//    struct m2bytes nbuf;
    //USART_PutChar('\x0d');
    USART_SendStr(id);
    USART_SendStr(":");
    /*    nbuf.one = num;
     num_to_str(&nbuf);
     USART_PutChar(nbuf.one);
     USART_PutChar(nbuf.two);
     */
    char buf[10];
    snprintf(buf, sizeof(buf), "%d", num);
    USART_SendStr(buf);
    USART_PutChar(';');
//    buf[cnt];
}

void float_to_uart(char *id, float num)
{
    int_to_uart(id, num);
    /*    USART_SendStr(id);
     USART_SendStr(":");
     int8_t buf[10];
     int8_t cnt = snprintf(buf, sizeof(buf), "%g", num);
     USART_SendStr(buf);
     USART_Putint8_t(';');
     */
}

void uart_nl(void)
{
    USART_PutChar('\x0d');
    USART_PutChar('\n');
}

ISR( TIMER1_OVF_vect)
{
    cli();
    TCNT1 = 0x10000 - (F_CPU / 256);

    SET_FLAG(f_e1sec);

//    SET_BIT(time_flag, tf_dps_en);
    sei();
}

float getADC()
{
    float resf = 0;
    for (int i = 0; i < ADC_MCNT; i++) {
	ADC_Start();
	while (ADCIsRun())
	    ;
	resf += ADC_GetData();
    }

    resf /= ADC_MCNT;
    return resf;
}


float calcTemp(void)
{
    ADC_Mux(ADC6_Temp);
    float resf = getADC();

    resf = 3132.0 / resf - 1;
    resf = 100.0 / resf;

    resf = (TR_NTC_KFC * TR_NTC_T0_GRAD_K) / (TR_NTC_KFC + (TR_NTC_T0_GRAD_K * log(resf / TR_NTC_T0_RES_Kom))) - 273.15
	    + 0.5;

    return resf;
}

void set_def_vals(void)
{
//    vars.setup_temp = 15;
//    vars.vent_nom_seg = 4;
}

void myread_eeprom(void)
{
    cli();
    eeprom_read_block((uint8_t *) &vars, (void *) EEMEM_VARS, sizeof(vars));
    uint8_t tst = eeprom_read_byte((void *) EEMEM_TST);
    if (tst != 0xaa) {
	set_def_vals();
    }
}

void myupdate_eeprom(void)
{
    cli();
    eeprom_update_block((uint8_t *) &vars, (void *) EEMEM_VARS, sizeof(vars));
    eeprom_update_byte((void *) EEMEM_TST, 0xaa);
}



void calcCond(void)
{
}

void calcVent(float delta, float id)
{
}



void calcVal(void)
{
    calcCond();

    int_to_uart("Ti", Ti);
    int_to_uart("Ts", Ts);
    int_to_uart("St", vars.setup_temp);
    float_to_uart("lD", last_delta);
    float delta = (float) vars.setup_temp;
    delta = check_rangef(-40.0, delta - Ts, 40.0);
    if (fabs(delta) < 0.2) {
    delta = 0.0;
    }
    float_to_uart("Dt", delta);
    float id = delta - last_delta;
    last_delta = delta;
    float_to_uart("Id", id);

    Ipid += delta;

    float pid = (delta + (Ipid * (float) PID_I_KFC) / 100.0 + (id * (float) PID_D_KFC) / 100.0)
        * (float) vars.pid_P_kfc;

    float_to_uart("Ip", Ipid);
    float_to_uart("pid", pid);
/*
    if (!(pid > 100.0 && delta > 0.0)) {
    if (!(pid < 0.0 && delta < 0.0)) {
        Ipid += delta;
    }
    }
*/

    temp_out_delay++;
    if (temp_out_delay > vars.temp_seg_delay) {
    temp_out_delay = 0;
    temp_out_prc = (check_rangef(0, pid, 101) + temp_out_prc) / 2;
    }

    int_to_uart("out", temp_out_prc);

    calcVent(delta, id);

    uart_nl();
}


void setup_pins(void)
{
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    DDRB = 0; //dir to in
    DDRC = 0; //dir to in
    DDRD = 0; //dir to in
    /*    set_dir_in(IN_BTN_HI_MODE);
     set_dir_in(IN_BTN_SETUP);
     set_dir_in(IN_BTN_T_UP);
     set_dir_in(IN_BTN_T_DWN);
     set_dir_in(IN_BTN_V_UP);
     set_dir_in(IN_BTN_V_DWN);
     set_dir_in(IN_ACC);
     IN_NIGHT_BR
     set_dir_in(IN_FROST_BTN);*/

    pin_low(OUT_CH_1);
    set_dir_out(OUT_CH_1);
    pin_low(OUT_CH_2);
    set_dir_out(OUT_CH_2);

    pin_low(OUT_CH_LED_1);
    set_dir_out(OUT_CH_LED_1);
    pin_low(OUT_CH_LED_2);
    set_dir_out(OUT_CH_LED_2);
}

int main(void)
{
    uint8_t time_cnt = 0;
    wdt_reset();
    cli();
    wdt_enable(WDTO_2S);
    wdt_reset();

    setup_pins();

    myread_eeprom();
    ADC_Init();
    USART_Init();

//timer init
    TCCR1A = 0;         //Т/С1 отсоединен от вывода ОС1, режим ШИМ отключен
    TCCR1B = 4;         //Коэффициент деления частоты системной синхронизации = 256
    TCNT1 = 0x10000 - (F_CPU / 512); //Инициализируем счетный регистр

    TIFR = 0;       //Сбрасываем все флаги прерываний от Т/С1
    TIMSK = 1 << TOIE1;        //Разрешаем прерывание при переполнении Т/С1

    sei();
    wdt_reset();


    Ti=-13.0;
    Ts=2;

    vars.setup_temp = 20;
    vars.vent_nom_seg = 4;
    vars.temp_seg = 2;
    vars.wmode = wmode_auto;
    vars.const_cnd_frost_t = COND_T_FROST;
    vars.pid_P_kfc = PID_P_KFC;
    vars.vent_var_kfc = VENT_VAR_KFC;
    vars.temp_seg_delay = TEMP_OUT_DELAY;
    vars.cond_enabled = 0;
    vars.auto_defrost = 3;


    uart_nl();
    int_to_uart("******begin************", 0);
    uart_nl();

    for(int c=0;c< 500;c++){
	uart_nl();
	int_to_uart("time",c*10);
	uart_nl();
	calcVal();
	Ts += 0.05;



	delay_ms(100);
	wdt_reset();
    }

    return 0;
}

