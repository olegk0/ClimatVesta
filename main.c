#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <math.h>

#include "io.h"
#include "disp.h"
#include "usart.h"
//#include "temp_18b20.h"

#define F_CPU 8000000UL  // Частота MHz

#include "vars.h"
#include "subs.h"

//----------------------------------------------

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

//-------------------- Timer on 0.5 sec -----------------------

ISR (TIMER1_OVF_vect)
{
    cli();
    TCNT1 = 0x10000 - (F_CPU / 256);

    SET_FLAG(f_e1sec);

//    SET_BIT(time_flag, tf_dps_en);
    sei();
}

//********************** Vent Enc INT**********************
void test_vent_enc(int8_t up)
{
    if (get_input(IN_BTN_V_UP) == get_input(IN_BTN_V_DWN)) {
	if (up) {
	    vent_enc++;
	} else {
	    vent_enc--;
	}
    }
}

ISR (PCINT0_vect) // IN_BTN_V_DWN Vent
{
    cli();
    test_vent_enc(1);
    sei();
}

ISR (PCINT2_vect) // IN_BTN_V_UP Vent
{
    cli();
    test_vent_enc(0);
    sei();
}

//********************** Temp Enc INT**********************

void test_temp_enc(int8_t up)
{
    delay(1);
    if (get_input(IN_BTN_T_UP) == get_input(IN_BTN_T_DWN)) {
	if (up) {
	    temp_enc++;
	} else {
	    temp_enc--;
	}
    }
}

ISR (INT0_vect) // IN_BTN_T_DWN Temp
{
    cli();
    test_temp_enc(1);
    sei();
}

ISR (INT1_vect) // IN_BTN_T_UP Temp
{
    cli();
    test_temp_enc(0);
    sei();
}

//********************** **********************
void set_def_vals(void)
{
    vars.setup_temp = 20;
    vars.vent_nom_seg = 4;
    vars.temp_seg = 2;
    vars.wmode = wmode_auto;
    vars.const_cnd_frost_t = COND_T_FROST;
    vars.pid_P_kfc = PID_P_KFC;
    vars.pid_I_kfc = PID_I_KFC;
    vars.vent_var_kfc = VENT_VAR_KFC;
    vars.temp_seg_delay = TEMP_OUT_DELAY;
    vars.cond_enabled = 0;
    vars.auto_defrost = 3;
}

void myread_eeprom(void)
{
    cli();
    eeprom_read_buf((uint8_t *) &vars, EEMEM_VARS, sizeof(vars));
//    eeprom_read_block(&vars, 10, sizeof(vars));
//    uint8_t tst = eeprom_read_byte(&ftst_ee);
    uint8_t tst = eeprom_read_byte1(EEMEM_TST);
    if (tst != 0xaa) {
	set_def_vals();
    }
}

void myupdate_eeprom(void)
{
    cli();
    eeprom_update_buf(EEMEM_VARS, (uint8_t *) &vars, sizeof(vars));
    if (eeprom_read_byte1(EEMEM_TST) != 0xaa) {
	eeprom_write_byte1(EEMEM_TST, 0xaa);
    }
    /*
     eeprom_update_block(&vars, 10, sizeof(vars));
     uint8_t tst = 0xaa;
     //eeprom_update_byte(&ftst_ee, &tst);
     */
}

/*
 uint8_t hex_to_num(int8_t sym)
 {
 if (sym >= '0' && sym <= '9')
 return (sym - '0');
 else if (sym >= 'A' && sym <= 'F')
 return (sym - 'A' + 10);
 return (255);
 }

 uint8_t str_to_num(struct m2bytes nbuf)
 {
 uint8_t num1, num2;

 num1 = hex_to_num(nbuf.one);
 if (num1 >= 0 && num1 <= 9)
 num1 *= 10;
 else
 return (255);
 num2 = hex_to_num(nbuf.two);
 if (num2 >= 0 && num2 <= 9)
 return (num2 + num1);
 return (255);
 }

 void num_to_hex(struct m2bytes *nbuf)
 {
 uint8_t tmp, num;
 //m2bytes nbuf;

 num = nbuf->one;
 tmp = num / 16;
 if (tmp > 9)
 nbuf->one = tmp + 'A' - 10;
 else
 nbuf->one = tmp + '0';
 num -= tmp * 16;
 if (num > 9)
 nbuf->two = num + 'A' - 10;
 else
 nbuf->two = num + '0';
 }
 */
#define DELAY_ENC_BTWN 500
#define DELAY_ENC_CKL_MS 30
void switch_temp(int8_t up)
{
    if (up) {
	if (get_output(OUT_BTN_T_UP)) {
	    pin_low(OUT_BTN_T_UP);
	    delay(DELAY_ENC_BTWN);
	    pin_low(OUT_BTN_T_DWN);
	} else {
	    pin_high(OUT_BTN_T_UP);
	    delay(DELAY_ENC_BTWN);
	    pin_high(OUT_BTN_T_DWN);
	}
    } else {
	if (get_output(OUT_BTN_T_UP)) {
	    pin_low(OUT_BTN_T_DWN);
	    delay(DELAY_ENC_BTWN);
	    pin_low(OUT_BTN_T_UP);
	} else {
	    pin_high(OUT_BTN_T_DWN);
	    delay(DELAY_ENC_BTWN);
	    pin_high(OUT_BTN_T_UP);
	}
    }
    delay_ms(DELAY_ENC_CKL_MS);
}

void switch_vent(int8_t up)
{
    if (up) {
	if (get_output(OUT_BTN_V_UP)) {
	    pin_low(OUT_BTN_V_UP);
	    delay(DELAY_ENC_BTWN);
	    pin_low(OUT_BTN_V_DWN);
	} else {
	    pin_high(OUT_BTN_V_UP);
	    delay(DELAY_ENC_BTWN);
	    pin_high(OUT_BTN_V_DWN);
	}
    } else {
	if (get_output(OUT_BTN_V_UP)) {
	    pin_low(OUT_BTN_V_DWN);
	    delay(DELAY_ENC_BTWN);
	    pin_low(OUT_BTN_V_UP);
	} else {
	    pin_high(OUT_BTN_V_DWN);
	    delay(DELAY_ENC_BTWN);
	    pin_high(OUT_BTN_V_UP);
	}
    }
    delay_ms(DELAY_ENC_CKL_MS);
}

void switch_cond(char on)
{
    /*    if (!on) {
     SET_FLAG(f_mode_hot);
     }
     */
    if (vars.cond_enabled) {
	if (on && vent_seg_cur > 0) {
	    CLR_FLAG(f_cond_ind_flash);
	    pin_high(OUT_FROST_SW);
	} else {
	    SET_FLAG(f_cond_ind_flash);
	    pin_low(OUT_FROST_SW);
	}
    } else {
	CLR_FLAG(f_cond_ind_flash);
	pin_low(OUT_FROST_SW);
    }
}

uint8_t test_cond_ON(void)
{
    return get_output(OUT_FROST_SW);
}

float calcTempFromADC(adc_in_type input)
{
    ADC_Mux(input);
    float resf = 0;
    for (int i = 0; i < ADC_MCNT; i++) {
	ADC_Start();
	while (ADCIsRun())
	    ;
	resf += ADC_GetData();
    }

    resf /= ADC_MCNT;

    resf = 3132.0f / resf - 1;
    resf = 100.0f / resf;

    resf = (TR_NTC_KFC * TR_NTC_T0_GRAD_K) / (TR_NTC_KFC + (TR_NTC_T0_GRAD_K * log(resf / TR_NTC_T0_RES_Kom))) - 273.15
	    + 0.5;

    return resf;
}

#include "calc.c"

void recycleSwitch(void) // эмуляция нажатия
{
    pin_low(OUT_BTN_Recycle);
    set_dir_out(OUT_BTN_Recycle);
    delay_ms(50);
//    pin_high(OUT_BTN_Recycle);
    set_dir_in(OUT_BTN_Recycle);
}

void prefAirSwitch(void) // эмуляция нажатия
{
    pin_low(OUT_BTN_MODE_UP_DOWN);
    set_dir_out(OUT_BTN_MODE_UP_DOWN);
    delay_ms(50);
    set_dir_in(OUT_BTN_MODE_UP_DOWN);
}

void hiAirSwitch(void) // эмуляция нажатия
{
    pin_low(IN_BTN_HI_MODE);
    set_dir_out(IN_BTN_HI_MODE);
    delay_ms(50);
    set_dir_in(IN_BTN_HI_MODE);
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

    pin_low(OUT_FROST_SW);
    set_dir_out(OUT_FROST_SW);
    pin_low(OUT_FROST_IND);
    set_dir_out(OUT_FROST_IND);
    set_dir_out(OUT_BTN_T_UP);
    set_dir_out(OUT_BTN_T_DWN);
    set_dir_out(OUT_BTN_V_UP);
    set_dir_out(OUT_BTN_V_DWN);
}

void setup_display(void)
{
    Send_7219(0x09, 0x00); //режим декодирования
    int8_t br_dsp = DISP_BRG_MIN;
    if (!get_input(IN_NIGHT_BR)) {
	br_dsp = DISP_BRG_MAX;
    }
    Send_7219(0x0A, br_dsp); //яркость
    Send_7219(0x0B, DISP_LEN - 1); //сколько разрядов используем
    Send_7219(0x0C, 1); //включим индикатор
    Send_7219(0x0F, 0x00);
}

void display_off(void)
{
    Send_7219(0, 0);
    Send_7219(0x0C, 0); //выключим индикатор
}

int main(void)
{
    wdt_reset();
    cli();
    wdt_enable(WDTO_4S);
    wdt_reset();
    int8_t disp_num = 0;
    uint8_t disp_bar = 0;
    int8_t tmp;
    uint8_t time_cnt;
    int8_t smode = 0;
    int8_t bdelay = 0;

    bootVarsInit();

    setup_pins();

    while (!get_input(IN_ACC)) { //ACC off
	wdt_reset();
	delay_s(1); // wait for ACC
	display_off();
    }

    delay_ms(10);
    myread_eeprom();

    //TODO sei();

    ADC_Init();
    USART_Init();
    //int_to_uart("T", 0);

    SPI_init();
    delay_ms(1);

    //int_to_uart("T", 1);
    Send_7219(0, 0);
    delay_ms(1);
    Clear_7219();
    Send_7219(0x09, 0x00); //режим декодирования
    Send_7219(0x0A, 0x01); //яркость
    Send_7219(0x0B, DISP_LEN - 1); //сколько разрядов используем
    Send_7219(0x0D, 0x00);
    Send_7219(0x0E, 0x00);
    Send_7219(0x0F, 0x00);
    delay_ms(1);

    //int_to_uart("T", 2);
    wdt_reset();

    disp_num = vars.setup_temp;
    Disp_Num(disp_num, 0);
    Send_7219(0x0C, 1); //включим индикатор
    disp_bar = vars.vent_nom_seg;
    Disp_Bars(disp_bar);

    cli();
    //int_to_uart("T", 3);
//timer init
    TCCR1A = 0;  	//Т/С1 отсоединен от вывода ОС1, режим ШИМ отключен
    TCCR1B = 4;  	//Коэффициент деления частоты системной	синхронизации = 256
    TCNT1 = 0x10000 - (F_CPU / 512); //Инициализируем счетный регистр

    TIFR1 = 0;       //Сбрасываем все флаги прерываний от Т/С1
    TIMSK1 = 1 << TOIE1;  	//Разрешаем прерывание при переполнении Т/С1

    //interrupt setup
    EICRA = 0; // External Interrupt Control Register A
    EIMSK = 0; //External Interrupt Mask Register
    EIMSK = (1 << INT0) | (1 << INT1);
    EICRA = (1 << ISC00) | (1 << ISC10); // set INT0 + INT1 to trigger on ANY logic change

    //int_to_uart("T", 4);

    //set by IN_BTN_V_DWN  + IN_BTN_V_UP
    PCMSK0 = (1 << PCINT0);  // set PCINT0 to trigger an interrupt on state change
    PCMSK2 = (1 << PCINT23);  // set PCINT23 to trigger an interrupt on state change
    PCICR = (1 << PCIE0) | (1 << PCIE2);    // Pin Change Interrupt Control Register

//todo	GIMSK = 0;     	//Запрет внешних прерываний
    //int_to_uart("T", 5);
    delay_ms(10);
    sei();

    vent_enc = 0;
    temp_enc = 0;
    time_cnt = 0;

    wdt_reset();
    /*    vent_seg_cur = VENT_SEGs;
     temp_seg_cur = TEMP_SEGs;
     */
    vent_seg_cur = -1;
    temp_seg_cur = -1;
    set_vent_seg(0, 1);
    set_temp(0, 1);

    wdt_enable(WDTO_4S);
    //int_to_uart("T", 10);

    tmp = 5;
    while (tmp--) {
	wdt_reset();
	Ts = getTempSalon();
	delay_ms(1);
	Ti = getTempIsparit();
	delay_ms(100);
    }

    if (vars.auto_defrost) {
	tmp = vars.auto_defrost - 3;
	if (Ts < tmp) {
	    smode = smode_hi_mode;
	}
    }

    //Ipid = (float) vars.setup_temp;
    //Ipid = check_rangef(-(100 / PID_I_KFC), Ipid - Ts, 100 / PID_I_KFC);

    setup_display();
    wdt_reset();
    /*
     if (!get_input(IN_FROST_BTN)) { // Если при включении нажата Setup сразу
     smode = 1;
     tmp = 'c';
     }
     */
    vent_enc = 0;
    temp_enc = 0;

    while (1) {
	if (get_input(IN_ACC)) { //ACC on

	    if (smode == smode_hi_mode) {	//HI_MODE
		time_cnt = 0;
		tmp = 10;
		if (get_input(IN_BTN_HI_MODE)) { // не нажата
		    recycleSwitch(); //recycle mode on
		    hiAirSwitch(); // обдув на лобовое
		    smode = smode_hi_mode_work;
		}

		set_temp(100, 1);
		set_vent_seg(tmp, 1);

		Disp_Num_Seg(NUM0_SEG, 'H', 0);
		Disp_Num_Seg(NUM1_SEG, 'I', 0);

	    } else if (smode == smode_hi_mode_work) {		//HI_MODE WORK
		//if (time_cnt > 2)
		{
		    uint8_t ti = get_input(IN_BTN_HI_MODE);
		    uint8_t tp1 = get_input(OUT_BTN_MODE_UP_DOWN);
		    if ((time_cnt > HI_HOT_TMO_S && (int) getTempSalon() > HI_HOT_T_c) || !ti || !tp1) {
			smode = 0;
			time_cnt = 0;
			recycleSwitch(); //recycle mode off
			if (ti) { // Если не нажата HI переходим в выбранный рабочий режим
			    prefAirSwitch(); // переключаем в любимый режим обдува
			}
		    }
		}

		if (vent_enc) {
		    tmp = check_range(0, tmp + vent_enc, VENT_SEGs);
		    time_cnt = 0;
		    vent_enc = 0;
		}

		set_temp(100, 1); // Синхронизация Т к 100%
		set_vent_seg(tmp, 1);

		//Disp_Num_Seg(NUM0_SEG, 'H', 0);
		//Disp_Num_Seg(NUM1_SEG, 'I', 0);
		Disp_Bars(tmp);

	    } else if (smode) {		//Setup mode

		if (time_cnt > SETUP_TMO_S) {
		    smode = 0;
		    time_cnt = 0;
		    disp_num = vars.setup_temp;
		    //cbtn = 0;
		    if (vars.wmode == wmode_auto) {
			set_vent_seg(1, 1);
			set_temp(0, 1);
		    }
		    continue;
		}

		if (vent_enc) { // переключаем пункты настройки
		    smode = check_range(1, smode + vent_enc, smode_SETUP_LAST - 1);
		    time_cnt = 0;
		    vent_enc = 0;
		    disp_num = smode - 1;
		    tmp = 'c';
		}

		int8_t lf = 0;
		if (time_cnt > 0) {
		    lf = 1;
		}

		if (temp_enc) { //изменение параметра
		    lf = 1;
		    time_cnt = 0;
		}

		if (lf) {
		    tmp = 0;
		    switch (smode) {
		    case smode_manual: // в ручной режим
			tmp = 'r';
			vars.wmode = check_range(0, vars.wmode + temp_enc, 1);
			if (vars.wmode == wmode_auto) {
			    disp_num = 'A';
			} else {
			    disp_num = 'P';
			}
			break;
		    case smode_auto_defrost: // авто подогрев лобового при включении зажигания если температура < 0
			tmp = 'H';
			vars.auto_defrost = check_range(0, vars.auto_defrost + temp_enc, 5);
			disp_num = vars.auto_defrost;
			break;
		    case smode_cnd_frost: //температура обмерзания кондиционера
			tmp = 'C';
			vars.const_cnd_frost_t = check_range(0, vars.const_cnd_frost_t + temp_enc, 9);
			disp_num = vars.const_cnd_frost_t;
			break;
		    case smode_seg_delay: // пропуск изменений заслонки
			tmp = 'd';
			vars.temp_seg_delay = check_range(0, vars.temp_seg_delay + temp_enc, 5);
			disp_num = vars.temp_seg_delay;
			break;
		    case smode_vent_kfc: //кфц отклонения вентилятора на изменения Т
			tmp = 'u';
			vars.vent_var_kfc = check_range(0, vars.vent_var_kfc + temp_enc, 20);
			disp_num = vars.vent_var_kfc;
			break;
		    case smode_pid_P_kfc: //P кфц
			//tmp = 'P';
			vars.pid_P_kfc = check_range(1, vars.pid_P_kfc + temp_enc, 20);
			disp_num = vars.pid_P_kfc;
			break;
		    case smode_pid_I_kfc: //I кфц
			//tmp = 'I';
			vars.pid_I_kfc = check_range(1, vars.pid_I_kfc + temp_enc, 30);
			disp_num = vars.pid_I_kfc;
			break;
		    case smode_load_def: // загрузить данные по умолчанию
			tmp = 'L';
			disp_num = '?';
			if (!get_input(IN_FROST_BTN)) { // press setup mode
			    set_def_vals();
			    smode = 1;
			    disp_num = 0;
			    tmp = 'c';
			    time_cnt = 0;
			}
			break;
		    case smode_disp_T_isp: //просмотр датчиков
			disp_num = getTempIsparit();
			break;
		    case smode_disp_T_salon:
			disp_num = getTempSalon();
			break;
		    }
		    temp_enc = 0;
		}

		if (tmp) {
		    Disp_Num_Seg(NUM0_SEG, tmp, 0);
		    Disp_Num_Seg(NUM1_SEG, disp_num, 0);
		} else {
		    Disp_Num(disp_num, 0);
		}
		Disp_Bars(smode);

	    } else { // work mode

		if (vars.wmode == wmode_auto) {
		    switch (time_cnt) {
		    case 0:
			setup_display();
			time_cnt++;
			break;
		    case 1 ... 6:
			break;
		    case 7:
			Ti = getTempIsparit();
			time_cnt = 8;
			break;
		    case 8:
			break;
		    case 9:
			Ts = getTempSalon();
			float t = Ts + 0.4;
			disp_num = t;
			time_cnt++;
			break;
		    case 10:
			break;
		    case 11: //calc
			calcVal();
			time_cnt++;
			break;
		    case 12:
			break;
		    case 13:
			set_temp(temp_out_prc, 1);
			time_cnt++;
			break;
		    case 14:
			break;
		    case 15:
			set_vent_prc(vent_add_prc, 1);
			disp_bar = vent_seg_cur;
			SET_FLAG(f_num_dot)
			time_cnt++;
			break;
		    default: //  цикл
			time_cnt = 0;
			CLR_FLAG(f_num_dot)
		    }

		    if (vent_enc) {
			vars.vent_nom_seg = check_range(0, vars.vent_nom_seg + vent_enc, VENT_SEGs + 2);
			disp_bar = vars.vent_nom_seg;
			time_cnt = 10;
			vent_enc = 0;
		    }

		    if (temp_enc) {
			vars.setup_temp = check_range(15, vars.setup_temp + temp_enc, 30);
			disp_num = vars.setup_temp;
			time_cnt = 0;
			temp_enc = 0;
		    }

		} else { // manual

		    if (time_cnt > 10) {
			setup_display();
			time_cnt = 0;
		    }

		    if (vent_enc) {
			vars.vent_nom_seg = check_range(0, vars.vent_nom_seg + vent_enc, VENT_SEGs);
			vent_enc = 0;
		    }

		    if (temp_enc) {
			vars.temp_seg = check_range(0, vars.temp_seg + temp_enc, TEMP_SEGs);
			temp_enc = 0;
		    }
		    disp_bar = vars.vent_nom_seg;
		    disp_num = seg2Prc(TEMP_SEGs, vars.temp_seg);
		    set_vent_seg(vars.vent_nom_seg, 0);
		    set_temp(disp_num, 0);
		    switch_cond(1);
		}
// логика
// кнопки
		if (!get_input(IN_BTN_HI_MODE)) { // to hi hot mode
		    if (delay_btn_check(&bdelay)) {
			smode = smode_hi_mode;
		    }
		    time_cnt = 0;
		} else if (!get_input(IN_FROST_BTN)) { // to setup mode
		    SET_FLAG(f_cond_btn_press);
		    if (delay_btn_check(&bdelay)) {
			smode = 1;
			disp_num = 0;
			tmp = 'c';
			CLR_FLAG(f_cond_btn_press);
			vent_enc = 0;
			temp_enc = 0;
		    }
		    time_cnt = 0;
		} else {
		    if (TST_FLAG(f_cond_btn_press)) {
			if (vars.cond_enabled) {
			    vars.cond_enabled = 0;
			} else {
			    vars.cond_enabled = 1;
			}
			CLR_FLAG(f_cond_btn_press);
		    }
		    bdelay = 0;
		}

//дисплей
		Disp_Num(disp_num, TST_FLAG(f_num_dot));
		Disp_Bars(disp_bar);
	    }

	    if (TST_FLAG(f_e1sec)) {
		time_cnt++;
		if (bdelay) {
		    bdelay++;
		}
		CLR_FLAG(f_e1sec);

		if (TST_FLAG(f_cond_ind_flash)) {
		    pin_low(OUT_FROST_IND);
		}
	    }

	} else { //poweroff prepare
	    time_cnt = 0;
	    wdt_reset();
	    myupdate_eeprom();
	    for (uint8_t i = 0; i < VENT_SEGs + 2; i++) {
		switch_vent(0);
	    }
	    for (uint8_t i = 0; i < TEMP_SEGs; i++) {
		switch_temp(0);
	    }
	    display_off();
	    delay_ms(10);

	    wdt_enable(WDTO_15MS); // reboot
	    while (1) {
		delay_s(1);
	    }
	}

	delay_ms(100);

	if (vars.cond_enabled) {
	    pin_high(OUT_FROST_IND);
	} else {
	    pin_low(OUT_FROST_IND);
	    switch_cond(0);
	}

	wdt_reset();
    }

}

