#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vars.h"
#include <math.h>
#include <unistd.h>


#define DELAY_ENC_CKL_MS 30

#define IN_ADC_T_FROST  1
#define IN_ADC_T_SALON  2

#define SWITCH_BIT(var,bit)     {(var) = (var) ^ (1<<bit);}
#define SET_BIT(var,bit)        {(var) = (var) | (1<<bit);}
#define CLR_BIT(var,bit)        {(var) = (var) & ~(1<<bit);}
#define TST_BIT(var,bit)        ((var) & (1<<bit))

float Tsol;
float To;

void int_to_uart(char *id, int16_t num)
{
    printf("%s:%d;", id, num);
}

void float_to_uart(char *id, float num)
{
    printf("%s:%2.2f;", id, num);
}

void uart_nl(void)
{
    putchar('\n');
}

float check_rangef(float min, float in, float max)
{
    if (in > max)
	in = max;
    if (in < min)
	in = min;
    return in;
}

int16_t check_range(int16_t min, int16_t in, int16_t max)
{
    if (in > max)
	in = max;
    if (in < min)
	in = min;
    return in;
}

void delay_ms(int ms)
{
    usleep(ms * 1000);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
char OUT_FROST_SW =0;
void switch_cond(char on)
{
/*
    if (!on) {
	SET_FLAG(f_mode_hot);
    }
*/
    if (vars.cond_enabled) {
    if (on && vent_seg_cur > 0) {
        CLR_FLAG(f_cond_ind_flash);
        OUT_FROST_SW = 1;
    } else {
        SET_FLAG(f_cond_ind_flash);
        OUT_FROST_SW = 0;
    }
    } else {
	CLR_FLAG(f_cond_ind_flash);
	OUT_FROST_SW = 0;
    }
    int_to_uart("Mfs", OUT_FROST_SW);
}

uint8_t test_cond_ON()
{
    return OUT_FROST_SW;
}


static int cvent = 0;
static int ctemp = 0;
void switch_vent(int8_t up)
{
//    printf("switch_vent:%d\n",up);
    if (up)
	cvent++;
    else
	cvent--;

    if (cvent < 0)
	cvent = 0;
    if (cvent > 8)
	cvent = 8;

}

void switch_temp(int8_t up)
{
//    printf("switch_temp:%d\n",up);
    if (up)
	ctemp++;
    else
	ctemp--;

    if (ctemp < 0)
	ctemp = 0;
    if (ctemp > 15)
	ctemp = 15;

}

float calcTempFromADC(char ch);

//*************************************************************************************

#include "../calc.c"

//****************************************************


float calcTempFromADC(char ch)
{
    switch(ch){
    case IN_ADC_T_FROST:
	if(test_cond_ON()){
	    return Ti - (15.0f / (float)To);
	}else{
	    if(Ti < To)
		return Ti + ((float)To / 20.0f);
	}
        return Ti;
        break;
    case IN_ADC_T_SALON:
    {
	float Tin = Ti + Tsol;
	float Tz = ((70.0 + Tin) * seg2Prc(TEMP_SEGs , ctemp) + Tin * (100 - seg2Prc(TEMP_SEGs , ctemp))) / 100.0f;
	float Vl = (40.0f * seg2Prc(VENT_SEGs , cvent))/100.0f;
	float k1 = 150.0f/(Vl+1.0f);
	float k2 = (1500.0f - (k1 + Vl )) * 100.0f / 1500.0f;
	float k3 = k1 * ( 100.0f - k2) / (k1 + Vl);
	float k4 = 100.0f - k2 -k3;
	return ( k2 * Ts + k3 * Tin + k4 * Tz) / 100.0f;
/*        if(Ts < (vars.setup_temp + 5))
            return Ts + 0.1;
        else if(Ts >= (vars.setup_temp - 5))
            return Ts - 0.1;
	else
            return Ts;*/
    }
        break;
    }



seg2Prc(VENT_SEGs , cvent);

seg2Prc(TEMP_SEGs , ctemp);


test_cond_ON();

    return 0;
}




int main(void)
{

    bootVarsInit();

#if 0
    Tsol = 0.0f;
    To = -30.0f;
    Ts = To;
    vars.cond_enabled = 0;
#else
    Tsol = 3.0f;//7
    To = 30.0f;
    Ts = 40;
    vars.cond_enabled = 1;
#endif

    int ret = 0;
    Ti = To;


    vars.setup_temp = 20;
    vars.vent_nom_seg = 4;
    vars.temp_seg = 2;
    vars.wmode = wmode_auto;
    vars.const_cnd_frost_t = COND_T_FROST;
    vars.pid_P_kfc = PID_P_KFC;
    vars.pid_I_kfc = PID_I_KFC;
    vars.vent_var_kfc = VENT_VAR_KFC;
    vars.temp_seg_delay = TEMP_OUT_DELAY;

    vars.auto_defrost = 3;

    int time_cnt = 0;
    int disp_num;
    int disp_bar;
    printf("******begin************\n");

    //Ipid = (float) vars.setup_temp;
    //Ipid = check_rangef(-(100 / PID_I_KFC), Ipid - Ts, 100 / PID_I_KFC);
    //Ipid = 0.0f;

    for (int c = 0; c < 3000; c++) {

	switch (time_cnt) {
	case 0:
//	    setup_display();
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
	    time_cnt++;
	    break;
	default: //  цикл
	    time_cnt = 0;
//	printf("time:%dc  %dm\n",c*10, c/6);
	}

	time_cnt++;
    }

}
