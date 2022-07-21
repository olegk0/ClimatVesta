void bootVarsInit(void)
{
    vent_add_prc = 0;
    temp_out_prc = 0;
    temp_out_delay = TEMP_OUT_DELAY;
    bootUp_delay = 0;

//    powerUp_time_sec = 0;
    Ipid = 0.0f;
    last_delta = 0.0f;
    flags = 0;
    SET_FLAG(f_mode_hot);
    CLR_FLAG(f_cond_btn_press);
//    downHeadFlag = 0;
//    upHeadFlag = 0;
    cond_needs_of_cool = 0;
}

float getTempSalon(void)
{
    /*   float ts =calcTempFromADC(IN_ADC_T_SALON);
     if(ts < )
     cond_over_cool_delta
     return ts;
     */
    return calcTempFromADC(IN_ADC_T_SALON);
}

float getTempIsparit(void)
{
    return calcTempFromADC(IN_ADC_T_FROST);
}

uint8_t prc2Seg(uint8_t segs, uint8_t prc)
{
    prc = check_range(0, prc, 100);
    uint16_t tmp = (segs * prc + segs) / 100;
    return tmp;
}

uint8_t seg2Prc(uint8_t segs, uint8_t seg)
{
    //seg = check_range(0, seg, segs);
    uint16_t tmp = (seg * 100) / segs;
    return tmp;
}

void set_vent_seg(uint8_t seg_vent, int8_t full_sync)
{
    seg_vent = check_range(0, seg_vent, VENT_SEGs);

    if (seg_vent == 0) { //add sync
	switch_vent(0);
	switch_cond(0); // выключаем кондиционер
    } else if (seg_vent == VENT_SEGs) {
	switch_vent(1);
    }

    if (vent_seg_cur != seg_vent) {
	uint8_t seg_v = seg_vent;
	if (full_sync) {
	    for (uint8_t i = 0; i < vent_seg_cur + 2; i++) {
		switch_vent(0);
	    }
	    delay_ms(DELAY_ENC_CKL_MS);
	    for (uint8_t i = 0; i < seg_vent; i++) {
		switch_vent(1);
	    }
	} else {
	    int8_t up = 0;
	    if (seg_vent > vent_seg_cur) {
		seg_vent = seg_vent - vent_seg_cur; // delta
		up = 1;
	    } else {
		seg_vent = vent_seg_cur - seg_vent; // delta
	    }
	    for (uint8_t i = 0; i < seg_vent; i++) {
		switch_vent(up);
	    }
	}
	vent_seg_cur = seg_v;
    }
}

void set_vent_prc(uint8_t prc, int8_t full_sync)
{
    uint8_t seg_vent = prc2Seg(VENT_SEGs, prc);
    set_vent_seg(seg_vent, full_sync);
}

void set_temp(uint8_t prc, int8_t full_sync)
{
    uint8_t seg_temp = prc2Seg(TEMP_SEGs, prc);

    if (seg_temp == 0) { //add sync
	switch_temp(0);
    } else if (seg_temp == TEMP_SEGs) {
	switch_temp(1);
    }

    if (temp_seg_cur != seg_temp) {
	uint8_t seg_t = seg_temp;
	if (full_sync) {
	    int8_t up = (temp_seg_cur > TEMP_SEGs / 2);
	    for (uint8_t i = 0; i < TEMP_SEGs / 2 + 2; i++) {
		switch_temp(up);
	    }
	    if (up) {
		seg_temp = TEMP_SEGs - seg_temp;
	    }
	    up = !up;
	    delay_ms(DELAY_ENC_CKL_MS);
	    for (uint8_t i = 0; i < seg_temp; i++) {
		switch_temp(up);
	    }
	} else {
	    int8_t up = 0;
	    if (seg_temp > temp_seg_cur) {
		seg_temp = seg_temp - temp_seg_cur; // delta
		up = 1;
	    } else {
		seg_temp = temp_seg_cur - seg_temp; // delta
	    }
	    for (uint8_t i = 0; i < seg_temp; i++) {
		switch_temp(up);
	    }
	}
	temp_seg_cur = seg_t;
    }
}
//****************************************************************************************

uint8_t cond_cicle = 0; // Время работы кондея в циклах (сейчас цикл около 15 сек.)

void calcCond(void)
{

    //if (Ti > vars.setup_temp) {
//    if (Ti > (vars.setup_temp - COND_WORK_DELTA_T1)/* && Ts > (float) vars.setup_temp*/) {
    /*	CLR_FLAG(f_mode_hot);
     } else if (downHeadFlag > 0) {
     if (upHeadFlag == 0) {
     SET_FLAG(f_mode_hot);
     }
     } else if (upHeadFlag > 0) {
     CLR_FLAG(f_mode_hot);
     }
     */
    if (Ti > (vars.setup_temp + COND_UP_SETUP_T) || cond_needs_of_cool > 2) {
	CLR_FLAG(f_mode_hot);
    } else {
	SET_FLAG(f_mode_hot);
    }

    char c_on = 0;
    if (TST_FLAG(f_mode_frost_cicle_on)) {
	//if (Ti > (vars.setup_temp - COND_WORK_DELTA_T * 2) || cond_cicle < 5) {
	if (Ts > vars.setup_temp || cond_cicle < 5 || temp_out_prc < 10) {
	    c_on = 1;
	}
    } else {
	if (Ti > (vars.setup_temp - COND_WORK_DELTA_T) && Ts > (vars.setup_temp - COND_OVERCOOL_DELTA_T)) {
	    c_on = 1;
	}
    }

    if (c_on && Ti > vars.const_cnd_frost_t) {
	SET_FLAG(f_mode_frost_cicle_on);
    } else {
	CLR_FLAG(f_mode_frost_cicle_on);
    }

    char mode_cond_on = 0;
    if ((!TST_FLAG(f_mode_hot) || test_cond_ON()) && TST_FLAG(f_mode_frost_cicle_on)) {
	mode_cond_on = 1;
	if (cond_cicle < 64)
	    cond_cicle++;
    } else {
	cond_cicle = 0;
    }

    switch_cond(mode_cond_on);

    int_to_uart("Mh", TST_FLAG(f_mode_hot));
    int_to_uart("Mf", TST_FLAG(f_mode_frost_cicle_on));
    int_to_uart("Mc", mode_cond_on);
}

void calcVent(float id)
{
    float vent_nom_prc = seg2Prc(VENT_SEGs, vars.vent_nom_seg);
    //delta = (delta / (float) vars.setup_temp) * (float) VENT_ADD_CTRL_CFC;

    float p = last_delta * (float) vars.vent_var_kfc/* * (float) vars.pid_P_kfc*/;
    //float_to_uart("Vp", p);

    if (id < 0.0f) {
	p = -p;
    }

    float rng = vent_nom_prc / 2.0f;
    p = check_rangef(-rng, p, rng);
    //float_to_uart("Vn", p);

    vent_add_prc = check_rangef(0.0f, p + vent_nom_prc, 100.0f);
    int_to_uart("Va", vent_add_prc);
}

void calcVal(void)
{
    if (bootUp_delay < 4) {  // Инициализация некоторых значений при старте.
	if (bootUp_delay < 2) {
	    Ipid = (Ts - Ti);
	    if (Ipid < 0) {
		Ipid = 0;
	    } else {
		Ipid = Ipid * 2.0f;
	    }
	}

	//SET_FLAG(f_mode_hot);
	switch_cond(0);
	bootUp_delay++;
    } else {
	calcCond();
    }

    float_to_uart("Ti", Ti);
    float_to_uart("Ts", Ts);

    float delta = (float) vars.setup_temp;
    if (vars.cond_enabled) {
	delta += COND_UP_SETUP_T; // если активен кондей - сместим установку температуры вверх
    }

    int_to_uart("St", delta);
    float_to_uart("lD", last_delta);

    delta = check_rangef(-40.0f, delta - Ts, 40.0f);

    float delta_v_cfc;
    float id;
    if (fabs(delta) < 0.2f) {
	delta = 0.0f;
	id = -last_delta;
	delta_v_cfc = 0;
    } else {
	id = delta - last_delta;
	delta_v_cfc = fabs(id / delta);
    }

    float_to_uart("Dt", delta);
    float_to_uart("Id", id);

    /*
     downHeadFlag <<= 1;
     upHeadFlag <<= 1;
     if (delta > 0.0f) {
     downHeadFlag |= 1;
     } else if (delta < 0.0f) {
     upHeadFlag |= 1;
     }
     */

    last_delta = delta;

    float pid = (delta + (Ipid * (float) vars.pid_I_kfc) / 10.0f) * (float) vars.pid_P_kfc;

    //if (!(pid > 100.0f && delta > 0.0f) && !(pid < -100.0f && delta < 0.0f)) {
    //if ((pid > 100.0f && delta > 0.0f) || (pid < -100.0f && delta < 0.0f)) {
    if ((pid > 100.0f && delta > 0.0f) || (Ipid < 0.0f && delta < 0.0f)) {
    } else {
	Ipid += delta;
    }

    float_to_uart("Ip", Ipid);
    float_to_uart("pid", pid);

    if (delta < 0.0f && temp_seg_cur <= TEMP_SEGs) {
	if (delta_v_cfc < D_COND_KFC || id < 0.0) {
	    cond_needs_of_cool += fabs(delta);
	}
    } else {
	cond_needs_of_cool = 0;
    }

    int_to_uart("CO", cond_needs_of_cool);

 /*   temp_out_delay++;
    if (temp_out_delay > vars.temp_seg_delay) {
	temp_out_delay = 0;
	*/
	//temp_out_prc = (check_rangef(0, pid, 101) + temp_out_prc) / 2;
    temp_out_prc = check_rangef(0, pid, 101);
    //}
    /*
     int_to_uart("UHF", upHeadFlag);
     int_to_uart("DHF", downHeadFlag);
     */
    int_to_uart("out", temp_out_prc);

    //calcVent(id);
    calcVent(pid);

    uart_nl();
}
