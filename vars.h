//#include <avr/pgmspace.h>

#define MAX_RXUDATA	8
#define MIN_RXUDATA	4

//********************************
#define VENT_SEGs	8
#define TEMP_SEGs	15

//********************************TermoRes**************************
#define ADC_MCNT 		6
#define TR_NTC_T0_GRAD_K	293.15f
#define TR_NTC_T0_RES_Kom 	14.0f
#define TR_NTC_KFC		2800.0f

//**************************Bright Disp******************************
#define DISP_BRG_MIN	1
#define DISP_BRG_MAX	7

//********************************
#define SETUP_TMO_S	5
#define HI_HOT_TMO_S	240 //Оттаивание лобового сек.
#define HI_HOT_T_c	10 //Оттаивание лобового минимальная температура.

#define VENT_VAR_KFC		10
//#define VENT_TO_HOT_SW_SEG	3
#define COND_WORK_DELTA_T	5
#define COND_UP_SETUP_T		2.0
#define COND_T_FROST		2
#define COND_OVERCOOL_DELTA_T	0
//********************************

#define PID_P_KFC	10
#define PID_I_KFC	1 // / 10
#define D_COND_KFC	0.01

#define TEMP_OUT_DELAY	2
//#define VENT_ADD_CTRL_CFC	30

//volatile uint8_t time[3] __attribute__ ((section (".noinit")));
volatile int8_t vent_seg_cur; // текущая установка вентилятор (в сегментах)
volatile int8_t temp_seg_cur; // текущая установка заслонка отопителя (в сегментах)
volatile int8_t vent_enc;
volatile int8_t temp_enc;
volatile float Ipid;
volatile float last_delta;
volatile int8_t vent_add_prc; // Вентилятор установка
volatile uint8_t temp_out_prc;
volatile uint8_t temp_out_delay;
volatile uint8_t bootUp_delay; // задержка при включении для стабилизации состояния датчиков (после обдува)

//volatile uint8_t upHeadFlag;
//volatile uint8_t downHeadFlag;
volatile int8_t cond_needs_of_cool;

enum {
    f_e1sec = 0, //каждую сек
    f_mode_hot, //нагреваем или охлаждаем
    f_mode_frost_cicle_on, //расчет кондиционер. on - off
    f_num_dot,
    f_cond_btn_press, //нажатие на кнопку кондиционера
    f_cond_ind_flash, //индикация работы кондиционера

//    tf_dps_upd,
//    tf_dps_en,
};
volatile uint8_t flags;

#define SET_FLAG(fl) SET_BIT(flags, fl)
#define CLR_FLAG(fl) CLR_BIT(flags, fl)
#define TST_FLAG(fl) TST_BIT(flags, fl)

volatile float Ti = 0;	//Т испаритель или наружняя
volatile float Ts = 0;	//Т салон

/*
 const int8_t str_nasos[] PROGMEM = "Нас";
 */

enum {
    smode_manual = 1,
    smode_auto_defrost,
    smode_cnd_frost,
    smode_seg_delay,
    smode_vent_kfc,
    smode_pid_P_kfc,
    smode_pid_I_kfc,
    smode_load_def,
    smode_disp_T_isp,
    smode_disp_T_salon,
    smode_SETUP_LAST,

    smode_hi_mode = 100, // Инит режима оттаивания
    smode_hi_mode_work, // Режим оттаивания.
};

typedef enum {
    wmode_auto,
    wmode_manual,
    wmode_END_OF_LIST,
} wmode_t;

typedef struct {
    wmode_t wmode;
    int8_t setup_temp;
    int8_t vent_nom_seg;
    int8_t temp_seg;	//for manual mode

    int8_t pid_P_kfc;	//P кфц
    int8_t pid_I_kfc;	//I кфц
    int8_t vent_var_kfc;	//кфц отклонения вентилятора на изменения Т
    int8_t temp_seg_delay;	// пропуск изменений заслонки
    int8_t const_cnd_frost_t; //температура обмерзания кондиционера
    int8_t cond_enabled; // включенный кондей
    int8_t auto_defrost; // авто вкючение разморозки лобового и задание температуры срабатывания 0: off, 1:(-2), ..., 3:0, ..., 5:2
} vars_t;

volatile vars_t vars;
#define EEMEM_TST 50
#define EEMEM_VARS 100
/*
 vars_t EEMEM vars_ee;
 uint8_t EEMEM ftst_ee;
 */
//volatile uint8_t disp_fl;
