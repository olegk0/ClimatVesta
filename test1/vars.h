//#include <avr/pgmspace.h>

#define MAX_RXUDATA	8
#define MIN_RXUDATA	4

//********************************
#define VENT_SEG	8
#define TEMP_SEG	15

//********************************TermoRes**************************
#define ADC_MCNT 		6
#define TR_NTC_T0_GRAD_K	293.15
#define TR_NTC_T0_RES_Kom 	14.0
#define TR_NTC_KFC		2800.0

//**************************Bright Disp******************************
#define DISP_BRG_MIN	1
#define DISP_BRG_MAX	7

//********************************
#define SETUP_TMO_S	5
#define HI_HOT_TMO_S	240

#define VENT_VAR_KFC		3
#define VENT_TO_HOT_SW_SEG	3
#define COND_WORK_DELTA_T	5
#define COND_T_FROST		5
#define COND_OVERCOOL_DELTA_T	0
//********************************
/*
#define PID_P_KFC	30
#define PID_I_KFC	10
#define PID_D_KFC	10
*/
#define PID_P_KFC	5
#define PID_I_KFC	2
#define PID_D_KFC	10

#define TEMP_OUT_DELAY	2
//#define VENT_ADD_CTRL_CFC	30

//volatile uint8_t time[3] __attribute__ ((section (".noinit")));
volatile int8_t vent_seg_cur;
volatile int8_t temp_seg_cur;
volatile int8_t vent_enc;
volatile int8_t temp_enc;
volatile float Ipid;
volatile float last_delta;
volatile int8_t vent_add_prc;
volatile uint8_t temp_out_prc;
volatile uint8_t temp_out_delay;

enum {
    f_e1sec = 0, //каждую сек
    f_mode_hot, //нагреваем или охлаждаем
    f_mode_frost_cicle_on, //on - off
    f_num_dot,
    f_cond_btn_press, //нажатие на кнопку кондиционера
    f_cond_ind_flash,
//    tf_dps_upd,
//    tf_dps_en,
};
volatile uint8_t flags;

#define SET_FLAG(fl) SET_BIT(flags, fl)
#define CLR_FLAG(fl) CLR_BIT(flags, fl)
#define TST_FLAG(fl) TST_BIT(flags, fl)

volatile float Ti = 0;	//Т испаритель
volatile float Ts = 0;	//Т салон

/*
 const int8_t str_nasos[] PROGMEM = "Нас";
 */

enum {
    smode_manual = 1,
    smode_auto_defrost,
    smode_load_def,
    smode_cnd_frost,
    smode_seg_delay,
    smode_vent_kfc,
    smode_pid_P_kfc,
    smode_disp_T_isp,
    smode_disp_T_salon,
    smode_SETUP_LAST,

    smode_hi_mode = 100,
    smode_hi_mode_work,
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

    int8_t pid_P_kfc;	//П кфц
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
