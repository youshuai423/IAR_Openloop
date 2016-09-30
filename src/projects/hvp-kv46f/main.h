/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "mcdrv_hvp-kv46f.h"
#include "app_init.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
#define pi 3.1415926535898
#define digit 100000
#define period 3700  // 半周期时钟数

#define VFramp 0.86  // V/f曲线斜率
#define Freramp 1  // 频率增长斜率
#define fre_req 15  // 频率给定值
#define Ud 40  // 直流电压
#define Frelimit_H 15  // 频率斜坡上限
#define Frelimit_L 0  // 频率斜坡下限
#define Voltlimit_H 40  // 电压给定上限
#define Voltlimit_L 5  // 电压给定下限

#define FTM0_MODULO 3700  // FTM1计数上限
#define FTM1_MODULO 60000  // FTM1计数上限
#define FTM3_MODULO 3700  // FTM3计数上限

/******************************************************************************
| variables
|----------------------------------------------------------------------------*/

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
void Init_PWMA();
void Init_FTM0();
void Init_FTM1();
void Init_FTM3();
void Init_ADC();
double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit);
double roundn(double);  // 截断小数点后位数

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/