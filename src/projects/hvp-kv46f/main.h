/*******************************************************************************
*
* Copyright 2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
*
*******************************************************************************
*
* @file     main.h
*
* @brief    ACIM sensorless on KV46 header file
*
* @board    hvp-kv46f150m
*
*******************************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_

/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "mcdrv_hvp-kv46f.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
#define pi 3.1415926535898
#define digit 100000
#define period 3700  // 半周期时钟数
#define M 0.95 // 调制度
#define VFramp 0.86
#define Freramp 1
#define fre_req 10
#define Ud 40
#define Frelimit_H 10
#define Frelimit_L 0
#define Voltlimit_H 40
#define Voltlimit_L 5
#define FTM1_MODULO 60000

#define frereqKp 1
#define frereqKi 0
#define frereqUplim 15
#define frereqDownlim 0

/******************************************************************************
| types
|----------------------------------------------------------------------------*/

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double RAMP(double ramp, double paramin, double Hlimit, double Llimit);
double roundn(double);  // 截断小数点后位数
void Init_FTM1(void);
double PImodule(double Kp, double Ki, double inputk, double err, double *lasterr, double Uplim, double Downlim);

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
  

#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/