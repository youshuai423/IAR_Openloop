#include "main.h"
#include "app_init.h"
#include "math.h"

double volt_cmd = 0;
double fre_cmd = 0;
double speed = 0;
double FTM1cnt = 0;

double spdcmd = 300;
//double fre_req = 0;
double spdlasterr = 0;

int period_count = 0;  // 载波周期数
int PIT_count = 0;
int fre_count = 0;
int Tinv[3] = {0, 0, 0};  // 三相对应比较值
int last[3];  // 上周期Tinv值(for test)
double Dm = 0, Dn = 0, D0 = 0;  // 占空比

int sector = 0;
double Angle = 0;
double theta = 0;

void main(void)
{
    /* disable all interrupts before peripherals are initialized */
    __disable_irq();
    
    /* init application ports */  
    InitPorts();  
    PORT_WR_PCR_MUX(PORTB, 22, 1);                                              
    GPIO_SET_PDDR(PTB, 1<<22);

    /* initialize peripheral motor control driver for motor M1 */
    MCDRV_Init_M1();      
    //Init_FTM1();
    Init_PIT();

    /* enable interrupts  */
    __enable_irq();

    /* infinite loop */
    while(1)
    {
    }
}

void PWMA_RELOAD0_IRQHandler(void)
{    
    volt_cmd = RAMP(VFramp, fre_cmd, Voltlimit_H, Voltlimit_L);
    
    //Angle = fmod((2 * pi * fre_cmd * (period_count / 10000.0)), (2 * pi));
    Angle += 2 * pi * fre_cmd * 0.0001;
    if (Angle > 2 * pi)
      Angle -= 2*pi;
    theta = fmod(Angle,1/3.0 * pi);
    sector = floor( Angle / (1/3.0 * pi)) + 1;
    Dm = volt_cmd / Ud * sin(1/3.0 * pi - theta);
    Dn = volt_cmd / Ud * sin(theta);
    D0 = (1 - Dm - Dn) / 2.0;
    Dm = roundn(Dm);
    Dn = roundn(Dn);
    D0 = roundn(D0);
    if (D0 < 0) D0 = 0;
    
    switch (sector)
    {
    case 1:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0 + Dn));
      Tinv[2] = (int)(period * (D0));
      break;
    case 2:
      Tinv[0] = (int)(period * (Dm + D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (D0));
      break;
    case 3:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (Dn + D0));
      break;
    case 4:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 5:
      Tinv[0] = (int)(period * (Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 6:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + D0));
    }
    
    PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
    PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
    PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
    
    PWM_WR_VAL3(PWMA, 0, Tinv[0]);
    PWM_WR_VAL3(PWMA, 1, Tinv[1]);
    PWM_WR_VAL3(PWMA, 2, Tinv[2]);
    
    period_count++;
    if (period_count > 10000) 
    {
      period_count = 0;
    }

    last[0] = Tinv[0];
    last[1] = Tinv[1];
    last[2] = Tinv[2];
    
    PWM_WR_STS_RF(PWMA, 0, TRUE);
    /* start PWMs (set load OK flags and run) */
    PWM_WR_MCTRL_LDOK(PWMA, TRUE);
 }

void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}

void PIT0_IRQHandler(void)
{
  PIT_WR_TFLG_TIF(PIT, 0, 1);
  GPIO_WR_PTOR(PTD, 1<<0);
  GPIO_WR_PTOR(PTB, 1<<22);
  
 /* if (FTM_RD_SC_TOF(FTM1) == 0)
  {
    speed = (FTM_RD_CNT(FTM1) - FTM1cnt) * 2.34375;
  }
  else
  {
    speed = (FTM_RD_CNT(FTM1) + FTM1_MODULO - FTM1cnt) * 2.34375;
    FTM_WR_SC_TOF(FTM1, 0);
  }
  FTM1cnt = FTM_RD_CNT(FTM1);
  */
  if (fre_cmd < fre_req)
  {
    PIT_count ++;
    fre_cmd = RAMP(Freramp, PIT_count * 0.1, Frelimit_H, Frelimit_L);
  }
}

void Init_FTM1(void)
{
  /* enable the clock for FTM1 */
  SIM_WR_SCGC6_FTM1(SIM, TRUE);
  
  /* Disable all channel 0-1 outputs using the OUTPUT MASK feature. */
  FTM_WR_OUTMASK(FTM1, 0x03);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM1, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM1, TRUE);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM1, 0x03);
       
  /* set modulo register */
  FTM_WR_MOD(FTM1, (uint32_t)FTM1_MODULO);
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM1, 0);                                                 
        
  FTM_WR_FILTER_CH0FVAL(FTM1, 0x03);
  FTM_WR_FILTER_CH1FVAL(FTM1, 0x03);
  
  /* initial setting of value registers to 0 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM1, 0, 0);
  FTM_WR_CnV_VAL(FTM1, 1, 0);

  FTM_WR_CnSC_ELSA(FTM1, 0, TRUE); 
  FTM_WR_CnSC_ELSA(FTM1, 1, TRUE); 
  
  FTM_WR_QDCTRL_PHAFLTREN(FTM1, TRUE); 
  FTM_WR_QDCTRL_PHBFLTREN(FTM1, TRUE); 
  FTM_WR_QDCTRL_QUADEN(FTM1, TRUE); 
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM1, TRUE);                                               
    
  /* set system clock as source for FTM1 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM1, 0x01);
        
  /* set ports */
  PORT_WR_PCR_MUX(PORTA, 12, 7);
  PORT_WR_PCR_MUX(PORTA, 13, 7);  
}

double RAMP(double ramp, double paramin, double Hlimit, double Llimit)
{
  double temp = ramp * paramin;
  if (temp > Hlimit)
    return Hlimit;
  else if (temp < Llimit)
    return Llimit;
  else
    return temp;
}

double PImodule(double Kp, double Ki, double inputk, double err, double *lasterr, double Uplim, double Downlim)
{
  //inputk += Kp * (err - *lasterr) + Ki * Ts * err;
  *lasterr = err;
  
  if (inputk >= Downlim && inputk <= Uplim)
    return inputk;
  else if (inputk > Uplim)
    return Uplim;
  else
    return Downlim;
}

double roundn(double input)
{
  double temp;
  temp = input * digit;
  temp = floor(temp);
  temp = temp / digit;
  return temp;
}