#include "main.h"
#include "app_init.h"
#include "math.h"

double volt_cmd = 0;
double fre_cmd = 0;
int16_t speed = 0;
double FTM1cnt = 0;

int period_count = 0;  // 载波周期数
int PIT_count = 0;
int fre_count = 0;
int Tinv[3] = {0, 0, 0};  // 三相对应比较值
int last[3];  // 上周期Tinv值(for test)
double Dm = 0, Dn = 0, D0 = 0;  // 占空比

int sector = 0;
double Angle = 0;
double theta = 0;

uint16_t IU = 0;
uint16_t IW = 0;
uint16_t maxIU = 0;
uint16_t minIU = 0;
uint16_t maxIW = 0;
uint16_t minIW = 0;
uint16_t minIUarray[100];
uint16_t minIWarray[100];
int kmin = 0;

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
    Init_FTM1();
    Init_PIT();
    Init_ADC();
    
    int i = 0;
    for(i = 0; i < 1000; i++){};  // 等待ADC模块稳定
    ADC_WR_CTRL1_START0(ADC, 1); 
    
    /* enable interrupts  */
    __enable_irq();

    /* infinite loop */
    while(1)
    {
    }
}

void PWMA_RELOAD0_IRQHandler(void)
{   
  IU = ADC_RD_RSLT_RSLT(ADC, 1);
  IW = ADC_RD_RSLT_RSLT(ADC, 2);
  
  if (IU > maxIU)
    maxIU = IU;
  else if (IU < minIU)
    minIU = IU;        
  if (IW > maxIW)
    maxIW = IW;
  else if (IW < minIW)
    minIU = IW;
  
    volt_cmd = RAMP(VFramp, 0, fre_cmd, Voltlimit_H, Voltlimit_L);
   
    Angle += 2 * pi * fre_cmd * 0.0001;
    if (Angle > 2 * pi)
      Angle -= 2*pi;
    theta = fmod(Angle,1/3.0 * pi);
    sector = (int)floor( Angle / (1/3.0 * pi)) + 1;
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
    if (period_count > 60000) 
    {
      period_count = 0;
      minIUarray[kmin] = minIU;
      minIWarray[kmin] = minIW;
      minIU = IU;
      minIW = IW;
      kmin++;
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
  
  if (FTM_RD_SC_TOF(FTM1) == 0)
  {
    speed = (int)((FTM_RD_CNT(FTM1) - FTM1cnt) * 2.34375);
  }
  else
  {
    speed = (int)((FTM_RD_CNT(FTM1) + FTM1_MODULO - FTM1cnt) * 2.34375);
    FTM_WR_SC_TOF(FTM1, 0);
  }
  FTM1cnt = FTM_RD_CNT(FTM1);
  
  if (fre_cmd < fre_req)
  {
    PIT_count ++;
    fre_cmd = RAMP(Freramp, fre_cmd, 0.1, Frelimit_H, Frelimit_L);
  }
}

void Init_ADC(void)
{
  /* enable clock for ADC modules */
  SIM_WR_SCGC5_ADC(SIM, 1);
  
  /* loop parallel mode */
  ADC_WR_CTRL1_SMODE(ADC, 0x3);
  
  /* enable end-of-scan interrupt */
  //ADC_WR_CTRL1_EOSIE0(ADC, 1);
  
  /* enable hwardware triggering */
  //ADC_WR_CTRL1_SYNC0(ADC, 1);
                      
  /* start ADCA */
  ADC_WR_CTRL1_STOP0(ADC, 0);
  ADC_WR_CTRL2_STOP1(ADC, 1);
    
  /* input clock is 24.66MHz (148MHz fast peripheral clock divided by 6), 
     single ended */
  ADC_WR_CTRL2_DIV0(ADC, 0x005);
  
  /* parallel scans done independently */
  ADC_WR_CTRL2_SIMULT(ADC, 0);
  
  ADC_WR_CLIST1_SAMPLE0(ADC, 0);
  ADC_WR_CLIST1_SAMPLE1(ADC, 6);
  ADC_WR_CLIST1_SAMPLE2(ADC, 7);

  /* enable samples first two samples on both ADCA and ADCB */
  //ADC_WR_SDIS(ADC, 0xFCFC);
  ADC_WR_SDIS(ADC, 0xFFF8);
        
  /* power-up ADCA and ADCB */
  ADC_WR_PWR_PD0(ADC, 0);
  //ADC_WR_PWR_PD1(ADC, 0);
 
  //ADC_WR_GC1_GAIN0(ADC, 2);
  //ADC_WR_GC1_GAIN1(ADC, 2);
  //ADC_WR_GC1_GAIN2(ADC, 2);
  
  ADC_WR_PWR2_SPEEDA(ADC, 3);
  
  /* enable & setup interrupt from ADC */
  // NVIC_EnableIRQ(ADCA_IRQn);                                                  /* enable Interrupt */
  // NVIC_SetPriority(ADCA_IRQn, 4);                                             /* set priority to interrupt */
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

double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit)
{
  double temp = ramp * increment + initial;
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