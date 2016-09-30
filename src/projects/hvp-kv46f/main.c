/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/******************************************************************************
| local variables
|----------------------------------------------------------------------------*/
double volt_cmd = 0;
double fre_cmd = 0;
int16_t speed = 0;
int16_t FTM1cnt = 0;

int period_count = 0;  // 载波周期数
int Tinv[3] = {0, 0, 0};  // 三相对应比较值
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

double temp = 0;

/******************************************************************************
@brief   Main

@param   N/A

@return  N/A
******************************************************************************/
 void main(void)
{
    /* disable all interrupts before peripherals are initialized */
    __disable_irq();
    
    /* init application ports */  
    InitPorts();  
    PORT_WR_PCR_MUX(PORTB, 22, 1);
    GPIO_SET_PDDR(PTB, 1<<22);

    /* initialize peripheral motor control driver for motor M1 */
    Init_PWMA(); 
    Init_FTM0();
    Init_FTM1();
    Init_FTM3();
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

/******************************************************************************
@brief   PWMA 中断 -- 电流采样及开环SVM

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RELOAD0_IRQHandler(void)
{   
    /* 读取电流采样 */
    IU = ADC_RD_RSLT_RSLT(ADC, 1);
    IW = ADC_RD_RSLT_RSLT(ADC, 2);
      // 存储最值
    if (IU > maxIU)
      maxIU = IU;
    else if (IU < minIU)
      minIU = IU;
    if (IW > maxIW)
      maxIW = IW;
    else if (IW < minIW)
      minIU = IW;
  
    /* V/f曲线计算电压给定值 */
    volt_cmd = RAMP(VFramp, 0, fre_cmd, Voltlimit_H, Voltlimit_L);
   
    /* SVM */
      // 扇区及夹角计算
    Angle += 2 * pi * fre_cmd * 0.0001;
    if (Angle > 2 * pi)      Angle -= 2*pi;    
    theta = fmod(Angle,1/3.0 * pi);
    sector = (int)floor( Angle / (1/3.0 * pi)) + 1;
      // 占空比计算
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
    if (period_count > 60000)  // 6s存储一次电流最小值
    {
      period_count = 0;
      minIUarray[kmin] = minIU;
      minIWarray[kmin] = minIW;
      minIU = IU;
      minIW = IW;
      kmin++;
    }
    
    PWM_WR_STS_RF(PWMA, 0, TRUE);
    /* start PWMs (set load OK flags and run) */
    PWM_WR_MCTRL_LDOK(PWMA, TRUE);
    
    /* PWM DA */
    FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * sin(Angle)));
    FTM_WR_SYNC_SWSYNC(FTM3, TRUE);
 }

/******************************************************************************
@brief   PWMA 错误中断

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}

/******************************************************************************
@brief   PIT 中断 -- 计算转速及频率给定值

@param   N/A

@return  N/A
******************************************************************************/
void PIT0_IRQHandler(void)
{
  PIT_WR_TFLG_TIF(PIT, 0, 1);
  GPIO_WR_PTOR(PTB, 1<<22);
  
  /* 转速计算 M法 */
  if (FTM_RD_SC_TOF(FTM1) == 0)  // 计数值未溢出
  {
    speed = (int)((FTM_RD_CNT(FTM1) - FTM1cnt) * 2.34375);
  }
  else  // 溢出
  {
    speed = (int)((FTM_RD_CNT(FTM1) + FTM1_MODULO - FTM1cnt) * 2.34375);
    FTM_WR_SC_TOF(FTM1, 0);
  }
  
  FTM1cnt = FTM_RD_CNT(FTM1);
  
  /* 频率斜坡函数 */
  if (fre_cmd < fre_req)
  {
    fre_cmd = RAMP(Freramp, fre_cmd, 0.1, Frelimit_H, Frelimit_L);
  }
}

/******************************************************************************
@brief   PWMA 初始化

@param   N/A

@return  N/A
******************************************************************************/
void Init_PWMA(void)
{
    /* enable clock for eFlexPWM modules 0,1 and 2 in SIM module */
    SIM_WR_SCGC4_eFlexPWM0(SIM, TRUE);
    SIM_WR_SCGC4_eFlexPWM1(SIM, TRUE);
    SIM_WR_SCGC4_eFlexPWM2(SIM, TRUE);

    /* full cycle reload */
    PWM_WR_CTRL_FULL(PWMA, 0, TRUE);
    PWM_WR_CTRL_FULL(PWMA, 1, TRUE);
    PWM_WR_CTRL_FULL(PWMA, 2, TRUE);
    
    /* value register initial values, duty cycle 50% */
    PWM_WR_INIT(PWMA, 0, (uint16_t)(-(M1_PWM_MODULO/2)));
    PWM_WR_INIT(PWMA, 1, (uint16_t)(-(M1_PWM_MODULO/2)));
    PWM_WR_INIT(PWMA, 2, (uint16_t)(-(M1_PWM_MODULO/2)));
    PWM_WR_INIT(PWMA, 3, (uint16_t)(-(M1_PWM_MODULO/2)));
    
    PWM_WR_VAL0(PWMA, 0, (uint16_t)(0));
    PWM_WR_VAL0(PWMA, 1, (uint16_t)(0));
    PWM_WR_VAL0(PWMA, 2, (uint16_t)(0));
    PWM_WR_VAL0(PWMA, 3, (uint16_t)(0));
    
    PWM_WR_VAL1(PWMA, 0, (uint16_t)((M1_PWM_MODULO/2)-1));
    PWM_WR_VAL1(PWMA, 1, (uint16_t)((M1_PWM_MODULO/2)-1));
    PWM_WR_VAL1(PWMA, 2, (uint16_t)((M1_PWM_MODULO/2)-1));
    PWM_WR_VAL1(PWMA, 3, (uint16_t)((M1_PWM_MODULO/2)-1));
    
    PWM_WR_VAL2(PWMA, 0, (uint16_t)(-(M1_PWM_MODULO/4)));
    PWM_WR_VAL2(PWMA, 1, (uint16_t)(-(M1_PWM_MODULO/4)));
    PWM_WR_VAL2(PWMA, 2, (uint16_t)(-(M1_PWM_MODULO/4)));
    PWM_WR_VAL2(PWMA, 3, (uint16_t)(-(M1_PWM_MODULO/4)));
    
    PWM_WR_VAL3(PWMA, 0, (uint16_t)((M1_PWM_MODULO/4)));
    PWM_WR_VAL3(PWMA, 1, (uint16_t)((M1_PWM_MODULO/4)));
    PWM_WR_VAL3(PWMA, 2, (uint16_t)((M1_PWM_MODULO/4)));
    PWM_WR_VAL3(PWMA, 3, (uint16_t)((M1_PWM_MODULO/4)));
    
    PWM_WR_VAL4(PWMA, 0, (uint16_t)(0)); /* ADCA trigger */
    PWM_WR_VAL4(PWMA, 1, (uint16_t)(0));
    PWM_WR_VAL4(PWMA, 2, (uint16_t)(0));
    PWM_WR_VAL4(PWMA, 3, (uint16_t)(0));
    
    PWM_WR_VAL5(PWMA, 0, (uint16_t)(0)); 
    PWM_WR_VAL5(PWMA, 1, (uint16_t)(0));
    PWM_WR_VAL5(PWMA, 2, (uint16_t)(0));
    PWM_WR_VAL5(PWMA, 3, (uint16_t)(0));
    
    /* PWMA module 0 trigger on VAL4 enabled for ADC synchronization */
    //PWM_WR_TCTRL_OUT_TRIG_EN(PWMA, 0, (1<<4));

    /* recomended value of deadtime for FNB41560 on HVP-MC3PH is 1.5us
       DTCNT0,1 = T_dead * f_fpc = 1.5us * 74MHz = 111 */
    PWM_WR_DTCNT0(PWMA, 0, 111);
    PWM_WR_DTCNT0(PWMA, 1, 111);
    PWM_WR_DTCNT0(PWMA, 2, 111);
    PWM_WR_DTCNT0(PWMA, 3, 111);
    PWM_WR_DTCNT1(PWMA, 0, 111);
    PWM_WR_DTCNT1(PWMA, 1, 111);
    PWM_WR_DTCNT1(PWMA, 2, 111);
    PWM_WR_DTCNT1(PWMA, 3, 111);
      
    /* channels A and B disabled when fault 0 occurs */
    PWM_WR_DISMAP_DIS0A(PWMA, 0, 0, 0x0);
    PWM_WR_DISMAP_DIS0A(PWMA, 1, 0, 0x0);
    PWM_WR_DISMAP_DIS0A(PWMA, 2, 0, 0x0); 
    PWM_WR_DISMAP_DIS0B(PWMA, 0, 0, 0x0);
    PWM_WR_DISMAP_DIS0B(PWMA, 1, 0, 0x0);
    PWM_WR_DISMAP_DIS0B(PWMA, 2, 0, 0x0); 

    /* modules one and two gets clock from module zero */
    PWM_WR_CTRL2_CLK_SEL(PWMA, 1, 0x2);
    PWM_WR_CTRL2_CLK_SEL(PWMA, 2, 0x2);
    
    /* master reload active for modules one and two*/
    PWM_WR_CTRL2_RELOAD_SEL(PWMA, 1, TRUE);
    PWM_WR_CTRL2_RELOAD_SEL(PWMA, 2, TRUE);
    
    /* master sync active for modules one and two*/
    PWM_WR_CTRL2_INIT_SEL(PWMA, 1, 0x2);
    PWM_WR_CTRL2_INIT_SEL(PWMA, 2, 0x2);
    
    /* fault 0 active in high, fault 1 active in low, manual clearing */
    PWM_WR_FCTRL_FLVL(PWMA, 0x1);
    PWM_WR_FCTRL_FAUTO(PWMA, 0x0);

    /* PWMs are re-enabled at PWM full cycle */
    PWM_WR_FSTS_FFULL(PWMA, 0x1); 
     
    /* PWM fault filter - 5 Fast periph. clocks sample rate, 5 agreeing 
       samples to activate */
    PWM_WR_FFILT_FILT_PER(PWMA, 5);
    PWM_WR_FFILT_FILT_CNT(PWMA, 5);
        
    /* enable A&B PWM outputs for submodules one, two and three */
    PWM_WR_OUTEN_PWMA_EN(PWMA, 0x7);
    PWM_WR_OUTEN_PWMB_EN(PWMA, 0x7);
    
    PWM_WR_CTRL_PRSC(PWMA, 0, 0);
    //PWM_WR_CTRL_PRSC(PWMA, 1, 3);
    //PWM_WR_CTRL_PRSC(PWMA, 2, 3);
       
    /* start PWMs (set load OK flags and run) */
    PWM_WR_MCTRL_CLDOK(PWMA, 0x7);
    PWM_WR_MCTRL_LDOK(PWMA, 0x7);
    PWM_WR_MCTRL_RUN(PWMA, 0x7);
    
    /* set ports */
    PORT_WR_PCR_MUX(PORTD, 0, 6);                                               /* HVP-MC3PH phase A top */
    PORT_WR_PCR_MUX(PORTD, 1, 6);                                               /* HVP-MC3PH phase A bottom */
    PORT_WR_PCR_MUX(PORTD, 2, 6);                                               /* HVP-MC3PH phase B top */
    PORT_WR_PCR_MUX(PORTD, 3, 6);                                               /* HVP-MC3PH phase B bottom */
    PORT_WR_PCR_MUX(PORTD, 4, 5);                                               /* HVP-MC3PH phase C top */
    PORT_WR_PCR_MUX(PORTD, 5, 5);                                               /* HVP-MC3PH phase C bottom */
    
    PWM_WR_INTEN_RIE(PWMA, 0 , TRUE);
    /* enable & setup interrupts */
    NVIC_EnableIRQ(PWMA_RELOAD0_IRQn);                                                  /* enable Interrupt */
    NVIC_SetPriority(PWMA_RELOAD0_IRQn, 3);                                             /* set priority to interrupt */
}

/******************************************************************************
@brief   ADC 初始化

@param   N/A

@return  N/A
******************************************************************************/
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

/******************************************************************************
@brief   FTM1 初始化 -- 计算转速

@param   N/A

@return  N/A
******************************************************************************/
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

/******************************************************************************
@brief   FTM3 初始化 -- PWM DA输出

@param   N/A

@return  N/A
******************************************************************************/
void Init_FTM3(void)
{
  /* enable the clock for FTM3 */
  SIM_WR_SCGC6_FTM3(SIM, TRUE);
  
  PORT_WR_PCR_MUX(PORTC, 11, 3);  // set port
  PORT_WR_PCR_MUX(PORTC, 10, 3);  // set port
  
  /* Disable all channel 0-1 outputs using the OUTPUT MASK feature. */
  FTM_WR_OUTMASK(FTM3, 0xff);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM3, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM3, TRUE);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM3, 0x03);
  
  FTM_WR_COMBINE_SYNCEN3(FTM3, TRUE);
  
  /* set modulo register */
  FTM_WR_MOD(FTM3, (uint16_t)(FTM3_MODULO));
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM3, (uint16_t)(-FTM3_MODULO));  //
        
  /* PWM update at counter in maximal value */
  FTM_WR_SYNC_CNTMAX(FTM3, TRUE);

  FTM_WR_CnSC_ELSA(FTM3, 6, TRUE); 
  FTM_WR_CnSC_MSB(FTM3, 6, TRUE); 
  FTM_WR_CnSC_ELSA(FTM3, 7, TRUE); 
  FTM_WR_CnSC_MSB(FTM3, 7, TRUE); 
  
    /* initial setting of value registers to 0 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM3, 6, (uint16_t)(0));
  FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(0));
  
    /* set LOAD OK register */
  FTM_WR_PWMLOAD_LDOK(FTM3, TRUE);
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM3, TRUE);                                               
    
  /* set system clock as source for FTM3 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM3, 0x01);
  
  FTM_WR_OUTMASK(FTM3, 0x00);  
}

/******************************************************************************
@brief   RAMP -- 增量式斜坡函数

@param   ramp -- 斜率
         initial -- 应变量起始值
         increment -- 自变量增量
         Hlimit -- 上限
         Llimit -- 下限

@return  应变量终值
******************************************************************************/
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

/******************************************************************************
@brief   roundn -- 有理数取指定位数

@param   input -- 输入

@return  舍弃指定位数后的值
******************************************************************************/
double roundn(double input)
{
  double temp;
  temp = input * digit;
  temp = floor(temp);
  temp = temp / digit;
  return temp;
}

void Init_FTM0(void)
{
  /* enable the clock for FTM0 */
  SIM_WR_SCGC6_FTM0(SIM, TRUE);
  
  /* Disable all channel 0-5 outputs using the OUTPUT MASK feature.
  (please note that the output pins are still driven as GPIO since the
  channel mode is set to FTM channel disabled after RESET) */
  FTM_WR_OUTMASK(FTM0, 0x3F);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM0, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM0, TRUE);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM0, 0x03);
       
  /* set modulo register */
  FTM_WR_MOD(FTM0, (uint32_t)((M1_PWM_MODULO / 2) - 1));
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM0, (uint32_t) (-M1_PWM_MODULO / 2));
 
  /* PWM update at counter in maximal value */
  FTM_WR_SYNC_CNTMAX(FTM0, TRUE);
    
  /* set combine mode */
  FTM_WR_COMBINE_COMBINE0(FTM0, TRUE);
  FTM_WR_COMBINE_COMBINE1(FTM0, TRUE);
  FTM_WR_COMBINE_COMBINE2(FTM0, TRUE);
    
  /* set complementary PWM */
  FTM_WR_COMBINE_COMP0(FTM0, TRUE); 
  FTM_WR_COMBINE_COMP1(FTM0, TRUE); 
  FTM_WR_COMBINE_COMP2(FTM0, TRUE); 
    
  /* enable dead time */
  FTM_WR_COMBINE_DTEN0(FTM0, TRUE);
  FTM_WR_COMBINE_DTEN1(FTM0, TRUE);   
  FTM_WR_COMBINE_DTEN2(FTM0, TRUE);   
                  
  /* enable PWM update synchronization */
  FTM_WR_COMBINE_SYNCEN0(FTM0, TRUE); 
  FTM_WR_COMBINE_SYNCEN1(FTM0, TRUE);  
  FTM_WR_COMBINE_SYNCEN2(FTM0, TRUE);                                    
    
  /* recomended value of deadtime for FNB41560 on HVP-MC3PH is 1.5us
  DTPS x DTVAL = T_dead * f_fpc = 1.5us * 74MHz = 111 ~ 112 = 4 x 28 */
  FTM_WR_DEADTIME_DTPS(FTM0, 0x2); 
  FTM_WR_DEADTIME_DTVAL(FTM0, 28);
    
  /* initial setting of value registers to 50 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM0, 0, (uint32_t)(-M1_PWM_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 1, (uint32_t)(M1_PWM_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 2, (uint32_t)(-M1_PWM_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 3, (uint32_t)(M1_PWM_MODULO / 4));    
  FTM_WR_CnV_VAL(FTM0, 4, (uint32_t)(-M1_PWM_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 5, (uint32_t)(M1_PWM_MODULO / 4));    

  /* note:
  1. From this moment the output pins are under FTM control. Since the PWM 
  output is disabled by the FTM0OUTMASK register, there is no change on 
  PWM outputs. Before the channel mode is set, the correct output pin 
  polarity has to be defined.
  2. Even if the odd channels are generated automatically by complementary 
  logic, these channels have to be set to be in the same channel mode. */
  FTM_WR_CnSC_ELSB(FTM0, 0, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 1, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 2, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 3, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 4, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 5, TRUE);
 
  /* set LOAD OK register */
  FTM_WR_PWMLOAD_LDOK(FTM0, TRUE);
  
  /* initialization trigger enable */
  FTM_WR_EXTTRIG_INITTRIGEN(FTM0, TRUE);  // ???????????
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM0, TRUE);                                               
    
  /* set system clock as source for FTM0 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM0, 0x01); 
  
  FTM_WR_OUTMASK(FTM0, 0x00);                                               
}