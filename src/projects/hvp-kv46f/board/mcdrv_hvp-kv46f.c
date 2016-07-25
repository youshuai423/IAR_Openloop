/*******************************************************************************
*
* Copyright 2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************//*!
*
* @file     mcdrv_hvp-kv46f150m.c
*
* @brief    Motor control peripheral driver main board file
*
* @board    hvp-kv46f150m
*
*******************************************************************************/
/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "mcdrv_hvp-kv46f.h"
#include "fsl_device_registers.h" 
#include "main.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
/* configuration structure for 3-phase PWM MC driver */
M1_MCDRV_PWM3PH_INIT_T     msM1Pwm3phInit;

/* configuration structure for ADC driver - phase currents, DC-bus voltage 
   and aux meaurement */
M1_MCDRV_ADC_INIT_T        msM1AdcInit;

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/
/* structure for 3-phase PWM MC driver */
M1_MCDRV_PWM3PH_T          msM1Pwm3ph;

/* structure for ADC driver - phase currents, DC-bus voltage and aux
   meaurement */
M1_MCDRV_ADC_T             msM1AdcSensor;

/******************************************************************************
| function implementations                                
|----------------------------------------------------------------------------*/
/******************************************************************************
@brief   Motor control driver main initialization
          - Calls initialization functions of peripherals required for motor
            control fucntionality

@param   N/A

@return  N/A
******************************************************************************/
void MCDRV_Init_M1(void)
{
    /* init ADC */
    //M1_MCDRV_ADC_PERIPH_INIT();  
    
    /* init XBARA */
    //InitXBARA();
    
    /* init FTM1 for slow loop counter*/
    //InitFTM1(); 
    
    /* six-channel PWM peripheral init  */
    M1_MCDRV_PWM_PERIPH_INIT(); 
    
    /* comparator CMP1 */
    //InitCMP1();
}


#if (M1_MCDRV_PWM3PH == MCDRV_FTM)
void InitFTM0(void)
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
    
    /* fault control mode - fault control is enabled for all channels, and 
       the selected mode is the manual fault clearing */
    FTM_WR_MODE_FAULTM(FTM0, 0x03); 
    
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
             
    /* enable fault control */
    FTM_WR_COMBINE_FAULTEN0(FTM0, TRUE);                                       
    FTM_WR_COMBINE_FAULTEN1(FTM0, TRUE);                                       
    FTM_WR_COMBINE_FAULTEN2(FTM0, TRUE);                                       
    
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
    FTM_WR_EXTTRIG_INITTRIGEN(FTM0, TRUE);
    
    /* initialize the channels output */
    FTM_WR_MODE_INIT(FTM0, TRUE);                                               
    
    /* set system clock as source for FTM0 (CLKS[1:0] = 01) */
    FTM_WR_SC_CLKS(FTM0, 0x01);
    
    /* set fault fitler to 15 agreeing consecutive samples */
    FTM_WR_FLTCTRL_FFVAL(FTM0, 0xF); 
    
    /* enable fault 3 - FNB41560 10.5A over-current flag */    
    FTM_WR_FLTCTRL_FAULT3EN(FTM0, TRUE); 
    
    /* fault 3 polarity setting (active low) */
    FTM_WR_FLTPOL_FLT3POL(FTM0, TRUE);
    
    /* fault 3 is connected to XBARA_49 */
    SIM_WR_SOPT4_FTM0FLT3(SIM, TRUE);
    
    /* enable fault 1 - adjustable over-current flag from comparator 1 */    
    FTM_WR_FLTCTRL_FAULT1EN(FTM0, TRUE); 
    
    /* fault 1 is connected to CMP1_OUT */
    SIM_WR_SOPT4_FTM0FLT1(SIM, TRUE);
    
    /* fault 1 polarity setting (active high) */
    FTM_WR_FLTPOL_FLT1POL(FTM0, FALSE);
        
    /* set ports */
    PORT_WR_PCR_MUX(PORTD, 0, 5);                                               /* HVP-MC3PH phase A top */
    PORT_WR_PCR_MUX(PORTD, 1, 5);                                               /* HVP-MC3PH phase A bottom */
    PORT_WR_PCR_MUX(PORTD, 2, 5);                                               /* HVP-MC3PH phase B top */
    PORT_WR_PCR_MUX(PORTD, 3, 5);                                               /* HVP-MC3PH phase B bottom */
    PORT_WR_PCR_MUX(PORTD, 4, 4);                                               /* HVP-MC3PH phase C top */
    PORT_WR_PCR_MUX(PORTD, 5, 4);                                               /* HVP-MC3PH phase C bottom */
    
    /* initialization FTM 3-phase PWM mc driver */
    msM1Pwm3phInit.pui32PwmBase       = (FTM_Type*)(FTM0);                      /* FTM0 base address */
    msM1Pwm3phInit.ui16ChanPairNumPhA = (M1_PWM_PAIR_PHA << 1);                 /* PWM phase A top & bottom channel pair number */
    msM1Pwm3phInit.ui16ChanPairNumPhB = (M1_PWM_PAIR_PHB << 1);                 /* PWM phase B top & bottom channel pair number */
    msM1Pwm3phInit.ui16ChanPairNumPhC = (M1_PWM_PAIR_PHC << 1);                 /* PWM phase C top & bottom channel pair number */
    msM1Pwm3phInit.ui16FaultFixNum = (1<<3);                                    /* FTM fault number for FNB41560 10.5A over-current fault detection */
    msM1Pwm3phInit.ui16FaultAdjNum = (1<<1);                                    /* FTM fault number for adjustable comparator over-current fault detection */
    
    /* pass initialization structure to the mc driver */
    MCDRV_FtmPwm3PhInit(&msM1Pwm3ph, &msM1Pwm3phInit);                          /* MC driver initialization */
    
    /* init all MC driver pointers*/
    msM1Pwm3ph.psUABC = &gsM1Drive.sFocACIM.sDutyABC;

}
#endif /* M1_MCDRV_PWM3PH == MCDRV_FTM */


#if (M1_MCDRV_PWM3PH == MCDRV_PWMA)
void InitPWMA(void)
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
#endif /* M1_MCDRV_PWM3PH == MCDRV_PWMA */

/******************************************************************************
@brief   Initialization of the FTM1 peripheral acts as slow control loop timer 

@param   N/A

@return  N/A
******************************************************************************/
void InitFTM1(void)
{
    /* enable clock to FTM1 module */
    SIM_WR_SCGC6_FTM1(SIM, TRUE);
  
    /* configuration of FTM1 module */
    FTM_WR_MODE_WPDIS(FTM1, TRUE);                                              /* disable write protection for certain registers */
    FTM_WR_MODE_FTMEN(FTM1, TRUE);                                              /* enable the counter */
    FTM_WR_CONF_BDMMODE(FTM1, 0x03);                                            /* counter running in BDM mode */
  
    FTM_WR_CNTIN_INIT(FTM1, 0);                                                 /* set count value to 0 */
    FTM_WR_MOD_MOD(FTM1, (M1_PWM_MODULO / 
        (M1_SPEED_LOOP_FREQ * M1_SAMPLE_TIME) / 16));                           /* 10x slower than fast loop */
     
    FTM_WR_PWMLOAD_LDOK(FTM1, TRUE);                                            /* LOADOK */
    
    /* settings up FTM SC register for clock setup */
    FTM_WR_SC_CLKS(FTM1, 0x01);                                                 /* set system clock as source for FTM0 (CLKS[1:0] = 01) */
    FTM_WR_SC_PS(FTM1, 0x04);                                                   /* set prescaler to 16 */
    FTM_WR_SC_TOIE(FTM1, TRUE);                                                 /* enable interrupt from FTM1 */
  
    /* enable & setup interrupts */
    NVIC_EnableIRQ(FTM1_IRQn);                                                  /* enable Interrupt */
    NVIC_SetPriority(FTM1_IRQn, 3);                                             /* set priority to interrupt */
}

/******************************************************************************
@brief   Initialization of the cyclic 12-bit ADC peripheral for current and 
         voltage sensing

@param   N/A

@return  N/A
******************************************************************************/
void InitADC12(void)
{
    /* ADC channel number assignment array to be passeed to MC ADC driver */
    static uint16_t ui16AdcArray[10] =  {M1_ADC0_PH_A, M1_ADC1_PH_A,            /* phase current A */
                                         M1_ADC0_PH_B, M1_ADC1_PH_B,            /* phase current B */
                                         M1_ADC0_PH_C, M1_ADC1_PH_C,            /* phase current C */
                                         ADC0_UDCB, ADC1_UDCB,                  /* DC-bus voltage */
                                         ADC0_AUX,  ADC1_AUX};                  /* auxiliary channel */
                                                                                
    /* enable clock for ADC modules */
    SIM_WR_SCGC5_ADC(SIM, TRUE);
  
    /* triggered parallel mode */
    ADC_WR_CTRL1_SMODE(ADC, 0x5);
    
    /* enable end-of-scan interrupt */
    ADC_WR_CTRL1_EOSIE0(ADC, TRUE);

    /* enable hwardware triggering */
    ADC_WR_CTRL1_SYNC0(ADC, TRUE);
    
    /* start ADC */
    ADC_WR_CTRL1_STOP0(ADC, FALSE);
    
    /* input clock is 24.66MHz (148MHz fast peripheral clock divided by 6), 
       single ended */
    ADC_WR_CTRL2_DIV0(ADC, 0x005);
    
    /* disable ADC */
    ADC_WR_CTRL2_STOP1(ADC, TRUE);

    /* enable samples first two samples on both ADCA and ADCB */
    ADC_WR_SDIS(ADC, 0xFCFC);
        
    /* power-up ADCA and ADCB */
    ADC_WR_PWR_PD0(ADC, FALSE);
    ADC_WR_PWR_PD1(ADC, FALSE);
        
    /* pass initialization structure to ADC MC driver */
    msM1AdcInit.ui16AdcArray = (&ui16AdcArray[0]);
    msM1AdcInit.pui32AdcBase = (ADC_Type *)ADC;
    
    MCDRV_Adc12Init(&msM1AdcSensor, &msM1AdcInit); 
    
    /* assign channels and init all pointers */
    MCDRV_Curr3Ph2ShChanAssign(&msM1AdcSensor);
    msM1AdcSensor.pf16UDcBus     = &gsM1Drive.sFocACIM.f16UDcBusMeas;
    msM1AdcSensor.psIABC         = &gsM1Drive.sFocACIM.sIABCMeas;
    msM1AdcSensor.pui16SVMSector = &gsM1Drive.sFocACIM.ui16SectorSVM;
    msM1AdcSensor.pui16AuxChan   = &gsM1Drive.f16AdcAuxSample;
    
    /* enable & setup interrupt from ADC */
    NVIC_EnableIRQ(ADCA_IRQn);                                                  /* enable Interrupt */
    NVIC_SetPriority(ADCA_IRQn, 2);                                             /* set priority to interrupt */
} 

/******************************************************************************
@brief   void InitXBARA(void)
          - PWMA0 trigger 0 connected to ADC0 trigger
          - pin 79 (overcurrent) connected to PWMA FAULT0

@param   N/A

@return  N/A
******************************************************************************/
void InitXBARA(void)
{
    /* Enable clock for XBARA */
    SIM_WR_SCGC5_XBARA(SIM, TRUE);
 
#if (M1_MCDRV_PWM3PH == MCDRV_FTM)
    
    /* connect FTM0_INIT trigger to ADC */
    XBARA_WR_SEL6_SEL12(XBARA, 17);
    
    /* connect XBARA_IN7 to FTM0_FLT3 (fixed over-current fault from 
       PTE1/ALT5) */
    PORT_WR_PCR_MUX(PORTE, 1, 5);
    XBARA_WR_SEL24_SEL49(XBARA, 7);

#elif (M1_MCDRV_PWM3PH == MCDRV_PWMA)
    
    /* connect PWMA0_TRG0 trigger to ADC */
    XBARA_WR_SEL6_SEL12(XBARA, 20);
    
    /* connect CMP1_OUT to PWMA_FAULT0 (adjustable over-current fault) */
    XBARA_WR_SEL14_SEL29(XBARA, 13);
    
    /* connect XBARA_IN7 to PWMA_FAULT1 (fixed over-current fault) */
    PORT_WR_PCR_MUX(PORTE, 1, 5);
    XBARA_WR_SEL15_SEL30(XBARA, 7);
    
#endif
    
}

/******************************************************************************
@brief   Initialization of the comparator 1 module for dc-bus over current 
         detection 

@param   N/A

@return  N/A
******************************************************************************/
void InitCMP1(void)
{
    /* enable clock for CMP module */
    SIM_WR_SCGC4_CMP(SIM, TRUE);
    
    /* filter - 4 consecutive samples must agree */
    CMP_WR_CR0_FILTER_CNT(CMP1, 4);     
    
    /* DAC output set value according to desired OC threshold level */
    CMP_WR_DACCR_VOSEL(CMP1, OVER_CURRENT_THRESHOLD_INT);   
    
    /* reference voltage will be VDD */
    CMP_WR_DACCR_VRSEL(CMP1, TRUE); 
    
    /* enable DAC */
    CMP_WR_DACCR_DACEN(CMP1, TRUE);
    
     /* plus is CMP1_IN5 ~ overcurrent pin */
    CMP_WR_MUXCR_PSEL(CMP1, 5);
    
    /* minus is CMP1_IN7 ~ 6bit reference */
    CMP_WR_MUXCR_MSEL(CMP1, 7);                                                 
    
    /* enable analog comparator */
    CMP_WR_CR1_EN(CMP1, TRUE);
    
}


/*
 *######################################################################
 *                           End of File
 *######################################################################
*/ 