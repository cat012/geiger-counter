// main.c
// 15-Jul-2020



#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "lcd.h"
#include "button.h"


char strbuff[64]; //string buffer

volatile uint16_t pulsecnt=0;  //counter


#define MAX_HVOLT  4150
#define MIN_HVOLT  3850

#define MAX_PWM    30

#define GEIGER_TIME  36

uint16_t pulsebuff[GEIGER_TIME+1];  //pulse counter //[0]-current

uint32_t doserate=0;  //dose rate, uR/h
uint32_t maxrate=0;
uint32_t dosetot=0;

//volatile uint32_t debugcnt=0;

volatile uint8_t syscnt=0;
volatile uint8_t scrcnt=0;


//=============================================================================
void __interrupt isr_high(void)
    {
    if(INT0IF && INT0IE) // external interrupt 0 - pulse from Geiger detector
        {
        INT0IF=0;

        pulsecnt++;
        }

    if(TMR0IF && TMR0IE)  //timer0 overflow - 100Hz ticks
        {
        TMR0IF=0;
        TMR0=TMR0_OVF_PRELOAD;

        button_cont();

        if(scrcnt) scrcnt--;
        if(syscnt) syscnt--;
        }

    if(TMR1IF && TMR1IE)  //timer1 overflow
        {
        TMR1IF=0;
        TMR1=TMR1_OVF_PRELOAD;

        //debugcnt++;
        }

    if(TMR3IF && TMR3IE)  //-----------  timer3  -----------
        {
        TMR3IF=0;
        TMR3=TMR3_OVF_PRELOAD;
        }
    }


//-----------------------------------------------------------------------------
void delay_ms(volatile uint16_t val)
    {
    while(val-->0) __delay_us(1000);
    }


//-----------------------------------------------------------------------------
static uint16_t get_adc(uint8_t channel, uint8_t samples)
    {
    ADCON0=((channel<<2)&0b00111100);

    ADON=1;

    uint16_t tempdata=0;

    for(uint8_t n=0; n<samples; n++)
        {
        GODONE=1;
        while(GODONE);
        if(n) tempdata+=ADRES;
        }

    tempdata /= samples-1;

    return tempdata;
    }


//-----------------------------------------------------------------------------
static uint16_t pwm_read(void)
    {
    uint16_t width=0;
    width=((CCPR2L<<2)|(DC2B1<<1)|(DC2B0));
    return width;
    }


//-----------------------------------------------------------------------------
static void pwm_set(uint16_t width)
    {
    DC2B0=width&0b0000000000000001;
    DC2B1=(width&0b0000000000000010)>>1;
    CCPR2L=(width&0b0000001111111100)>>2;
    }


//-----------------------------------------------------------------------------
static void pwm_incr(void)
    {
    uint16_t tmp = pwm_read();
    if(++tmp>MAX_PWM) tmp=MAX_PWM;
    pwm_set(tmp);
    }


//-----------------------------------------------------------------------------
static void pwm_decr(void)
    {
    uint16_t tmp = pwm_read();
    if(--tmp<1) tmp=1;
    pwm_set(tmp);
    }


//-----------------------------------------------------------------------------
static inline void doserate_calc(void)
    {
    static uint32_t pulsetot=0;  //counter

    pulsebuff[0]=pulsecnt;
    pulsecnt=0;

    for(uint8_t k=GEIGER_TIME; k>0; k--) pulsebuff[k]=pulsebuff[k-1];  //shift
    pulsetot+=pulsebuff[0];
    if(pulsetot>999999UL*3600/GEIGER_TIME) pulsetot=999999UL*3600/GEIGER_TIME;  //overflow 999999uR

    uint32_t tmprate=0;
    for(uint8_t k=GEIGER_TIME; k>0; k--) tmprate+=pulsebuff[k]; //calc dose rate
    if(tmprate>999999) tmprate=999999; //overflow
    doserate=tmprate;
    if(tmprate>maxrate) maxrate=tmprate;   //peak
    dosetot=(pulsetot*GEIGER_TIME/3600);   //dose
    }


//-----------------------------------------------------------------------------
static inline void doserate_reset(void)
    {
    for(uint8_t i=0; i<GEIGER_TIME+1; i++) pulsebuff[i]=0;
    doserate=0;
    maxrate=0;
    dosetot=0;
    }


//-----------------------------------------------------------------------------
void main(void)
    {
    IRCF2=1; IRCF1=0; IRCF0=1; //111=8M 110=4M 101=2M 100=1M 011=500k 010=250k 001=125k 000=31k

    //INTSRC PLLEN - TUN4 TUN3 TUN2 TUN1 TUN0
    OSCTUNE=0b00000001;

    SBOREN=1; //1=enable brown out reset
    SWDTEN=1; //0=disable watchdog timer
    RBPU=1;   //0=enable pull-up

    PEIE=1;
    GIE=1;

    //TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0
    T0CON=0b00000000 | TMR0_PRESCALER;
    TMR0=TMR0_OVF_PRELOAD; //preload
    TMR0ON=1;
    TMR0IF=0;
    TMR0IE=1;

    //RD16 T1RUN T1CKPS1 T1CKPS0 T1OSCEN T1SYNC TMR1CS TMR1ON
    T1CON=0b00000000 | TMR1_PRESCALER;
    TMR1=TMR1_OVF_PRELOAD;
    TMR1ON=0;
    TMR1IF=0;
    TMR1IE=0;

    //- TOUTPS3 TOUTPS2 TOUTPS1 TOUTPS0 TMR2ON T2CKPS1 T2CKPS0
    T2CON=0b00000000 | TMR2_PRESCALER;  //00-1:1 //01-1:4  //1x-1:16
    TMR2=0;
    TMR2ON=0;
    TMR2IF=0;
    TMR2IE=0;

    ////RD16 T3CCP2 T3CKPS1 T3CKPS0 T3CCP1 T3SYNC TMR3CS TMR3ON
    T3CON=0b00000000 | TMR3_PRESCALER;
    TMR3=TMR3_OVF_PRELOAD;;
    TMR3ON=0;
    TMR3IF=0;
    TMR3IE=0;

    //- - DCxB1 DCxB0 CCPxM3 CCPxM2 CCPxM1 CCPxM0
    CCP2CON=0b00000000;

    //SETUP FOR PWM OPERATION
    PR2=255;
    CCPR2L=0;  ///pulse width us
    DC2B1=0;
    DC2B0=0;
    INIT_PULSEOUT;
    TMR2=0;
    //T2CKPS0=1; //1:4 prescaler
    TMR2ON=1;
    CCP2M3=1; CCP2M2=1; CCP2M1=0; CCP2M0=0;

    //PWM Period = (PR2 + 1) * 4 * Tosc * (TMR2 Prescale Value)

    //P1M1 P1M0 DC1B1 DC1B0 CCP1M3 CCP1M2 CCP1M1 CCP1M0
    CCP1CON=0b00000000;

    CMCON=0b000111; //Comparators Off

    INIT_HV_SENS;
    INIT_B_SENS;

    ADCON1 = 0b001101; //5-Vref=Vss //4+Vref=Vdd //Port 0
    ADCON0 = 0b000000; //Channel 0  //A/D converter module is disabled
    //ADFM - ACQT2 ACQT1 ACQT0 ADCS2 ADCS1 ADCS0
    ADCON2 = 0b10010100; //TACQ 010=4TAD //TAD 000=FOSC/2 100=FOSC/4 101=FOSC/16 Fosc/4=0.5M=2us  //111=FRC
    ADON=0; //Turn off A/D module

    INIT_GEIGER_DET;

    /*** external interrupt 0 settigs ***/
    INTEDG0=0; //on falling edge
    INT0IE=1; //enable

    INIT_BUTTON;

    UPIN_ALL_INIT;

    lcd_init();

    uint8_t scrupd=0;
    uint8_t scrmode=0;
    uint16_t hvolt=0;
    uint16_t bvolt=0;

    //uint32_t t1=0;
    //uint32_t t2=0;

    for(;;)
        {
        if(syscnt==0)
            {
            syscnt=EVENT_PERIOD(200);

            bvolt=ADC_BVOLT(get_adc(0,8));
            hvolt=ADC_HVOLT(get_adc(1,8));

            if(hvolt<=MIN_HVOLT) pwm_incr();
            if(hvolt>=MAX_HVOLT) pwm_decr();

            scrupd=0;
            }

        if(scrcnt==0)
            {
            scrcnt=EVENT_PERIOD(1000);

            doserate_calc();

            CLRWDT();
            scrupd=0;
            }

        if(scrupd==0)
            {
            scrupd=1;

            //sprintf(strbuff,"%2u%6lu", pwm_read(), doserate);
            scrmode ? sprintf(strbuff,"D %6lu", dosetot) : sprintf(strbuff,"R %6lu", doserate);
            lcd_goto(0,0);
            //t1=debugcnt;
            lcd_print(strbuff);
            //t2=debugcnt;

            //sprintf(strbuff,"%2lu ms", t2-t1);
            sprintf(strbuff,"%3u %4u", hvolt/10, bvolt);
            lcd_goto(1,0);
            lcd_print(strbuff);
            }

        switch(button_check())
            {
            case 2:
                scrupd=0;
                doserate_reset();
                DLED_RED;
                delay_ms(10);
                DLED_OFF;
                break;

            case 1:
                scrupd=0;
                scrmode ? scrmode=0 : scrmode=1;
                DLED_GREEN;
                delay_ms(10);
                DLED_OFF;
                break;

            case 0: break;
            default: break;
            }
        }
    }



