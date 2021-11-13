// main.c
//
// 13-Nov-2021


#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "lcd.h"
#include "button.h"
#include "i2c.h"
#include "ds1307.h"
#include "at24c32.h"



#define MAX_HVOLT  4150
#define MIN_HVOLT  3850

#define MAX_PWM    30

#define GEIGER_TIME  36

#define VOLT_UPD_FREQ  5
#define SEC_UPD_FREQ   1


#define LOG_START_ADDR  2000

#define LOG_TIME_ADDR  LOG_START_ADDR
#define LOG_TIME_SIZE  4

#define LOG_DATA_ADDR  (LOG_TIME_ADDR+LOG_TIME_SIZE)
#define LOG_DATA_SIZE  1440


char strbuff[16]; //string buffer

uint8_t rtcdata[DS1307_RTC_SIZE];   //ds1307 rtc data

uint16_t pulsebuff[GEIGER_TIME+1];  //pulse counter //[0]-current

volatile uint16_t pulsecnt=0;  //counter

uint16_t highvolt=0;
uint16_t sysvolt=0;

uint32_t pulsetot=0;  //counter

uint32_t doserate=0;  //dose rate, uR/h
uint32_t peakrate=0;
uint32_t dosetot=0;

uint8_t tday=0; //99 days
uint8_t thrs=0;
uint8_t tmin=0;
uint8_t tsec=0;

uint8_t phrs=0;
uint8_t pmin=0;
uint8_t pdat=0;
uint8_t pmon=0;

uint8_t hvnorm=0;

volatile uint8_t voltflag=0;
volatile uint8_t secflag=0;



//-----------------------------------------------------------------------------
static inline void sys_cnt(void)
    {
    static uint8_t cnt0=0;
    static uint8_t cnt1=0;

    if(++cnt0>=(TMR0_OVF_FREQ/SEC_UPD_FREQ))
        {
        cnt0=0;
        secflag=0;
        }

    if(++cnt1>=(TMR0_OVF_FREQ/VOLT_UPD_FREQ))
        {
        cnt1=0;
        voltflag=0;
        }
    }


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

        sys_cnt();
        }

    if(HLVDIF && HLVDIE)  // high/low-voltage detect interrupt
        {
        CCP2CON=0b00000000;  //disable PWM
        INIT_PULSEOUT;

        lcd_clear();
        lcd_goto(0,0);
        lcd_print((char*)"LOW VOLT");

        DLED_RED;

        for(;;)
            {
            CLRWDT();
            }
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
    pulsebuff[0]=pulsecnt;
    pulsecnt=0;

    for(uint8_t k=GEIGER_TIME; k>0; k--) pulsebuff[k]=pulsebuff[k-1];  //shift
    pulsetot+=pulsebuff[0];
    if(pulsetot>999999UL*3600/GEIGER_TIME) pulsetot=999999UL*3600/GEIGER_TIME;  //overflow 999999uR

    uint32_t tmprate=0;
    for(uint8_t k=GEIGER_TIME; k>0; k--) tmprate+=pulsebuff[k]; //calc dose rate
    if(tmprate>999999) tmprate=999999; //overflow
    doserate=tmprate;
    if(tmprate>peakrate)
        {
        peakrate=tmprate;
        phrs=rtcdata[HOURS_REG];
        pmin=rtcdata[MINUTES_REG];
        pdat=rtcdata[DATE_REG];
        pmon=rtcdata[MONTH_REG];
        }

    dosetot=(pulsetot*GEIGER_TIME/3600);   //dose

    if(tday<99)  //dose time counters
        {
        if(++tsec>59)
            {
            tsec=0;
            if(++tmin>59)
                {
                tmin=0;
                if(++thrs>23)
                    {
                    thrs=0;
                    ++tday;
                    }
                }
            }
        }
    }


//-----------------------------------------------------------------------------
static inline void doserate_reset(void)
    {
    for(uint8_t i=0; i<GEIGER_TIME+1; i++) pulsebuff[i]=0;
    doserate=0;
    pulsetot=0;
    dosetot=0;
    peakrate=0;
    phrs=0;
    pmin=0;
    pdat=0;
    pmon=0;
    tday=0;
    thrs=0;
    tmin=0;
    tsec=0;
    }


//-----------------------------------------------------------------------------
static inline void screen_main(uint8_t scrmode)
    {
    lcd_cursor(0,0);

    if(scrmode==0)  //doserate + time
        {
        lcd_goto(0,0);
        lcd_char(hvnorm ? ' ' : 0);

        sprintf(strbuff,"%6lu", doserate);
        lcd_goto(0,2);
        lcd_print(strbuff);

        sprintf(strbuff, "%02u:%02u", rtcdata[HOURS_REG], rtcdata[MINUTES_REG]);
        lcd_goto(1,0);
        lcd_print(strbuff);
        }

    if(scrmode==1 || scrmode==2)  //peak doserate time/peak doserate date
        {
        lcd_goto(0,0);
        lcd_char('P');
        sprintf(strbuff,"%6lu", peakrate);
        lcd_goto(0,2);
        lcd_print(strbuff);

        if(scrmode==1) sprintf(strbuff, "%02u:%02u", phrs, pmin);
        else if(scrmode==2) sprintf(strbuff, "%02u/%02u", pdat, pmon);
        lcd_goto(1,0);
        lcd_print(strbuff);
        }

    if(scrmode==3)  //dose total time
        {
        lcd_goto(0,0);
        lcd_char('D');

        sprintf(strbuff,"%6lu", dosetot);
        lcd_goto(0,2);
        lcd_print(strbuff);

        sprintf(strbuff, "%02u-%02u:%02u", tday, thrs, tmin);
        lcd_goto(1,0);
        lcd_print(strbuff);
        }

    if(scrmode==4)  //time and date
        {
        sprintf(strbuff, "%02u:%02u:%02u", rtcdata[HOURS_REG], rtcdata[MINUTES_REG], rtcdata[SECONDS_REG]);
        lcd_goto(0,0);
        lcd_print(strbuff);
        sprintf(strbuff, "%02u/%02u/%02u", rtcdata[DATE_REG], rtcdata[MONTH_REG], rtcdata[YEAR_REG]);
        lcd_goto(1,0);
        lcd_print(strbuff);
        }

    if(scrmode==5)  //reset couters
        {
        lcd_goto(0,0);
        lcd_print((char*)"RESET");
        }

    if(scrmode==6 || scrmode==7 || scrmode==8)  //set clock
        {
        sprintf(strbuff, "%02u:%02u:%02u", rtcdata[HOURS_REG], rtcdata[MINUTES_REG], rtcdata[SECONDS_REG]);
        lcd_goto(1,0);
        lcd_print(strbuff);

        if(scrmode==6) lcd_goto(1,1);      //hrs
        else if(scrmode==7) lcd_goto(1,4); //min
        else if(scrmode==8) lcd_goto(1,7); //sec

        lcd_cursor(1,1);
        }

    if(scrmode==9 || scrmode==10 || scrmode==11 || scrmode==12)  //set date
        {
        sprintf(strbuff, "%02u", rtcdata[DAY_REG]);
        lcd_goto(0,0);
        lcd_print(strbuff);

        sprintf(strbuff, "%02u/%02u/%02u", rtcdata[DATE_REG], rtcdata[MONTH_REG], rtcdata[YEAR_REG]);
        lcd_goto(1,0);
        lcd_print(strbuff);

        if(scrmode==9) lcd_goto(0,1);       //day
        else if (scrmode==10) lcd_goto(1,1); //date
        else if (scrmode==11) lcd_goto(1,4); //month
        else if (scrmode==12) lcd_goto(1,7); //year

        lcd_cursor(1,1);
        }

    if(scrmode==100)  //system voltage
        {
        sprintf(strbuff,"%3u", highvolt/10);
        lcd_goto(0,0);
        lcd_print(strbuff);

        sprintf(strbuff,"%1u.%02u", sysvolt/1000, (sysvolt%1000)/10);
        lcd_goto(0,4);
        lcd_print(strbuff);

        sprintf(strbuff,"%02u", pwm_read());
        lcd_goto(1,0);
        lcd_print(strbuff);
        }
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
    TMR0=TMR0_OVF_PRELOAD;
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
    TMR3=TMR3_OVF_PRELOAD;
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

    HLVDCON=0b11110; //low-voltage detect enable //1110 = 4.36-4.59-4.82

    LVDIF=0;
    LVDIE=1;  //low-voltage detect interrupt enable

    BUTTON_INIT;
    UPIN_ALL_INIT;

    lcd_init();
    i2c_init();
    rtc_init();

    uint8_t scrupd=0;
    uint8_t scrmode=0;

    for(;;)
        {
        if(!voltflag)
            {
            voltflag=1;

            sysvolt=ADC_BVOLT(get_adc(0,8));
            highvolt=ADC_HVOLT(get_adc(1,8));

            if(highvolt<=MIN_HVOLT) pwm_incr();
            if(highvolt>=MAX_HVOLT) pwm_decr();

            (highvolt>MIN_HVOLT && highvolt<MAX_HVOLT) ? hvnorm=1: hvnorm=0;

            if(!hvnorm || scrmode==100) scrupd=0;
            }

        if(!secflag)
            {
            secflag=1;

            doserate_calc();

            CLRWDT();
            scrupd=0;
            }

        if(!scrupd)
            {
            scrupd=1;

            rtc_read(rtcdata);
            screen_main(scrmode);
            }

        switch(button_check())
            {
            case 2:
                scrupd=0;
                if(scrmode==0) { scrmode++; lcd_clear(); break; }
                if(scrmode==1) { scrmode=3; lcd_clear(); break; }
                if(scrmode>2 && scrmode<12) { scrmode++; lcd_clear(); break; }
                if(scrmode==12) { scrmode=0; lcd_clear(); break; }
                break;

            case 1:
                scrupd=0;
                if(scrmode==0) { scrmode=100; lcd_clear(); break; }
                if(scrmode==1) { scrmode=2; break; }
                if(scrmode==2) { scrmode=0; lcd_clear(); break; }
                if(scrmode==3) { scrmode=0; lcd_clear(); break; }
                if(scrmode==4) { scrmode=0; lcd_clear(); break; }

                if(scrmode==5) { scrmode=0; doserate_reset(); lcd_clear(); break; }

                if(scrmode==6) { rtc_set_hrs_incr(rtcdata); break; }
                if(scrmode==7) { rtc_set_min_incr(rtcdata); break; }
                if(scrmode==8) { rtc_set_sec(0); break; }
                if(scrmode==9) { rtc_set_day_incr(rtcdata); break; }
                if(scrmode==10) { rtc_set_dat_incr(rtcdata); break; }
                if(scrmode==11) { rtc_set_mon_incr(rtcdata); break; }
                if(scrmode==12) { rtc_set_year_incr(rtcdata); break; }

                if(scrmode==100) { scrmode=0; lcd_clear(); break; }
                break;

            case 0: break;
            default: break;
            }
        }
    }


