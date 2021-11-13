// main.h
// PIC18F4520
//
// 04-Sep-2020


#ifndef MAIN_H
#define MAIN_H


#define _XTAL_FREQ 2000000


// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (3==Minimum setting)
/*
BORV=11 2.00-2.11-2.22 V
BORV=10 2.65-2.79-2.93 V
BORV=01 4.11-4.33-4.55 V
BORV=00 4.36-4.59-4.82 V
*/

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 2048     // Watchdog Timer Postscale Select bits (1-32768)

// CONFIG3H
#pragma config CCP2MX = PORTBE  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>


/* unused pins PORTA */
//#define UPIN_A0  LATAbits.LATA0=0; TRISAbits.RA0=0
//#define UPIN_A1  LATAbits.LATA1=0; TRISAbits.RA1=0
#define UPIN_A2  LATAbits.LATA2=0; TRISAbits.RA2=0
#define UPIN_A3  LATAbits.LATA3=0; TRISAbits.RA3=0
#define UPIN_A4  LATAbits.LATA4=0; TRISAbits.RA4=0
#define UPIN_A5  LATAbits.LATA5=0; TRISAbits.RA5=0
#define UPIN_A6  LATAbits.LATA6=0; TRISAbits.RA6=0
#define UPIN_A7  LATAbits.LATA7=0; TRISAbits.RA7=0

#define UPIN_A_INIT  /*UPIN_A0; UPIN_A1;*/ UPIN_A2; UPIN_A3; UPIN_A4; UPIN_A5; UPIN_A6; UPIN_A7

/* unused pins PORTB */
//extint
//#define UPIN_B0  LATBbits.LATB0=0; TRISBbits.RB0=0
#define UPIN_B1  LATBbits.LATB1=0; TRISBbits.RB1=0
#define UPIN_B2  LATBbits.LATB2=0; TRISBbits.RB2=0
//pwm
//#define UPIN_B3  LATBbits.LATB3=0; TRISBbits.RB3=0
#define UPIN_B4  LATBbits.LATB4=0; TRISBbits.RB4=0
//ICSP
//#define UPIN_B5  LATBbits.LATB5=0; TRISBbits.RB5=0
//#define UPIN_B6  LATBbits.LATB6=0; TRISBbits.RB6=0
//#define UPIN_B7  LATBbits.LATB7=0; TRISBbits.RB7=0

#define UPIN_B_INIT  /*UPIN_B0;*/ UPIN_B1; UPIN_B2; /*UPIN_B3;*/ UPIN_B4 /*; UPIN_B5; UPIN_B6; UPIN_B7*/

/* unused pins PORTC */
#define UPIN_C0  LATCbits.LATC0=0; TRISCbits.RC0=0
#define UPIN_C1  LATCbits.LATC1=0; TRISCbits.RC1=0
#define UPIN_C2  LATCbits.LATC2=0; TRISCbits.RC2=0
//i2c
#define UPIN_C3  LATCbits.LATC3=0; TRISCbits.RC3=0
//i2c
#define UPIN_C4  LATCbits.LATC4=0; TRISCbits.RC4=0
#define UPIN_C5  LATCbits.LATC5=0; TRISCbits.RC5=0
#define UPIN_C6  LATCbits.LATC6=0; TRISCbits.RC6=0
//conn. #3  //pullup 1.1M
//button
//#define UPIN_C7  LATCbits.LATC7=0; TRISCbits.RC7=1

#define UPIN_C_INIT   UPIN_C0; UPIN_C1; UPIN_C2; UPIN_C3; UPIN_C4; UPIN_C5; UPIN_C6 /*; UPIN_C7*/

/* unused pins PORTD */
//display 4-bit bus
//#define UPIN_D0  LATDbits.LATD0=0; TRISDbits.RD0=0
//#define UPIN_D1  LATDbits.LATD1=0; TRISDbits.RD1=0
//#define UPIN_D2  LATDbits.LATD2=0; TRISDbits.RD2=0
//#define UPIN_D3  LATDbits.LATD3=0; TRISDbits.RD3=0
#define UPIN_D4  LATDbits.LATD4=0; TRISDbits.RD4=0
#define UPIN_D5  LATDbits.LATD5=0; TRISDbits.RD5=0
//bicolor led
//#define UPIN_D6  LATDbits.LATD6=0; TRISDbits.RD6=0
//#define UPIN_D7  LATDbits.LATD7=0; TRISDbits.RD7=0

#define UPIN_D_INIT  /*UPIN_D0; UPIN_D1; UPIN_D2; UPIN_D3;*/ UPIN_D4; UPIN_D5 /*; UPIN_D6; UPIN_D7*/

/* unused pins PORTE */
//display
//#define UPIN_E0  LATEbits.LATE0=0; TRISEbits.RE0=0
//#define UPIN_E1  LATEbits.LATE1=0; TRISEbits.RE1=0
#define UPIN_E2  LATEbits.LATE2=0; TRISEbits.RE2=0

#define UPIN_E_INIT  /*UPIN_E0;  UPIN_E1;*/ UPIN_E2


#define UPIN_ALL_INIT  UPIN_A_INIT; UPIN_B_INIT; UPIN_C_INIT; UPIN_D_INIT; UPIN_E_INIT


#define DLED_LAT_0   LATDbits.LATD6
#define DLED_TRIS_0  TRISDbits.RD6

#define DLED_LAT_1   LATDbits.LATD7
#define DLED_TRIS_1  TRISDbits.RD7

#define DLED_OFF  DLED_LAT_0=0; DLED_TRIS_0=0; DLED_LAT_1=0; DLED_TRIS_1=0

#define DLED_RED    DLED_LAT_0=0; DLED_TRIS_0=0; DLED_LAT_1=1; DLED_TRIS_1=0
#define DLED_GREEN  DLED_LAT_0=1; DLED_TRIS_0=0; DLED_LAT_1=0; DLED_TRIS_1=0

#define DEBUG_GREEN_BLINK  DLED_GREEN; delay_ms(10); DLED_OFF
#define DEBUG_RED_BLINK    DLED_RED; delay_ms(10); DLED_OFF


#define PULSEOUT_TRIS  TRISBbits.RB3
#define PULSEOUT_LAT   LATBbits.LATB3

#define INIT_PULSEOUT  PULSEOUT_LAT=0; PULSEOUT_TRIS=0


#define HV_SENS_LAT   LATAbits.LATA1
#define HV_SENS_TRIS  TRISAbits.RA1

#define INIT_HV_SENS  HV_SENS_LAT=0; HV_SENS_TRIS=1


#define B_SENS_LAT   LATAbits.LATA0
#define B_SENS_TRIS  TRISAbits.RA0

#define INIT_B_SENS  B_SENS_LAT=0; B_SENS_TRIS=1


#define GEIGER_DET_LAT   LATBbits.LATB0
#define GEIGER_DET_TRIS  TRISBbits.RB0

#define INIT_GEIGER_DET  GEIGER_DET_LAT=0; GEIGER_DET_TRIS=1


//Vcc, mV
#define ADC_REF 4975UL
//ADC resolution
#define ADC_RES 1024UL

#define HVOLT_DIV 1895UL
#define ADC_HVOLT(ADC) (uint16_t)(((((uint32_t)ADC*HVOLT_DIV)/1000UL)*ADC_REF)/ADC_RES)

#define BVOLT_DIV 3111UL
#define ADC_BVOLT(ADC) (uint16_t)(((((uint32_t)ADC*BVOLT_DIV)/1000UL)*ADC_REF)/ADC_RES)


/* timer 0 prescaler settings */
#define  TMR0_PRESCALER_256  0b00000111
#define  TMR0_PRESCALER_128  0b00000110
#define  TMR0_PRESCALER_64   0b00000101
#define  TMR0_PRESCALER_32   0b00000100
#define  TMR0_PRESCALER_16   0b00000011
#define  TMR0_PRESCALER_8    0b00000010
#define  TMR0_PRESCALER_4    0b00000001
#define  TMR0_PRESCALER_2    0b00000000

#define TMR0_PRESCALER  TMR0_PRESCALER_2

#define TMR0_OVF_FREQ     100U
#define TMR0_OVF_PRELOAD  (65536U-(((_XTAL_FREQ/4U)/2U)/TMR0_OVF_FREQ))


/* timer 1 prescaler settings */
#define TMR1_PRESCALER_8  0b00110000
#define TMR1_PRESCALER_4  0b00100000
#define TMR1_PRESCALER_2  0b00010000
#define TMR1_PRESCALER_1  0b00000000

#define TMR1_PRESCALER  TMR1_PRESCALER_1

#define TMR1_OVF_FREQ     1000U
#define TMR1_OVF_PRELOAD  (65536U-(((_XTAL_FREQ/4U)/1U)/TMR1_OVF_FREQ))


/* timer 2 prescaler settings */
#define TMR2_PRESCALER_16  0b00000010
#define TMR2_PRESCALER_4   0b00000001
#define TMR2_PRESCALER_1   0b00000000

#define TMR2_PRESCALER  TMR2_PRESCALER_1


/* timer 3 prescaler settings */
#define TMR3_PRESCALER_8  0b00110000
#define TMR3_PRESCALER_4  0b00100000
#define TMR3_PRESCALER_2  0b00010000
#define TMR3_PRESCALER_1  0b00000000

#define TMR3_PRESCALER  TMR3_PRESCALER_1

#define TMR3_OVF_FREQ     1000U
#define TMR3_OVF_PRELOAD  (65536U-(((_XTAL_FREQ/4U)/1U)/TMR3_OVF_FREQ))


//NOTE
//Preload timer: 65536-(((Xtal_Freq/4)/Prescaler)/Int_Freq)
//Int_Freq: ((Xtal_Freq/4)/Prescaler)/(65536-Preload)
//Int_Time: 1/((Xtal_Freq/4)/Prescaler)/(65536-Preload)


#endif /* MAIN_H */

