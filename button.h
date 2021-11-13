//04-Sep-2020


#ifndef BUTTON_H_
#define	BUTTON_H_


#include <stdint.h>
#include <stdio.h>

#include <xc.h>


#define BUTTON_PIN   PORTCbits.RC7
#define BUTTON_TRIS  TRISCbits.RC7

#define BUTTON_INIT  BUTTON_PIN=0; BUTTON_TRIS=1


#define TMR_OVF_FREQ        100
#define EVENT_PERIOD_MS(x)  ((x)/(1000/TMR_OVF_FREQ))


inline void button_cont(void);
inline uint8_t button_check(void);


#endif  //BUTTON_H_

