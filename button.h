//28-03-2020



#include <stdint.h>
#include <stdio.h>

#include <xc.h>


#define BUTTON_PIN   PORTCbits.RC7
#define BUTTON_TRIS  TRISCbits.RC7

#define INIT_BUTTON  BUTTON_PIN=0; BUTTON_TRIS=1


#define TMR_OVF_FREQ        100U
#define EVENT_PERIOD_MS(x)  ((x)/(1000U/TMR_OVF_FREQ))


inline void button_cont(void);
inline uint8_t button_check(void);



