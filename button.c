

#include "button.h"



volatile uint8_t btncnt=0;
volatile uint8_t hldcnt=0;



//-----------------------------------------------------------------------------
inline void button_cont(void)
    {
    if(btncnt) btncnt--;
    if(hldcnt) hldcnt--;
    }


//-----------------------------------------------------------------------------
inline uint8_t button_check(void)
    {
    uint8_t k=0;

    static uint8_t stage=0;

    switch(stage)
        {
        case 0:
            if(btncnt==0)
                {
                btncnt=EVENT_PERIOD_MS(30);

                if(!BUTTON_PIN)
                    {
                    stage=1;
                    }
                }
            break;

        case 1:
            if(btncnt==0)
                {
                if(!BUTTON_PIN)
                    {
                    stage=2;
                    hldcnt=255;
                    }
                else stage=0;
                }
            break;

        case 2:
            if(hldcnt<(0xff-EVENT_PERIOD_MS(500)))
                {
                k=2;
                stage=4;
                }
            else if(btncnt==0)
                {
                btncnt=EVENT_PERIOD_MS(30);

                if(BUTTON_PIN)
                    {
                    stage=3;
                    btncnt=EVENT_PERIOD_MS(30);
                    }
                }
            break;

        case 3:
            if(btncnt==0)
                {
                if(BUTTON_PIN)
                    {
                    stage=0;
                    k=1;
                    }
                }
            break;

        case 4:
            if(btncnt==0)
                {
                btncnt=EVENT_PERIOD_MS(30);

                if(BUTTON_PIN)
                    {
                    stage=5;
                    btncnt=EVENT_PERIOD_MS(30);
                    }
                }
            break;

        case 5:
            if(btncnt==0)
                {
                if(BUTTON_PIN)
                    {
                    stage=0;
                    }
                }
            break;
        }

    return k;
    }



