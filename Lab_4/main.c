#include "msp.h"
#include "Clock.h"
#include "BumpInt.h"
#include "Clock.h"
#include "RobotLights.h"
#include "Motor.h"
#include "PWM.h"

uint8_t bump = 0;
int intflag = 0;

void HandleCollision(uint8_t ISR_data)
{
    bump = ~ISR_data;
    bump = bump & 0x3F;
    Motor_Stop();
    Motor_Backward(3000,3000);
    Clock_Delay1ms(400);
    Motor_Stop();
    intflag = 1;
}


/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    Clock_Init48MHz();
    MvtLED_Init();
    BumpInt_Init(&HandleCollision);
    Motor_Init();
    
    // start inital move forwards.

    while (1)
    {

	    if (intflag)
	    {
	        Clock_Delay1ms(1000);
	        intflag = 0;

            // if bump left
            if (bump & 0xF0) {
                // turn right
                Motor_Right(0,3000);
                Clock_Delay1ms(1000);
            }

            // if bump right
            if (bump & 0x0F) {
                // turn left
                Motor_Left(3000,0);
                Clock_Delay1ms(1000);
            }

            Motor_Stop();
	    } else {
	        Motor_Forward(3000,3000);
	        Clock_Delay1ms(100);
	    }
    }
}
