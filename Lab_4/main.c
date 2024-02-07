#include "msp.h"
#include "Clock.h"
#include "BumpInt.h"
#include "Clock.h"
#include "RobotLights.h"
#include "Motor.h"
#include "PWM.h"

uint8_t bump;

void HandleCollision(uint8_t ISR_data)
{
    bump = 0;
    bump = ISR_data;
    Front_Lights_OFF();
}

/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    Clock_Init48MHz();
    MvtLED_Init();
    // Motor_Init();
    BumpInt_Init(&HandleCollision);
    

    while (1)
    {

//	    MotorForward(); // 33 percent duty cycle
        Clock_Delay1ms(1000);
//	    bump = ;
//	    // if bump
//	    if (bump...)
//	    {
//            // stop
//            MotorStop();
//
//            // backup for 400 ms
//            MotorBackward();
//
//            // if bump left
//            if (bump > ) {
//                // turn right
//                MotorRight();
//            }
//            // if bump right
//            else if (bump < ) {
//                // turn left
//                MotorLeft();
//            }
//	    }
    }
}
