#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "PWM.h"
#include "Reflectance.h"


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	Reflectance_Init();
	Motor_Init();


	while (1)
	{
	    Motor_Forward(10000, 10000);
	    Clock_Delay1ms(500);

	    Motor_Backward(10000,10000);

        Clock_Delay1ms(500);
	}
}
