#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "PWM.h"


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	PWM_Init34(15000, 1, 1);
}
