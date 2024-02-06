#include "msp.h"
#include "Clock.h"
#include "BumpInt.h"
#include "Clock.h"
#include "RobotLights.h"

void handle_collision(uint8_t ISR_data)
{
    Front_Lights_ON();
    Clock_Delay1ms(1000);
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

	BumpInt_Init(&handle_collision);

	while (1)
	{
	    Clock_Delay1ms(1000);
	}
}
