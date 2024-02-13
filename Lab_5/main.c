#include "msp.h"
#include "Clock.h"
#include "RobotLights.h"
#include "Motor.h"

/**
 * main.c
 */
void main(void)
{
	// System Initialization
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	MvtLED_Init();
	Motor_Init();



}
