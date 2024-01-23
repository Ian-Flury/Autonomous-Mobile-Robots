#include "msp.h"
#include "Clock.h"

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	P8 -> SEL0 &= ~ 0xC0; // Chassis rear LEDs
    P8 -> SEL1 &= ~ 0xC0; // P8.7-6
    P8 -> DIR |= 0xC0;

    P2 -> SEL0 &= ~0x07; // BGR LED pins are
    P2 -> SEL1 &= ~0x07; // P2.2-0;
    P2 -> DIR |= 0x07;

    int Color = 0;
    int Count = 0;
    while(1){
        P8 ->OUT ^= 0xC0;
        Clock_Delay1ms(100);
        Count++;
        if (Count == 4){
            Count = 0;
            Color++;
            P2 -> OUT |= Color;
            P2 -> OUT &= Color;
        }
    }
}
