#include "msp.h"
#include "Clock.h"

/**
 * main.c
 */
uint8_t Sw1(void);
uint8_t Sw2(void);
void LED(uint8_t new);

void main(void)
{
	uint8_t curr_color = 1;
	uint8_t sw1 = 1;
	uint8_t sw2 = 1;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();

	// Switch Initialization (IN)
	P1 -> SEL1 &= ~0x12;
	P1 -> SEL0 &= ~0x12;
	P1 -> DIR &= ~0x12;
	P1 -> REN |= 0x12;
	P1 -> OUT |= 0x12;

	// LED Initialization (OUT)
	P2 -> SEL1 &= ~0x07;
	P2 -> SEL0 &= ~0x07;
	P2 -> DIR |= 0x07;

	while(1){
	    sw1 = Sw1();
	    sw2 = Sw2();
	    if (sw1 == 0 && sw2 == 0)
	    {
	        while(sw1 == 0 && sw2 == 0){
	            sw1 = Sw1();
	            sw2 = Sw2();
	        }
	        if (curr_color == 8){
	            LED(0);
	            curr_color = 1;
	        }
	        else {
	            LED(curr_color);
	            curr_color++;
	        }
	    }
	}
}

uint8_t Sw1(void){
    return (P1->IN&0x02 ) >> 1;
}

uint8_t Sw2(void){
    return (P1->IN&0x10) >> 4;
}

void LED(uint8_t new){
    P2->OUT = (P2->OUT&(~0x07))|new;
}
