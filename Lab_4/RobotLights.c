#include <RobotLights.h>
#include "msp.h"
/*
 * MovementLights.c
 *
 *  Created on: Oct 23, 2022
 *      Author: John Tadrous
 */

/* MovementLights initializes and supports the LEDs on the chassis board and the
 * colored BGR LED on the MSP432 launchpad
 * BGR lights: P2.2-P2.0
 * LED Front Right: P8.5
 * LED Front Left: P8.0
 * LED Rear Right: P8.7
 * LED Rear Left: P8.6
 */

void MvtLED_Init(){
    // FRONT and REAR
    P8 -> SEL1 &= ~0xE1;
    P8 -> SEL0 &= ~0xE1;
    P8 -> DIR |= 0xE1;

    // BGR
    P2 -> SEL1 &= ~0x07;
    P2 -> SEL0 &= ~0x07;
    P2 -> DIR |= 0x07;
}

// Turn on white light and chassis front lights
void Front_Lights_ON(){
    // FRONT
    P8->OUT = (P8->OUT&(~0x21))|0x21;
    // BGR
    P2->OUT = (P2->OUT&(~0x07))|7;
}

// Switch off BGR and chassis front lights
void Front_Lights_OFF(){
    // FRONT
    P8->OUT = (P8->OUT&(~0x21))&~0x21;
    // BGR
    P2->OUT = (P2->OUT&(~0x07))|0;
}

// Turn on back lights
void Back_Lights_ON(){
    P8->OUT = (P8->OUT&(~0xC0))|0xC0;
}

// Turn off back lights
void Back_Lights_OFF(){
    P8->OUT = (P8->OUT&(~0xC0))&~0xC0;
}
