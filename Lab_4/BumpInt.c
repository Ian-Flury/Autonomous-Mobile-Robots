// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

// Edited by J. Tadrous 06/02/2022

#include <stdint.h>
#include "msp.h"
#include "CortexM.h" // Global Interrupt Control


void(*rTask)(uint8_t); // Globalizing input function



uint8_t FallingEdges4;



void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 4
    // Initialize Bump sensors
    // Make six Port 4 pins inputs
    // Activate interface pullup
    // pins 7,6,5,3,2,0  11101101 -> 1110111
    // Interrupt on falling edge (on touch)
    rTask = task;
    DisableInterrupts();
    FallingEdges4 = 0;
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;  // configure P1.4 as GPIO
    P4->DIR &= ~0xED;   // make P4 input Button 2
    P4->REN |= 0xED;    // enable pull resistors
    P4->OUT |= 0xED;    // P4 pull-up
    P4->IES |= 0xED;    // P4 falling edge event
    P4->IFG &= ~0xED;   // clear flag
    P4->IE |= 0xED;     // arm interrupt on P4 7,6,5,3,2,0.
    NVIC->IP[9]=(NVIC->IP[9]&0xFF00FFFF)|0x00200000;    // bits 23-21
    NVIC->ISER[1] = 0x00000040; // enable bit 6
    EnableInterrupts();

}


uint8_t Bump_Read(void){
    // write this as part of Lab 4
    // Read current state of 6 switches
    // Returns a 6-bit positive logic result (0 to 63)
    // bit 5 Bump5
    // bit 4 Bump4
    // bit 3 Bump3
    // bit 2 Bump2
    // bit 1 Bump1
    // bit 0 Bump0
    uint8_t tmp = 0;
    uint8_t res = 0;
    uint8_t raw = P4->IN;

    uint8_t lower = raw & 0x0F;
    uint8_t upper = raw & 0xF0;
    tmp = lower & 0x01;

    res = (lower >> 1) | tmp;

    tmp = 0;
    tmp = upper >> 2;
    res |= tmp;

    return res;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    FallingEdges4++;
    rTask(Bump_Read()); // Execute task from high-level software
    P4->IFG &= ~0xED; // ACK all
}

