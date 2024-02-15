#include "msp.h"
#include "Clock.h"
#include "RobotLights.h"
#include "Motor.h"

/**
 * main.c
 */

// Linked data structure - A 3-state Approach
struct State {
    uint32_t out; // 2-bit output
    uint32_t delay; // time to delay in 1ms
    const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left &fsm[1]
#define Right &fsm[2]

State_t fsm[3]= {
    {0x03, 20, { Right, Left, Right, Center }}, // Center
    {0x02, 20, { Left, Center, Right, Center }}, // Left
    {0x01, 20, { Right, Left, Center, Center }} // Right
};

State_t *Spt; // pointer to the current state
void main(void)
{
    uint16_t speed = 2000;
    uint8_t dataCenter;
    uint8_t leftMotor, rightMotor;
    uint32_t Time = 1000;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;// stop watchdog
    Clock_Init48MHz();
    Reflectance_Init();
    Motor_Init();
    Spt = Center;
    while (1){
        rightMotor = (Spt -> out) & 0x01; // get outputs from
        // current state
        leftMotor = ((Spt -> out) & 0x02)>>1;
        Motor_Forward(leftMotor*speed, rightMotor*speed); // apply outputs
        Clock_Delay1ms(Spt->delay); // stay in that state for
        // specified delay
        dataCenter = Reflectance_Center(Time); // get input from
        // central sensors
        Spt = Spt->next[dataCenter];
    }
}
