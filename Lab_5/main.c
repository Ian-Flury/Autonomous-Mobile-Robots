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
#define HardLeft &fsm[3]
#define HardRight &fsm[4]
#define Backward &fsm[5]


State_t fsm[6]= {
//               0x00       0x01       0x10      0x11
    {0x06, 20, { Backward, Right,     Left,     Center }},     // Backwards
    {0x05, 20, { Backward, Center,    HardLeft, Center }},     // HardLeft
    {0x04, 20, { Backward, HardRight, Center,   Center }},     // HardRight
    {0x03, 20, { Backward, Right,     Left,     Center }},     // Center
    {0x02, 20, { Backward, Center,    HardLeft, Center }},    // Left
    {0x01, 20, { Backward, HardRight, Center,   Center }}     // Right
};

State_t *Spt; // pointer to the current state
void main(void)
{
    uint16_t speed = 2000;
    uint8_t next_state;
    uint8_t leftMotor, rightMotor;
    uint32_t Time = 1000;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;// stop watchdog
    Clock_Init48MHz();
    Reflectance_Init();
    Motor_Init();
    Spt = Center;

    while (1){
        switch(Spt->out)
        {
        case 1: // Turn Right
            Motor_Right(0, speed);
            break;
        case 2: // Turn Left
            Motor_Left(speed, 0);
            break;
        case 3: // Forward
            Motor_Forward(speed, speed);
            break;
        case 4: // Hard Right
            Motor_Right(0, speed);
            Clock_Delay1ms(10);
            Motor_Forward(speed, 0);
            break;
        case 5: // Hard Left
            Motor_Left(speed, 0);
            Clock_Delay1ms(10);
            Motor_Forward(speed, 0);
            break;
        case 6: // Backwards
            Motor_Backward(speed, speed);
            break;
        default:
            break;
        }
        Clock_Delay1ms(Spt->delay);
        next_state = Reflectance_Center(Time);
        Spt = Spt->next[next_state];
    }
}
