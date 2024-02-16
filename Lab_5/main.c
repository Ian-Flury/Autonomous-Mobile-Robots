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

#define Right &fsm[0]
#define Left &fsm[1]
#define Center &fsm[2]
#define HardRight &fsm[3]
#define HardLeft &fsm[4]
#define Backward &fsm[5]

State_t fsm[6]= {
    {0x01, 10, { HardRight, HardRight,  Center,     Center}},   // Turn Right
    {0x02, 10, { HardLeft,  Center,     HardLeft,   Center}},   // Turn Left
    {0x03, 20, { HardRight, Right,      Left,       Center}},   // Center
    {0x04, 20, { HardRight, HardRight,  Left,       Center}},   // Turn Hard Right
    {0x05, 20, { HardLeft,  Right,      HardLeft,   Center}},   // Turn Hard Left
    {0x06, 20, { Backward,  Right,      Left,       Center}}    // Center
};

State_t *Spt; // pointer to the current state
void main(void)
{
    uint16_t speed = 2500;
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
        case 1: // Left Forward (Turn Right)
            Motor_Forward(speed, speed/3);
            break;
        case 2: // Right Forward (Turn Left)
            Motor_Forward(speed/3, speed);
            break;
        case 3: // Forward
            Motor_Forward(speed, speed);
            break;
        case 4: // Left Forward, Right Backwards (Hard Right)
            Motor_Right(1.5*speed, 1.5*speed);
            break;
        case 5: // Right Forward, Left Backwards (Hard Left)
            Motor_Left(1.5*speed, 1.5*speed);
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
