#include "msp.h"
#include "Clock.h"
#include "ADC14.h"
#include "Motor.h"
#include "RobotLights.h"
#include <math.h>
#include "BumpInt.h"

#define NUM_OF_SAMPLES 1

double compute_left_distance(float left_avg);
double compute_right_distance(float right_avg);
double compute_center_distance(float center_avg);
uint8_t control(double left_mm, double center_mm, double right_mm);

// Handle Collisions
uint8_t bump;
void HandleCollision(uint8_t ISR_data);

// FSM
struct State {
    uint32_t out;
    uint32_t delay; // MAY NOT NEED THIS
    const struct State *next[8]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Forward &fsm[0]
#define SlightRight &fsm[1]
#define SlightLeft &fsm[2]
#define TurnRight &fsm[3]
#define TurnLeft &fsm[4]
#define Turn180 &fsm[5]
#define Backwards &fsm[6]


State_t fsm[7]= {
//  IR READING:   0x00        0x01           0x02          0x03           0x04            0x05        0x06            0x07
    {0x01, 20,  { Forward,    SlightLeft,    TurnRight,    SlightLeft,    SlightRight,    Turn180,    SlightRight,    TurnRight    }},  // Forward
    {0x02, 15,  { Forward,    SlightLeft,    TurnRight,    SlightLeft,    SlightRight,    Turn180,    SlightRight,    TurnRight    }},  // Slight Right
    {0x03, 15,  { Forward,    SlightLeft,    TurnRight,    SlightLeft,    SlightRight,    Turn180,    SlightRight,    TurnRight    }},  // Slight Left
    {0x04, 850, { Forward,    SlightLeft,    Turn180,      SlightLeft,    SlightRight,    Turn180,    SlightRight,    Turn180      }},  // Turn Right
    {0x05, 850, { Forward,    SlightLeft,    TurnLeft,     SlightLeft,    SlightRight,    Turn180,    SlightRight,    TurnLeft     }},  // Turn Left
    {0x06, 1800,{ Forward,    SlightLeft,    TurnLeft,     SlightLeft,    SlightRight,    Turn180,    SlightRight,    TurnLeft     }},  // Turn 180 Degrees
    {0x07, 10,  { Forward,    SlightLeft,    Turn180,      SlightLeft,    SlightRight,    Turn180,    SlightRight,    Turn180      }}   // Backwards
};

State_t *Spt; // pointer to the current state

/**
 * main.c
 */

void main(void)
{
	// Initialization Functions
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	ADC0_InitSWTriggerCh17_14_16();
	Motor_Init();
	BumpInt_Init(&HandleCollision);

	//       A17,  A16,   A14
	uint32_t left, right, center;
	uint8_t  collision_detected;
	uint8_t next_state;

	uint16_t speed = 3000;

	double left_mm, center_mm, right_mm;

	Spt = Forward;


	while (1)
	{
	    // Reset
        left = 0;
        right = 0;
        center = 0;

        // Compute Distance (mm) - Identify
        ADC_In17_14_16(&right, &center, &left);
        left_mm = compute_left_distance(left);
        right_mm = compute_right_distance(right);
        center_mm = compute_center_distance(center);

        next_state = control(left_mm, center_mm, right_mm);
        Spt = Spt->next[next_state];


        switch(Spt->out)
        {
        case 1: // Forward
            Motor_Forward(speed, speed);
            break;

        case 2: // Slight Right

            Motor_Right(speed, speed);
            break;

        case 3: // Slight Left
            Motor_Left(speed, speed);
            break;

        case 4: // Turn Right
            Motor_Right(speed, speed);
            break;

        case 5: // Turn Left
            Motor_Left(speed, speed);
            break;

        case 6: // Turn 180
            Motor_Right(speed, speed);
            break;

        case 7: // Backwards
            Motor_Backward(speed, speed);
            Clock_Delay1ms(15);
            Motor_Stop();
            break;

        default:
            break;
        }
        Clock_Delay1ms(Spt->delay);


    }
}

double compute_left_distance(float left_avg)
{
    double exp = -1.239;
    double coeff = pow(10,7);
    return (coeff * pow(left_avg, exp));
}

double compute_right_distance(float right_avg)
{
    double exp = -1.217;
    double coeff = 8000000;
    return (coeff * pow(right_avg, exp));
}

double compute_center_distance(float center_avg)
{
    double exp = -1.248;
    double coeff = pow(10,7);
    return (coeff * pow(center_avg, exp));
}

uint8_t control(double left_mm, double center_mm, double right_mm)
{
    uint8_t control = 0;
    if (left_mm < 110) {
        control |= 0x04;
    }

    if (center_mm < 110) {
        control |= 0x02;
    }

    if (right_mm < 110) {
        control |= 0x01;
    }
    return control;
}

void HandleCollision(uint8_t ISR_data)
{
    Motor_Stop();
    Motor_Backward(3000,3000);
    Clock_Delay1ms(800);
    Motor_Stop();
    Motor_Right(3000,3000);
    Clock_Delay1ms(400);
    bump = ~ISR_data;
    bump = bump & 0x3F;
}
