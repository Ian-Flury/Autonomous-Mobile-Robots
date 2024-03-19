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
uint8_t control(double left, double center, double right);
uint8_t robo_action(uint8_t control, double left, double center, double right);

// Handle Collisions
uint8_t bump;
void HandleCollision(uint8_t ISR_data);

/**
 * main.c
 */

void main(void)
{
    // Initialization Functions
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init48MHz();
    ADC0_InitSWTriggerCh17_14_16();
    Motor_Init();
    BumpInt_Init(&HandleCollision);

    //       A17,  A16,   A14
    uint32_t left, right, center;
    uint8_t ir_scan;
    uint16_t speed = 3000;
    double left_mm, center_mm, right_mm;

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
        ir_scan = control(left_mm, center_mm, right_mm);

        switch(robo_action(ir_scan, left_mm, center_mm, right_mm))
        {
        case 1: // Forward
            Motor_Forward(speed, speed);
            Clock_Delay1ms(10);
            break;

        case 2: // Slight Right
            Motor_Right(speed, speed);
            Clock_Delay1ms(15);
            break;

        case 3: // Slight Left
            Motor_Left(speed, speed);
            Clock_Delay1ms(15);
            break;

        case 4: // Turn Right
            Motor_Right(speed, speed);
            Clock_Delay1ms(785);
            break;

        case 5: // Turn Left
            Motor_Left(speed, speed);
            Clock_Delay1ms(785);
            break;

        case 6: // Turn 180
            Motor_Right(speed, speed);
            Clock_Delay1ms(1575);
            break;

        case 7: // Backwards
            Motor_Backward(speed, speed);
            Clock_Delay1ms(15);
            Motor_Stop();
            break;

        default:
            break;
        }
//        Clock_Delay1ms(Spt->delay);
//        Spt = Spt->next[next_state];
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

uint8_t robo_action(uint8_t control, double left, double center, double right)
{
    // ADD GAP DETECTION

    switch(control)
    {
    case 0: // Forward
        return 1;

    case 1: // Slight Left
        return 3;

    case 2:
        if (right > left + 5){
            return 4;
        }
        // Prioritize Turning Left @ T-Intersection
        else {
            return 5;
        }

    case 3: // Slight Left
        return 3;

    case 4: // Slight Right
        return 2;

    case 5: // Corner, Turn 180
        return 6;

    case 6: // Slight Right
        return 2;

    case 7:
        return 6;

    default:
        return 1;
    }
}

uint8_t control(double left, double center, double right)
{
    uint8_t control = 0;
    if (left < 100) {
        control |= 0x04;
    }

    if (center < 100) {
        control |= 0x02;
    }

    if (right < 100) {
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
