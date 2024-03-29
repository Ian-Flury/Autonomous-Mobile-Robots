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

// Handle Collisions
void HandleCollision(uint8_t ISR_data);
uint8_t bump;

/*
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
    uint16_t speed = 2750;
    uint16_t fspeed = 2750;
    double left_mm, center_mm, right_mm;

    while (1)
    {
        // Reset
        left = 0;
        right = 0;
        center = 0;

        // Compute Distance (mm)
        ADC_In17_14_16(&right, &center, &left);
        left_mm = compute_left_distance(left);
        right_mm = compute_right_distance(right);
        center_mm = compute_center_distance(center);
        ir_scan = control(left_mm, center_mm, right_mm);

        switch(ir_scan)
        {
        case 1: // Forward
            Motor_Forward(fspeed, fspeed);
            Clock_Delay1ms(5);
            break;

        case 2: // Slight Right
            Motor_Forward(fspeed*1.5, fspeed);
            Clock_Delay1ms(10);
            break;

        case 3: // Slight Left
            Motor_Forward(fspeed, fspeed*1.5);
            Clock_Delay1ms(10);
            break;

        case 4: // Turn Right
            Motor_Right(speed, speed);
            Clock_Delay1ms(650);
//            Motor_Forward(fspeed, fspeed);
//            Clock_Delay1ms(10);
            break;

        case 5: // Turn Left
            Motor_Left(speed, speed);
            Clock_Delay1ms(650);
//            Motor_Forward(fspeed, fspeed);
//            Clock_Delay1ms(5);
            break;

        case 6: // Turn 180
            Motor_Right(speed, speed);
            Clock_Delay1ms(1575);
            break;
//
//        case 7: // Backwards
//            Motor_Backward(speed, speed);
//            Clock_Delay1ms(15);
//            Motor_Stop();
//            break;

        default:
            break;
        }
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

uint8_t control(double left, double center, double right)
{
    uint8_t control = 1;
    uint16_t speed = 3000;
    if (center < 85) {
        // Dead End
        if (left < 130 & right < 130) {
            control = 5;
        }
        else if (right > left + 20) {
            control = 4;
        }
        else {
            control = 5;
        }
    }
    else {
        // Slight Left
        if (right < 90) {
            control = 3;
        }

        // Found Gap -> turn right
        if (right > 350){
            Motor_Forward(speed, speed);
            Clock_Delay1ms(1000);
            control = 4;
        }

        // Slight Right
        if (right > 90 & right < 350){
            control = 2;
        }
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
