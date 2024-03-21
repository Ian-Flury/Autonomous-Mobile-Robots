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
//uint8_t robo_follow(uint8_t control);

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
    uint16_t speed = 4000;
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
        case 0: // Stop
            Motor_Stop();
            break;

        case 1: // Forward
            Motor_Forward(speed, speed);
            break;

        case 2: // Backwards
            Motor_Backward(speed, speed);
            break;

        case 3: // Slight Right
            Motor_Right(1.5*speed, speed);
            break;

        case 4: // Slight Left
            Motor_Left(speed, speed*1.5);
            break;

        case 5: // Turn Right
            Motor_Right(speed, speed);
            break;

        case 6: // Turn Left
            Motor_Left(speed, speed);
            break;

        default:
            break;
        }
        Clock_Delay1ms(5);
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

//uint8_t robo_follow(uint8_t control) {
//    uint8_t follow = 0;
//    return follow;
//}

uint8_t control(double left, double center, double right)
{
    uint16_t FAR = 400;
    uint16_t CLOSE = 110;

    // Stop
    if (left > FAR & center > FAR & right > FAR) {
        return 0;
    }

    // Forward
    if (center < FAR & center > CLOSE) {
        return 1;
    }

    // Backwards
    if (center < CLOSE) {
        return 2;
    }

    // Slight Right
    if (left > CLOSE & center > CLOSE & right > FAR) {
        return 3;
    }

    // Slight Left
    if (left > FAR & center > CLOSE & right > CLOSE) {
        return 4;
    }

    // Turn Right
    if (center > FAR & right > CLOSE) {
        return 5;
    }

    // Turn Left
    if (center > FAR & left > CLOSE) {
        return 6;
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
