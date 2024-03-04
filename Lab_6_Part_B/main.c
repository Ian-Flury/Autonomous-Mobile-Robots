#include "msp.h"
#include "Clock.h"
#include "ADC14.h"
#include "Motor.h"
#include "RobotLights.h"
#include <math.h>

#define NUM_OF_SAMPLES 5

double compute_left_distance(float left_avg);
double compute_right_distance(float right_avg);
double compute_center_distance(float center_avg);
uint8_t control(double left_mm, double center_mm, double right_mm);

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

	//       A17,  A16,   A14
	uint32_t left, right, center;
	uint32_t delay = 100;
	uint8_t  collision_detected;
	uint16_t speed = 3000;

	uint32_t left_buf[NUM_OF_SAMPLES];
	uint32_t center_buf[NUM_OF_SAMPLES];
	uint32_t right_buf[NUM_OF_SAMPLES];

	float left_avg, center_avg, right_avg;
	double left_mm, center_mm, right_mm;

	while (1)
	{
	    // Reset
        left_avg = 0.0;
        right_avg = 0.0;
        center_avg = 0.0;
        left = 0;
        right = 0;
        center = 0;

	    // Collect Samples
		int i;
		for (i = 0; i < NUM_OF_SAMPLES; i++) {
			ADC_In17_14_16(&right, &center, &left);
			left_buf[i] = left;
			center_buf[i] = center;
			right_buf[i] = right;
		}

		// Compute Sample Average
		for (i = 0; i < NUM_OF_SAMPLES; i++) {
			left_avg += (float)left_buf[i];
			center_avg += (float)center_buf[i];
			right_avg += (float)right_buf[i];
		}
		left_avg = left_avg / NUM_OF_SAMPLES;
		center_avg = center_avg / NUM_OF_SAMPLES;
		right_avg = right_avg / NUM_OF_SAMPLES;

		// Compute Distance (mm) - Identify Collisions
		left_mm = compute_left_distance(left_avg);
		right_mm = compute_right_distance(right_avg);
		center_mm = compute_center_distance(center_avg);
        collision_detected = control(left_mm, center_mm, right_mm);

        switch(collision_detected)
        {
        case 1: // Collision Right, Slight Left
            delay = 15;
            Motor_Forward(speed, 1.1 * speed);
            break;

        case 2: // Collision Front, Reverse -> Turn Right
            Motor_Backward(speed, speed);
            Clock_Delay1ms(delay * 5);
            Motor_Stop();

            delay = 950;
            Motor_Right(speed, speed);
            break;

        case 3: // Collision Front-Right, Slight Left
            delay = 15;
            Motor_Forward(speed, 1.1 * speed);
            break;

        case 4: // Collision Left, Slight Right
            delay = 15;
            Motor_Forward(1.1 * speed, speed);
            break;

        case 5: // Collision Sides, Turn Around
            delay = 950;
            Motor_Stop();
            Motor_Right(speed, speed);
            break;

        case 6: // Collision Front-Left, Slight Right
            delay = 15;
            Motor_Forward(1.1 * speed, speed);
            break;

        case 7: // Corner Collision
            Motor_Backward(speed, speed);
            Clock_Delay1ms(delay * 5);
            Motor_Stop();

            delay = 950;
            Motor_Right(speed, speed);
            break;

        default:
            Motor_Forward(speed, speed);
            break;
        }
        Clock_Delay1ms(delay);
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
