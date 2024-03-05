#include "msp.h"
#include "Clock.h"
#include "ADC14.h"

#define NUM_OF_SAMPLES 30

double compute_left_distance(float left_avg)
{

}

double compute_right_distance(float right_avg)
{
	
}

double compute_center_distance(float center_avg)
{
	
}

/**
 * main.c
 */
void main(void)
{
	// initialization functions
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	ADC0_InitSWTriggerCh17_14_16();

	//  A17,  A16,   A14
	uint32_t left, right, center;

	uint32_t left_buf[NUM_OF_SAMPLES];
	uint32_t center_buf[NUM_OF_SAMPLES];
	uint32_t right_buf[NUM_OF_SAMPLES];

	float left_avg, center_avg, right_avg;

	while (1)
	{
		// collect the samples
		int i;
		for (i = 0; i < NUM_OF_SAMPLES; i++)
		{
			ADC_In17_14_16(&right, &center, &left);
			left_buf[i] = left;
			center_buf[i] = center;
			right_buf[i] = right;
		}

		// compute the average of the samples
		for (i = 0; i < NUM_OF_SAMPLES; i++)
		{
			left_avg += (float)left_buf[i];
			center_avg += (float)center_buf[i];
			right_avg += (float)right_buf[i];
		}
		left_avg = left_avg / NUM_OF_SAMPLES;
		center_avg = center_avg / NUM_OF_SAMPLES;
		right_avg = right_avg / NUM_OF_SAMPLES;

		// compute the distance in mm

		Clock_Delay1ms(1000);

		// reset
		left_avg = 0.0;
		right_avg = 0.0;
		center_avg = 0.0;
		left = 0;
		right = 0;
		center = 0;

//		uint16_t speed = 3000;
//		uint32_t delay = 0;
//		uint8_t collision_detected;
//
//		switch(collision_detected)
//		{
//		case 1: // Collision Right, Turn Left
//		    delay = 20;
//		    Motor_Left(speed, speed);
//            Back_Lights_ON();
//            Front_Lights_OFF();
//		    break;
//		case 2: // Collision Front, Reverse, Turn Right
//		    delay = 20;
//		    Motor_Backward(speed, speed);
//		    Clock_Delay1ms(delay);
//		    delay = 20;
//		    Motor_Right(speed, speed);
//            Back_Lights_ON();
//            Front_Lights_OFF();
//		    break;
//		case 4: // Collision Left, Turn Right
//		    delay = 20;
//		    Motor_Right(speed, speed);
//            Back_Lights_ON();
//            Front_Lights_OFF();
//		    break;
//		case 5: // Collision Sides, Turn Around
//		    delay = 110;
//		    Motor_Right(speed, speed);
//            Back_Lights_ON();
//            Front_Lights_OFF();
//		    break;
//		default:
//		    delay = 30;
//		    Motor_Forward(speed, speed);
//		    Front_Lights_ON();
//		    break;
//		}
//		Clock_Delay1ms(delay);
	}
}
