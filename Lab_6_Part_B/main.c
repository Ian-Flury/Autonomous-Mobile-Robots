#include "msp.h"
#include "Clock.h"
#include "ADC14.h"
#include <math.h>

#define NUM_OF_SAMPLES 30

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
	if (left_mm < 110)
	{
		control |= 0x04;
	}

	if (center_mm < 110)
	{
		control |= 0x02;
	}

	if (right_mm < 110)
	{
		control |= 0x01;
	}
	return control;
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

		double left_mm = compute_left_distance(left_avg);
		double right_mm = compute_right_distance(right_avg);
		double center_mm = compute_center_distance(center_avg);

		Clock_Delay1ms(1000);

		// reset
		left_avg = 0.0;
		right_avg = 0.0;
		center_avg = 0.0;
		left = 0;
		right = 0;
		center = 0;
	}
}
