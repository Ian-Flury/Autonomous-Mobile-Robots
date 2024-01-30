#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "PWM.h"
#include "Reflectance.h"


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	Reflectance_Init();
	Motor_Init();

	// delay for a second so the robot doesn't take off when we click the button.
	Clock_Delay1ms(750);

    uint8_t sensor_data = Reflectance_Read(1000);
    // forwards
    while (!sensor_data)
    {
        Motor_Forward(5000,5000);
        Clock_Delay1ms(10);
        sensor_data = Reflectance_Read(1000);
        if (sensor_data != 0)
        {
            Motor_Stop();
            break;
        }
    }

    Clock_Delay1ms(500);

    sensor_data = Reflectance_Read(1000);
    while (!sensor_data)
    {
        Motor_Backward(1200,1200);
        Clock_Delay1ms(10);
        sensor_data = Reflectance_Read(1000);
        if (sensor_data != 0)
        {
            Motor_Stop();
            break;
        }
    }
}





