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


	while (1)
	{
	    Motor_Forward(2000, 2000);
	    Clock_Delay1ms(1000);

	    Motor_Stop();
        Clock_Delay1ms(500);

        Motor_Right(1000,2000);
        Clock_Delay1ms(500);

        Motor_Left(2000,1000);
        Clock_Delay1ms(500);

        Motor_Stop();
        Clock_Delay1ms(2000);


        // actual lab code:

        uint8_t sensor_data = 0;

        // forwards
        while (!sensor_data)
        {
            Motor_Forward(5000, 5000);
            sensor_data = Reflectance_Read(1000);
            if (sensor_data)
            {
                break;
            }
        }

        // backwards
        while (!sensor_data)
        {

        }
	}
}





