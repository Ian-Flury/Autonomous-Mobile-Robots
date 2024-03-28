#include <stdint.h>
#include "msp.h"
#include "AP.h"
#include "CortexM.h"
#include "Clock.h"
#include "RobotLights.h"

uint8_t frontLights;
uint8_t backLights;

void writeFrontLights(void);
void writeBackLights(void);
void lightStatus(void);

/**
 * main.c
 */
void main(void)
{
	volatile int r;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();
	MvtLED_Init();
	r = AP_Init();
	// Creating a GATT Service
	AP_AddService(0xFFF0);
	frontLights = 0;
	uint8_t frontLights_old = 0;
	backLights = 0;
	uint8_t backLights_old = 0;
	AP_AddCharacteristic(0xFFF1,1,&frontLights,0x02,0x08,"frontLights",0,&writeFrontLights);
	AP_AddCharacteristic(0xFFF2, 1, &backLights, 0x02, 0x08, "backLights", 0, &writeBackLights);
	// Notifications to echo back change in light status
	AP_AddNotifyCharacteristic(0xFFF3, 1, &frontLights, "Front Lights", lightStatus); // CCCD = 0
	AP_AddNotifyCharacteristic(0xFFF4, 1, &backLights, "Back Lights", lightStatus); // CCCD = 1;
	AP_RegisterService();
	AP_StartAdvertisement();

	while(1){
	    AP_BackgroundProcess();

	    if (frontLights_old != frontLights){
            AP_SendNotification(0);
            frontLights_old = frontLights;
        }

        if (backLights_old != backLights){
            AP_SendNotification(1);
            backLights_old = backLights;
        }
	}
}

void writeFrontLights(void){
    if (frontLights >0){
        Front_Lights_ON();
    }
    else{
        Front_Lights_OFF();
    }
}

void writeBackLights(void){
    if (backLights>0){
        Back_Lights_ON();
    }
    else{
        Back_Lights_OFF();
    }
}

void lightStatus(void){};
