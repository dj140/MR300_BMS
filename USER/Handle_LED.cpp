#include "FileGroup.h"

void LED_Init()
{
        pinMode(LED_Pin, OUTPUT);
}

void Task_InfoLED()
{
	
        //mtm.TaskSetIntervalTime(TP_InfoLED, 500);
        togglePin(LED_Pin);
}

