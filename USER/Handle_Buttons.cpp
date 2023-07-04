#include "FileGroup.h"


ButtonEvent btOK;

static void ButtonEvent_Handler(ButtonEvent* btn, int event)
{
    if(btn == &btOK)
    {
        if(event == ButtonEvent::EVENT_ButtonPress)
        {
					 mtm.TaskStateCtrl(TP_InfoLED, 1);
					 BMS.enableDischarging();
	         BMS.enableCharging();
        }
        if(event == ButtonEvent::EVENT_ButtonLongPressed)
        {
//					 digitalWrite(Power_Pin, LOW);
					 mtm.TaskStateCtrl(TP_InfoLED, 0);
			  	 digitalWrite(Power_Pin, LOW);
	         BMS.disableALLMosfet();
					 BMS.shutdown();
					 Serial.println("power off");
        }
        if(event == ButtonEvent::EVENT_ButtonDoubleClick)
        {
//					digitalWrite(LED_Pin, LOW);
//           mtm.TaskStateCtrl(TP_InfoLED, 0);
//	         BMS.disableALLMosfet();
//					 BMS.shutdown();
        }
    }
    
  
}

void Button_Init()
{
    
    pinMode(BMS_BOOT_PIN, INPUT); 
    btOK.EventAttach(ButtonEvent_Handler);
}



void Task_Button_Update()
{
    btOK.EventMonitor(!digitalRead(BMS_BOOT_PIN));  
}
