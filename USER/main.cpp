#include "FileGroup.h"
#define BMS_I2C_ADDRESS 0x08

bq769x0 BMS(bq76940, BMS_I2C_ADDRESS);    // battery management system object

MillisTaskManager mtm(TP_MAX, true);

void setup()
{   
		pinMode(Power_Pin, OUTPUT);
	  digitalWrite(Power_Pin, HIGH);
	  Serial.begin(115200);
		Button_Init();
	  LED_Init();
	  BMS_Init();

    mtm.TaskRegister(TP_Button_Update, Task_Button_Update, 10);
    mtm.TaskRegister(TP_InfoLED, Task_InfoLED,1000,0);
	  mtm.TaskRegister(TP_BMS_Update, Task_BMS_Update, 250);
	 
}

void loop()
{
   mtm.Running(millis());
}
/**
  * @brief  Main Function
  * @param  None
  * @retval None
  */
int main(void)
{
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    GPIO_JTAG_Disable();
    Delay_Init();
    //ADCx_Init(ADC1);
    setup();
    for(;;)loop();
}

//void I2C_Scan(bool startScan)
//{
//    //DEBUG_FUNC_LOG();
//	    Wire.begin();
//    if(!startScan)
//        return;
//    
//    uint8_t error, address;
//    int nDevices;
//    
//    Serial.println("I2C device scanning...");

//    nDevices = 0;
//    for (address = 1; address < 127; address++ )
//    {
//        // The i2c_scanner uses the return value of
//        // the Write.endTransmisstion to see if
//        // a device did acknowledge to the address.
//        Wire.beginTransmission(address);
//        error = Wire.endTransmission();

//        if (error == 0)
//        {
//            Serial.print("I2C device found at address 0x");
//            if (address < 16)
//                Serial.print("0");
//            Serial.print(address, HEX);
//            Serial.println(" !");

//            nDevices++;
//        }
//        else if (error == 4)
//        {
//            Serial.print("Unknow error at address 0x");
//            if (address < 16)
//                Serial.print("0");
//            Serial.println(address, HEX);
//        }
//    }

//    Serial.printf("%d I2C devices was found\r\n", nDevices);
//}
