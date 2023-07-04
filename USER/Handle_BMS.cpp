#include "FileGroup.h"

TwoWire Wire(SCL_Pin, SDA_Pin, SOFT_STANDARD);

void BMS_Init()
{
    int err = BMS.begin(BMS_ALERT_PIN, BMS_BOOT_PIN);
		BMS.setTemperatureLimits(-20, 45, 0, 45);
		BMS.setShuntResistorValue(2);
		BMS.setShortCircuitProtection(14000, 200);  // delay in us
		BMS.setOvercurrentChargeProtection(8000, 200);  // delay in ms
		BMS.setOvercurrentDischargeProtection(8000, 320); // delay in ms
		BMS.setCellUndervoltageProtection(2800, 2); // delay in s
		BMS.setCellOvervoltageProtection(4100, 2);  // delay in s
		BMS.setThermistorBetaValue(3950);  //bk value

		BMS.setBalancingThresholds(0, 3300, 20);  // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
		BMS.setIdleCurrentThreshold(100);
		BMS.enableAutoBalancing();
	
		BMS.enableDischarging();
	  BMS.enableCharging();
}

void Task_BMS_Update()
{
		BMS.update();  // should be called at least every 250 ms

//	  BMS.printRegisters(); 

}

