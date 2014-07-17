#include <EEPROM.h>
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include "OpenBCI_04.h"

OpenBCI board;

//ArduinoOutStream cout(Serial);

void setup(void) {

	Serial.begin(115200);
	pinMode(LIS3DH_SS,OUTPUT);
	digitalWrite(LIS3DH_SS,HIGH);	 // de-select the LIS3DH
	pinMode(SD_SS,OUTPUT);
	digitalWrite(SD_SS,HIGH);			 // de-select the SD card

	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	
	//cout << ("starting ");
	Serial.println("STARTING");
	
	board.initialize_ads();

	board.initialize_accel();
	
	board.printAllRegisters();

}

void loop(void) {

}