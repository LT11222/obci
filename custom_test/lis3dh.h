#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include "pins_arduino.h"
#include "Definitions.h"

#ifndef ____LIS3DH__
#define ____LIS3DH__

class LIS3DH {
public:
	void initialize(void);
	byte LIS3DH_read(byte);
	void LIS3DH_write(byte,byte);
	int LIS3DH_read16(byte);
	int getX(void);
	int getY(void);
	int getZ(void);
	void readAllRegs(void);
	
	void cslow();
	void cshigh();
	
private:

};





/*
SPI.setDataMode(SPI_MODE3);
LIS3DH_write(CTRL_REG5, 0x00);
LIS3DH_write(CTRL_REG1, 0x17);
// set CTRL_REG4 to 0x80 for block update; 0x4x to set data direction
LIS3DH_write(CTRL_REG4, 0x28); // bits 5:4 determine resolution, 3 sets high resolution
LIS3DH_write(TMP_CFG_REG, 0x40);	// enable temperature sensor
LIS3DH_write(CTRL_REG3, 0x00);
LIS3DH_write(INT1_CFG, 0x00);
Serial.print("LIS3DH Device ID: 0x");Serial.println(LIS3DH_read(0x0F),HEX);
SPI.setDataMode(SPI_MODE0);
*/

#endif