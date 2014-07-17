#include "LIS3DH.h"

void LIS3DH::initialize(){
    SPI.setDataMode(SPI_MODE3);  
    LIS3DH_write(CTRL_REG1, 0x47);
    // set CTRL_REG4 to 0x80 for block update; 0x4x to set data direction
    LIS3DH_write(CTRL_REG4, 0xA8); // bits 5:4 determine resolution, 3 sets high resolution
    LIS3DH_write(TMP_CFG_REG, 0x40);  // enable temperature sensor
    SPI.setDataMode(SPI_MODE0);
}

byte LIS3DH::LIS3DH_read(byte reg){
	reg |= READ_REG;
	cslow();
	SPI.transfer(reg);
	byte inByte = SPI.transfer(0x00);
	cshigh();
	return inByte;
}


void LIS3DH::LIS3DH_write(byte reg, byte value){
	cslow();
	SPI.transfer(reg);
	SPI.transfer(value);
	cshigh();
}

int LIS3DH::LIS3DH_read16(byte reg){
	int inData;
//	byte r = reg | READ_REG | READ_MULTI;
	reg |= READ_REG | READ_MULTI;
	cslow();	
	SPI.transfer(reg);
	inData = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
	cshigh();
	return inData;
}

int LIS3DH::getX(){
	return LIS3DH_read16(OUT_X_L);
}

int LIS3DH::getY(){
	return LIS3DH_read16(OUT_Y_L);
}

int LIS3DH::getZ(){
	return LIS3DH_read16(OUT_Z_L);
}

void LIS3DH::readAllRegs(){
	Serial.print("LIS3DH Device ID: 0x");Serial.println(LIS3DH_read(0x0F),HEX);
	byte inByte;
	byte reg = STATUS_REG_AUX | READ_REG | READ_MULTI;
	
	cslow();	
	SPI.transfer(reg);
	for (int i = STATUS_REG_AUX; i <= WHO_AM_I; i++){
		inByte = SPI.transfer(0x00);
		Serial.print("0x0");Serial.print(i,HEX);
		Serial.print("\t");Serial.println(inByte,HEX);
		delay(30);
	}
	cshigh();
	
	Serial.println();
	reg = TMP_CFG_REG | READ_REG | READ_MULTI;
	//reg = OUT_X_L | READ_REG | READ_MULTI;
	cslow();	
	SPI.transfer(reg);
	for (int i = TMP_CFG_REG; i <= INT1_DURATION; i++){
	//for (int i = OUT_X_L; i <= INT1_DURATION; i++){
		inByte = SPI.transfer(0x00);
		Serial.print("0x");Serial.print(i,HEX);
		Serial.print("\t");Serial.println(inByte,HEX);
		delay(30);
	}
	cshigh();
	
	Serial.println();
	reg = CLICK_CFG | READ_REG | READ_MULTI;
	cslow();	
	SPI.transfer(reg);
	for (int i = CLICK_CFG; i <= TIME_WINDOW; i++){
		inByte = SPI.transfer(0x00);
		Serial.print("0x");Serial.print(i,HEX);
		Serial.print("\t");Serial.println(inByte,HEX);
		delay(30);
	}
	cshigh();
	
}

void LIS3DH::cslow(void)
{
	SPI.setDataMode(SPI_MODE3);
	digitalWrite(LIS3DH_SS, LOW);
}

void LIS3DH::cshigh(void)
{
	digitalWrite(LIS3DH_SS, HIGH);
	SPI.setDataMode(SPI_MODE0);
}