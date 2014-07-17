//
//  ADS1299.h
//  Part of the Arduino Library
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer 2013.
//

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include "pins_arduino.h"
#include "Definitions.h"

#ifndef ____ADS1299__
#define ____ADS1299__

class ADS1299 {
public:
    
    void initialize(int _DRDY, int _RST, int _CS);
    
    //ADS1299 SPI Command Definitions (Datasheet, p35)
    //System Commands
    void WAKEUP();
    void STANDBY();
    void RESET();
    void START();
    void STOP();
    
    //Data Read Commands
    void RDATAC();
    void SDATAC();
    void RDATA();
	
	void reset(void);
	
	void takeSPI(void);                   //grab the SPI bus and change settings to work with ADS preferences
    void releaseSPI(void);                //put the SPI bus back the way you found it and release control
    
    //Register Read/Write Commands
    byte getDeviceID();
    byte RREG(byte _address);
    void RREGS(byte _address, byte _numRegistersMinusOne);     
    void printRegisterName(byte _address);
    void WREG(byte _address, byte _value); 
    void WREGS(byte _address, byte _numRegistersMinusOne); 
    void printHex(byte _data);
    void updateChannelData();
	void setSRB1(boolean desired_state);

    int DRDY, CS; 		// pin numbers for DRDY and CS 
    int DIVIDER;		// select SPI SCK frequency
    int stat;			// used to hold the status register
    byte regData [24];	// array is used to mirror register data
    long channelData [9];	// array used when reading channel data
    boolean verbose;		// turn on/off Serial feedback
	
	void activateChannel(int N, byte gainCode,byte inputCode); //setup the channel 1-8
	void activateChannel(int, byte, byte, boolean);
    void deactivateChannel(int N);                            //disable given channel 1-8
	
    void configureInternalTestSignal(byte amplitudeCode, byte freqCode);
	
    void start(void);
    void stop(void);
	
    int isDataAvailable(void);
	
	void printAllRegisters(void);
	
	void cslow();
	void cshigh();
	
private:
	boolean use_N_inputs;
    boolean use_SRB2[OPENBCI_NCHAN];
    boolean isRunning;
    boolean use_SRB1(void);
	byte old_SPCR;
    byte old_SPSR;

};

#endif