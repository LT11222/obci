//
//  ADS1299.cpp   ARDUINO LIBRARY FOR COMMUNICATING WITH ADS1299
//  
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer, 2013
//
#include "ADS1299.h"

void ADS1299::initialize(int _DRDY, int _RST, int _CS){
    DRDY = _DRDY;
    CS = _CS;
	int RST = _RST;
	
		delay(50);				// recommended power up sequence requiers Tpor (~32mS)	
		pinMode(RST,OUTPUT);
		pinMode(RST,LOW);
		delayMicroseconds(4);	// toggle reset pin
		pinMode(RST,HIGH);
		delayMicroseconds(20);	// recommended to wait 18 Tclk before using device (~8uS);
	

    // **** ----- SPI Setup ----- **** //
    
    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    // pinMode(SS, OUTPUT);
	
    
    digitalWrite(SCK, LOW);
    digitalWrite(MOSI, LOW);
    // digitalWrite(SS, HIGH);
    
    // **** ----- End of SPI Setup ----- **** //
    
    // initalize the  data ready chip select and reset pins:
    pinMode(DRDY, INPUT);
    pinMode(CS, OUTPUT);
	
	digitalWrite(CS,HIGH); 	
	digitalWrite(RST,HIGH);

	takeSPI();
    reset();
    releaseSPI();
    verbose = false;      // when verbose is true, there will be Serial feedback 

};

void ADS1299::takeSPI(void) {

    // SPI.setDataMode(SPI_MODE1);
    old_SPCR = SPCR;
    old_SPSR = SPSR;

    // int FREQ = SCK_MHZ;   // SCK_MHZ defined as 4 in _OpenBCI_03.h
  // set as master and enable SPI
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);
    // set bit order
    SPCR &= ~(_BV(DORD)); ////SPI data format is MSB (pg. 25)
  // set data mode
    SPCR = (SPCR & ~SPI_MODE_MASK) | SPI_DATA_MODE; //clock polarity = 0; clock phase = 1 (pg. 8)
    // set clock divider
  // switch (FREQ){
  //   case 8:
  //     DIVIDER = SPI_CLOCK_DIV_2;
  //     break;
  //   case 4:
  //     DIVIDER = SPI_CLOCK_DIV_4;
  //     break;
  //   case 1:
  //     DIVIDER = SPI_CLOCK_DIV_16;
  //     break;
  //   default:
  //     break;
  // }
  //   SPCR = (SPCR & ~SPI_CLOCK_MASK) | (DIVIDER);  // set SCK frequency  
  //   SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | (DIVIDER); // by dividing 16MHz system clock
}

void ADS1299::releaseSPI(void){
    // SPI.setDataMode(SPI_MODE0);
    SPCR = old_SPCR;
    SPSR = old_SPSR;
}

void ADS1299::reset(void)
{
  RESET();             // send RESET command to default all registers
  SDATAC();            // exit Read Data Continuous mode to communicate with ADS
  
  delay(100);
    
  // turn off all channels
  for (int chan=1; chan <= 8; chan++) {
    deactivateChannel(chan);
  }

};

//System Commands
void ADS1299::WAKEUP() {
    cslow(); 
    SPI.transfer(_WAKEUP);
    cshigh(); 
    delayMicroseconds(3);  		//must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
}

void ADS1299::STANDBY() {		// only allowed to send WAKEUP after sending STANDBY
    cslow();
    SPI.transfer(_STANDBY);
    cshigh();
}

void ADS1299::RESET() {			// reset all the registers to default settings
    cslow();
    SPI.transfer(_RESET);
    delayMicroseconds(12);   	//must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
    cshigh();
}

void ADS1299::START() {			//start data conversion 
    cslow();
    SPI.transfer(_START);
    cshigh();
}

void ADS1299::STOP() {			//stop data conversion
    cslow();
    SPI.transfer(_STOP);
    cshigh();
}

void ADS1299::RDATAC() {
    cslow();
    SPI.transfer(_RDATAC);
    cshigh();
	delayMicroseconds(3);   
}
void ADS1299::SDATAC() {
    cslow();
    SPI.transfer(_SDATAC);
    cshigh();
	delayMicroseconds(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}


// Register Read/Write Commands
byte ADS1299::getDeviceID() {			// simple hello world com check
	byte data = RREG(0x00);
	if(verbose){						// verbose otuput
		Serial.print("Device ID ");
		printHex(data);	
        Serial.println();
	}
	return data;
}

byte ADS1299::RREG(byte _address) {		//  reads ONE register at _address
    byte opcode1 = _address + 0x20; 	//  RREG expects 001rrrrr where rrrrr = _address
    cslow(); 				//  open SPI
    SPI.transfer(opcode1); 					//  opcode1
    SPI.transfer(0x00); 					//  opcode2
    regData[_address] = SPI.transfer(0x00);//  update mirror location with returned byte
    cshigh(); 			//  close SPI	
	if (verbose){						//  verbose output
		printRegisterName(_address);
		printHex(_address);
		Serial.print(", ");
		printHex(regData[_address]);
		Serial.print(", ");
		for(byte j = 0; j<8; j++){
			Serial.print(bitRead(regData[_address], 7-j));
			if(j!=7) Serial.print(", ");
		}
		
		Serial.println();
	}
	return regData[_address];			// return requested register value
}

// Read more than one register starting at _address
void ADS1299::RREGS(byte _address, byte _numRegistersMinusOne) {
//	for(byte i = 0; i < 0x17; i++){
//		regData[i] = 0;					//  reset the regData array
//	}
    byte opcode1 = _address + 0x20; 	//  RREG expects 001rrrrr where rrrrr = _address
    cslow(); 				//  open SPI
    SPI.transfer(opcode1); 					//  opcode1
    SPI.transfer(_numRegistersMinusOne);	//  opcode2
    for(int i = 0; i <= _numRegistersMinusOne; i++){
        regData[_address + i] = SPI.transfer(0x00); 	//  add register byte to mirror array
		}
    cshigh(); 			//  close SPI
	if(verbose){						//  verbose output
		for(int i = 0; i<= _numRegistersMinusOne; i++){
			printRegisterName(_address + i);
			printHex(_address + i);
			Serial.print(", ");
			printHex(regData[_address + i]);
			Serial.print(", ");
			for(int j = 0; j<8; j++){
				Serial.print(bitRead(regData[_address + i], 7-j));
				if(j!=7) Serial.print(", ");
			}
			Serial.println();
			delay(30);
		}
    }
    
}

void ADS1299::WREG(byte _address, byte _value) {	//  Write ONE register at _address
    byte opcode1 = _address + 0x40; 	//  WREG expects 010rrrrr where rrrrr = _address
    cslow(); 				//  open SPI
    SPI.transfer(opcode1);					//  Send WREG command & address
    SPI.transfer(0x00);						//	Send number of registers to read -1
    SPI.transfer(_value);					//  Write the value to the register
    cshigh(); 			//  close SPI
	regData[_address] = _value;			//  update the mirror array
	if(verbose){						//  verbose output
		Serial.print("Register ");
		printHex(_address);
		Serial.println(" modified.");
	}
}

void ADS1299::WREGS(byte _address, byte _numRegistersMinusOne) {
    byte opcode1 = _address + 0x40;		//  WREG expects 010rrrrr where rrrrr = _address
    cslow(); 				//  open SPI
    SPI.transfer(opcode1);					//  Send WREG command & address
    SPI.transfer(_numRegistersMinusOne);	//	Send number of registers to read -1	
	for (int i=_address; i <=(_address + _numRegistersMinusOne); i++){
		SPI.transfer(regData[i]);			//  Write to the registers
	}	
	digitalWrite(CS,HIGH);				//  close SPI
	if(verbose){
		Serial.print("Registers ");
		printHex(_address); Serial.print(" to ");
		printHex(_address + _numRegistersMinusOne);
		Serial.println(" modified");
	}
}


void ADS1299::updateChannelData(){
	byte inByte;
	cslow();				//  open SPI
	stat = SPI.transfer(0x00);				//  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	stat = SPI.transfer(0x00);				//  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	stat = SPI.transfer(0x00);				//  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	for(int i = 0; i<8; i++){
		for(int j=0; j<3; j++){		//  read 24 bits of channel data in 8 3 byte chunks
			inByte = SPI.transfer(0x00);
			channelData[i] = (channelData[i]<<8) | inByte;
		}
	}
	cshigh();				//  close SPI
	
	for(int i=0; i<8; i++){			// convert 3 byte 2's compliment to 4 byte 2's compliment	
		if(bitRead(channelData[i],23) == 1){	
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
//	if(verbose){
//		Serial.print(stat); Serial.print(", ");
//		for(int i=0; i<8; i++){
//			Serial.print(channelData[i]);
//			if(i<7){Serial.print(", ");}
//		}
//		Serial.println();
//	}
}
	



void ADS1299::RDATA() {					//  use in Stop Read Continuous mode when DRDY goes low
	byte inByte;						//  to read in one sample of the channels
    cslow();				//  open SPI
    SPI.transfer(_RDATA);					//  send the RDATA command
	stat = SPI.transfer(0x00);				//  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	for(int i = 0; i<8; i++){
		for(int j=0; j<3; j++){		//  read in the status register and new channel data
			inByte = SPI.transfer(0x00);
			channelData[i] = (channelData[i]<<8) | inByte;
		}
	}
	cshigh();				//  close SPI
	
	for(int i=0; i<8; i++){
		if(bitRead(channelData[i],23) == 1){	// convert 3 byte 2's compliment to 4 byte 2's compliment
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
    
}


// String-Byte converters for RREG and WREG
void ADS1299::printRegisterName(byte _address) {
    if(_address == ID){
        Serial.print("ID, ");
    }
    else if(_address == CONFIG1){
        Serial.print("CONFIG1, ");
    }
    else if(_address == CONFIG2){
        Serial.print("CONFIG2, ");
    }
    else if(_address == CONFIG3){
        Serial.print("CONFIG3, ");
    }
    else if(_address == LOFF){
        Serial.print("LOFF, ");
    }
    else if(_address == CH1SET){
        Serial.print("CH1SET, ");
    }
    else if(_address == CH2SET){
        Serial.print("CH2SET, ");
    }
    else if(_address == CH3SET){
        Serial.print("CH3SET, ");
    }
    else if(_address == CH4SET){
        Serial.print("CH4SET, ");
    }
    else if(_address == CH5SET){
        Serial.print("CH5SET, ");
    }
    else if(_address == CH6SET){
        Serial.print("CH6SET, ");
    }
    else if(_address == CH7SET){
        Serial.print("CH7SET, ");
    }
    else if(_address == CH8SET){
        Serial.print("CH8SET, ");
    }
    else if(_address == BIAS_SENSP){
        Serial.print("BIAS_SENSP, ");
    }
    else if(_address == BIAS_SENSN){
        Serial.print("BIAS_SENSN, ");
    }
    else if(_address == LOFF_SENSP){
        Serial.print("LOFF_SENSP, ");
    }
    else if(_address == LOFF_SENSN){
        Serial.print("LOFF_SENSN, ");
    }
    else if(_address == LOFF_FLIP){
        Serial.print("LOFF_FLIP, ");
    }
    else if(_address == LOFF_STATP){
        Serial.print("LOFF_STATP, ");
    }
    else if(_address == LOFF_STATN){
        Serial.print("LOFF_STATN, ");
    }
    else if(_address == GPIO){
        Serial.print("GPIO, ");
    }
    else if(_address == MISC1){
        Serial.print("MISC1, ");
    }
    else if(_address == MISC2){
        Serial.print("MISC2, ");
    }
    else if(_address == CONFIG4){
        Serial.print("CONFIG4, ");
    }
}

// Used for printing HEX in verbose feedback mode
void ADS1299::printHex(byte _data){
	Serial.print("0x");
    if(_data < 0x10) Serial.print("0");
    Serial.print(_data, HEX);
}

//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//

//  deactivate the given channel...note: stops data colleciton to issue its commands
//  N is the channel number: 1-8
// 
void ADS1299::deactivateChannel(int N)
{
  byte reg, config;
	
  		//check the inputs
  if ((N < 1) || (N > 8)) return;
  
  		//proceed...first, disable any data collection
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  		//shut down the channel
  N = constrain(N-1,0,7);  //subtracts 1 so that we're counting from 0, not 1
  reg = CH1SET+(byte)N;						// select the current channel
  config = RREG(reg); delay(1);	// get the current channel settings
  bitSet(config,7);  						// set bit7 to shut down channel
  if (use_N_inputs) bitClear(config,3);  	// clear bit3 to disclude from SRB2 if used
  WREG(reg,config); delay(1);	    // write the new value to disable the channel
  
  		//remove the channel from the bias generation...
  reg = BIAS_SENSP; 					 	// set up to disconnect the P inputs from Bias generation
  config = RREG(reg); delay(1);	//get the current bias settings
  bitClear(config,N);          				//clear this channel's bit to remove from bias generation
  WREG(reg,config); delay(1); 	//send the modified byte back to the ADS

  reg = BIAS_SENSN; 						// set up to disconnect the N inputs from Bias generation
  config = RREG(reg); delay(1);	//get the current bias settings
  bitClear(config,N);          				//clear this channel's bit to remove from bias generation
  WREG(reg,config); delay(1);  	//send the modified byte back to the ADS
  
}; 
    
        
//Active a channel in single-ended mode  
//  N is 1 through 8
//  gainCode is defined in the macros in the header file
//  inputCode is defined in the macros in the header file
void ADS1299::activateChannel(int N,byte gainCode,byte inputCode) 
{
	byte reg, config;
   //check the inputs
  if ((N < 1) || (N > 8)) return;
  
  //proceed...first, disable any data collection
  SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  //active the channel using the given gain.  Set MUX for normal operation
  //see ADS1299 datasheet, PDF p44
  N = constrain(N-1,0,7);  //shift down by one
  config = 0b00000000;  						//left-most zero (bit 7) is to activate the channel
  gainCode = gainCode & 0b01110000;  			//bitwise AND to get just the bits we want and set the rest to zero
  config = config | gainCode; 					//bitwise OR to set just the gain bits high or low and leave the rest
  inputCode = inputCode & 0b00000111;  		//bitwise AND to get just the bits we want and set the rest to zero
  config = config | inputCode; 					//bitwise OR to set just the gain bits high or low and leave the rest

  if (use_SRB2[N]) config |= 0b00001000;  	//set the SRB2 flag if you plan to use it
  ADS1299::WREG(CH1SET+(byte)N,config); delay(1);

  //add this channel to the bias generation
  //see ADS1299 datasheet, PDF p44
  reg = BIAS_SENSP; 					// set up to connect the P inputs for bias generation
  config = ADS1299::RREG(reg); 			//get the current bias settings
  bitSet(config,N);                   	//set this channel's bit to add it to the bias generation
  ADS1299::WREG(reg,config); delay(1); //send the modified byte back to the ADS

  reg = BIAS_SENSN;  					// set up to connect the N input for bias generation
  config = ADS1299::RREG(reg); 			//get the current bias settings
  bitSet(config,N);                   	//set this channel's bit to add it to the bias generation
  ADS1299::WREG(reg,config); delay(1); //send the modified byte back to the ADS
  
  // // Now, these actions are necessary whenever there is at least one active channel
  // // though they don't strictly need to be done EVERY time we activate a channel.
  // // just once after the reset.
  
  //activate SRB1 as the Negative input for all channels, if needed
  setSRB1(use_SRB1());

  //Finalize the bias setup...activate buffer and use internal reference for center of bias creation, datasheet PDF p42
  ADS1299::WREG(CONFIG3,0b11101100); delay(1);   // THIS COULD MOVE TO THE INITIALIZATION
};


// USE THIS METHOD IF YOU WANT TO CONTROL THE SIGNAL INCLUSION IN SRB AND BIAS GENERATION
void ADS1299::activateChannel(int N,byte gainCode,byte inputCode,boolean useInBias) 
{
	byte reg, config;
   //check the inputs
  if ((N < 1) || (N > 8)) return;
  
  //proceed...first, disable any data collection
  ADS1299::SDATAC(); delay(1);      // exit Read Data Continuous mode to communicate with ADS

  //active the channel using the given gain.  Set MUX for normal operation
  //see ADS1299 datasheet, PDF p44
  N = constrain(N-1,0,7);  //shift down by one
  config = 0b00000000;  						//left-most zero (bit 7) is to activate the channel
  gainCode = gainCode & 0b01110000;  			//bitwise AND to get just the bits we want and set the rest to zero
  config = config | gainCode; 					//bitwise OR to set just the gain bits high or low and leave the rest
  inputCode = inputCode & 0b00000111;  		//bitwise AND to get just the bits we want and set the rest to zero
  config = config | inputCode; 					//bitwise OR to set just the gain bits high or low and leave the rest
  if ((use_SRB2[N]) && useInBias)config |= 0b00001000;  	//set the SRB2 flag if you plan to use it
  ADS1299::WREG(CH1SET+(byte)N,config); delay(1);

  //add this channel to the bias generation
  //see ADS1299 datasheet, PDF p44
if(useInBias){
  reg = BIAS_SENSP; 					// set up to connect the P inputs for bias generation
  config = ADS1299::RREG(reg); 			//get the current bias settings
  bitSet(config,N);                   	//set this channel's bit to add it to the bias generation
  ADS1299::WREG(reg,config); delay(1); //send the modified byte back to the ADS

  reg = BIAS_SENSN;  					// set up to connect the N input for bias generation
  config = ADS1299::RREG(reg); 			//get the current bias settings
  bitSet(config,N);                   	//set this channel's bit to add it to the bias generation
  ADS1299::WREG(reg,config); delay(1); //send the modified byte back to the ADS
}
  
  // // Now, these actions are necessary whenever there is at least one active channel
  // // though they don't strictly need to be done EVERY time we activate a channel.
  // // just once after the reset.
  
  //activate SRB1 as the Negative input for all channels, if needed
  setSRB1(use_SRB1());

  //Finalize the bias setup...activate buffer and use internal reference for center of bias creation, datasheet PDF p42
  ADS1299::WREG(CONFIG3,0b11101100); delay(1);   // THIS COULD MOVE TO THE INITIALIZATION
};

// SRB1 is used when connecting to the P channel inputs
void ADS1299::setSRB1(boolean desired_state) {
	if (desired_state) {
		ADS1299::WREG(MISC1,0b00100000); delay(1);  //ADS1299 datasheet, PDF p46
	} else {
		ADS1299::WREG(MISC1,0b00000000); delay(1);  //ADS1299 datasheet, PDF p46
	}
}


//Configure the test signals that can be inernally generated by the ADS1299
void ADS1299::configureInternalTestSignal(byte amplitudeCode, byte freqCode)
{
	if (amplitudeCode == ADSTESTSIG_NOCHANGE) amplitudeCode = (ADS1299::RREG(CONFIG2) & (0b00000100));
	if (freqCode == ADSTESTSIG_NOCHANGE) freqCode = (ADS1299::RREG(CONFIG2) & (0b00000011));
	freqCode &= 0b00000011;  		//only the last two bits are used
	amplitudeCode &= 0b00000100;  	//only this bit is used
	byte message = 0b11010000 | freqCode | amplitudeCode;  //compose the code
	
	ADS1299::WREG(CONFIG2,message); delay(1);
	
}
 
// Start continuous data acquisition
void ADS1299::start(void)
{
    ADS1299::RDATAC(); 
	delay(1);   // enter Read Data Continuous mode
    ADS1299::START();   			// start the data acquisition
	delay(1);
    isRunning = true;
}
  
// Query to see if data is available from the ADS1299...return TRUE is data is available
int ADS1299::isDataAvailable(void)
{
  return (!(digitalRead(PIN_DRDY)));
}
  
// Stop the continuous data acquisition
void ADS1299::stop(void)
{
    ADS1299::STOP(); delay(1);   		// start the data acquisition
    ADS1299::SDATAC(); delay(1);      	// exit Read Data Continuous mode to communicate with ADS
    isRunning = false;
}
  
void ADS1299::printAllRegisters(void)   
{
    boolean wasRunning = false;
		boolean prevVerboseState = verbose;
		if (isRunning){ stop(); wasRunning = true; }
        verbose = true;						// set up for verbose output
        RREGS(0x00,0x10);     	// read out the first registers
        delay(100);  						// stall to let all that data get read by the PC
        RREGS(0x11,0x17-0x11);	// read out the rest
        verbose = prevVerboseState;
		if (wasRunning){ start(); }
}

void ADS1299::cslow(void)
{
	SPI.setDataMode(SPI_MODE1);
	digitalWrite(CS, LOW);
}

void ADS1299::cshigh(void)
{
	digitalWrite(CS, HIGH);
	SPI.setDataMode(SPI_MODE0);
}