
//
//  _OpenBCI_03.h
//  Part of the Arduino Library for the ADS1299 Shield
//  Created by Chip Audette, Fall 2013
//	Modified by Joel Murphy, Winter 2014
//


#ifndef _____OpenBCI_04__
#define _____OpenBCI_04__

#include <SdFat.h>
#include <SdFatUtil.h>

#include "ADS1299.h"
#include "LIS3DH.h"

/*   Arduino Uno - Pin Assignments
  SCK = 13
  MISO [DOUT] = 12
  MOSI [DIN] = 11
  CS = 10; 
  RESET = 9;
  DRDY = 8;
*/

class OpenBCI {

  public:
  
    ADS1299 ads;
	LIS3DH accel;
	SdFat card;
  
    //initialization functions
    void initialize_ads(void);
	void initialize_accel(void);
	void printAllRegisters(void);  
	
	//board mode functions
    void printChannelDataAsText(int N, long int sampleNumber);
    void writeChannelDataAsBinary(int N, uint16_t sampleNumber);
    void writeChannelDataAsOpenEEG_P2(long int sampleNumber);
    
	void setSRB1(boolean desired_state);
    
	void printDeviceID(void);

    
};

#endif