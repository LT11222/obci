#include <SPI.h>
#include <SdFatUtil.h>
#include <_OpenBCI_03.h>

//------------------------------------------------------------------------------
//	<< OpenBCI BUSINESS >>
_OpenBCI_03 OBCI; //Uses SPI bus and pins to say data is ready.	Uses Pins 13,12,11,10,9,8,4
#define MAX_N_CHANNELS (8)	//must be less than or equal to length of channelData in ADS1299 object!!
int nActiveChannels = 8;	 //how many active channels would I like?
byte gainCode = ADS_GAIN24;	 //how much gain do I want
byte inputType = ADSINPUT_NORMAL;	 //here's the normal way to setup the channels
boolean startBecauseOfSerial = false;

//Design filters  (This BIQUAD class requires ~6K of program space!  Ouch.)
//For frequency response of these filters: http://www.earlevel.com/main/2010/12/20/biquad-calculator/
#include <Biquad_multiChan.h>   //modified from this source code:  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
#define SAMPLE_RATE_HZ (250.0)  //default setting for OpenBCI
#define FILTER_Q (0.5)        //critically damped is 0.707 (Butterworth)
#define FILTER_PEAK_GAIN_DB (0.0) //we don't want any gain in the passband
#define HP_CUTOFF_HZ (0.5)  //set the desired cutoff for the highpass filter
Biquad_multiChan stopDC_filter(MAX_N_CHANNELS,bq_type_highpass,HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
//Biquad_multiChan stopDC_filter(MAX_N_CHANNELS,bq_type_bandpass,10.0 / SAMPLE_RATE_HZ, 6.0, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
#define NOTCH_FREQ_HZ (60.0)
#define NOTCH_Q (4.0)              //pretty sharp notch
#define NOTCH_PEAK_GAIN_DB (0.0)  //doesn't matter for this filter type
Biquad_multiChan notch_filter1(MAX_N_CHANNELS,bq_type_notch,NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, NOTCH_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan notch_filter2(MAX_N_CHANNELS,bq_type_notch,NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, NOTCH_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
boolean useFilters = false;  //enable or disable as you'd like...turn off if you're daisy chaining!


uint16_t sampleCounter = 0;

byte printval = 0;

int outputType;

ArduinoOutStream cout(Serial);

boolean is_running = true;

void setup(void) {
	Serial.begin(115200);
	pinMode(LIS3DH_SS,OUTPUT);
	digitalWrite(LIS3DH_SS,HIGH);	 // de-select the LIS3DH
	pinMode(SD_SS,OUTPUT);
	digitalWrite(SD_SS,HIGH);			 // de-select the SD card

	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	
	cout << ("starting ");
	OBCI.initialize();

	SPI.setDataMode(SPI_MODE1);
	for (int chan=1; chan <= nActiveChannels; chan++) {
		OBCI.activateChannel(chan, gainCode, inputType);
	}
	OBCI.printAllRegisters(); //print state of all registers
	
	SPI.setDataMode(SPI_MODE1);
	
	OBCI.start();
	
}

void loop() {
	while (Serial.read() >= 0) {}	// clear out the serial buffer
	
	if(is_running) {
	
		while(!(OBCI.isDataAvailable())){	 // watch the DRDY pin
			delayMicroseconds(100);
		}
		
		SPI.setDataMode(SPI_MODE1);
		OBCI.updateChannelData();
		SPI.setDataMode(SPI_MODE0);
		sampleCounter++;
		
		if (useFilters) applyFilters();
		/*
		SPI.setDataMode(SPI_MODE1);
		OBCI.writeChannelDataAsBinary(MAX_N_CHANNELS,sampleCounter);
		SPI.setDataMode(SPI_MODE0);
		*/
		
		//print the data
		//OBCI.writeChannelDataAsBinary(MAX_N_CHANNELS,sampleCounter);  //print all channels, whether active or not
		OBCI.printChannelDataAsText(MAX_N_CHANNELS,sampleCounter);  //print all channels, whether active or not

		/*
		for (int currentChannel = 0; currentChannel < 8; currentChannel++){
			for (int currentNibble = 5; currentNibble >= 0; currentNibble--){
				byte nibble = (OBCI.channelData[currentChannel] >> currentNibble*4) & 0x0F;
				nibble = toHex(nibble);
				printval = nibble;
				cout << bin << printval;
			}
				
			if(currentChannel < 7){
				cout << pstr(",");
			}
		}
		
		cout << endl;
		*/
	
	}
	
}

byte toHex(byte in){
	if (in > 9){
		in += 55;	// convert to ASCII A-F
	}
	else{
		in += 48;	// convert to ASCII 0-9
	}
	return in;
}



void serialEvent(){            // send an 'x' on the serial line to trigger ADStest()
  while(Serial.available()){      
    char inChar = (char)Serial.read();
    switch (inChar)
    {
      //turn channels on and off
      case '1':
        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
      case '2':
        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
      case '3':
        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
      case '4':
        changeChannelState_maintainRunningState(4,DEACTIVATE); break;
      case '5':
        changeChannelState_maintainRunningState(5,DEACTIVATE); break;
      case '6':
        changeChannelState_maintainRunningState(6,DEACTIVATE); break;
      case '7':
        changeChannelState_maintainRunningState(7,DEACTIVATE); break;
      case '8':
        changeChannelState_maintainRunningState(8,DEACTIVATE); break;
      case 'q':
        changeChannelState_maintainRunningState(1,ACTIVATE); break;
      case 'w':
        changeChannelState_maintainRunningState(2,ACTIVATE); break;
      case 'e':
        changeChannelState_maintainRunningState(3,ACTIVATE); break;
      case 'r':
        changeChannelState_maintainRunningState(4,ACTIVATE); break;
      case 't':
        changeChannelState_maintainRunningState(5,ACTIVATE); break;
      case 'y':
        changeChannelState_maintainRunningState(6,ACTIVATE); break;
      case 'u':
        changeChannelState_maintainRunningState(7,ACTIVATE); break;
      case 'i':
        changeChannelState_maintainRunningState(8,ACTIVATE); break;
        
      //turn lead-off detection on and off
      case '!':
        changeChannelLeadOffDetection_maintainRunningState(1,ACTIVATE,PCHAN); break;
      case '@':
        changeChannelLeadOffDetection_maintainRunningState(2,ACTIVATE,PCHAN); break;
      case '#':
        changeChannelLeadOffDetection_maintainRunningState(3,ACTIVATE,PCHAN); break;
      case '$':
        changeChannelLeadOffDetection_maintainRunningState(4,ACTIVATE,PCHAN); break;
      case '%':
        changeChannelLeadOffDetection_maintainRunningState(5,ACTIVATE,PCHAN); break;
      case '^':
        changeChannelLeadOffDetection_maintainRunningState(6,ACTIVATE,PCHAN); break;
      case '&':
        changeChannelLeadOffDetection_maintainRunningState(7,ACTIVATE,PCHAN); break;
      case '*':
        changeChannelLeadOffDetection_maintainRunningState(8,ACTIVATE,PCHAN); break;
      case 'Q':
        changeChannelLeadOffDetection_maintainRunningState(1,DEACTIVATE,PCHAN); break;
      case 'W':
        changeChannelLeadOffDetection_maintainRunningState(2,DEACTIVATE,PCHAN); break;
      case 'E':
        changeChannelLeadOffDetection_maintainRunningState(3,DEACTIVATE,PCHAN); break;
      case 'R':
        changeChannelLeadOffDetection_maintainRunningState(4,DEACTIVATE,PCHAN); break;
      case 'T':
        changeChannelLeadOffDetection_maintainRunningState(5,DEACTIVATE,PCHAN); break;
      case 'Y':
        changeChannelLeadOffDetection_maintainRunningState(6,DEACTIVATE,PCHAN); break;
      case 'U':
        changeChannelLeadOffDetection_maintainRunningState(7,DEACTIVATE,PCHAN); break;
      case 'I':
        changeChannelLeadOffDetection_maintainRunningState(8,DEACTIVATE,PCHAN); break;
       case 'A':
        changeChannelLeadOffDetection_maintainRunningState(1,ACTIVATE,NCHAN); break;
      case 'S':
        changeChannelLeadOffDetection_maintainRunningState(2,ACTIVATE,NCHAN); break;
      case 'D':
        changeChannelLeadOffDetection_maintainRunningState(3,ACTIVATE,NCHAN); break;
      case 'F':
        changeChannelLeadOffDetection_maintainRunningState(4,ACTIVATE,NCHAN); break;
      case 'G':
        changeChannelLeadOffDetection_maintainRunningState(5,ACTIVATE,NCHAN); break;
      case 'H':
        changeChannelLeadOffDetection_maintainRunningState(6,ACTIVATE,NCHAN); break;
      case 'J':
        changeChannelLeadOffDetection_maintainRunningState(7,ACTIVATE,NCHAN); break;
      case 'K':
        changeChannelLeadOffDetection_maintainRunningState(8,ACTIVATE,NCHAN); break;
      case 'Z':
        changeChannelLeadOffDetection_maintainRunningState(1,DEACTIVATE,NCHAN); break;
      case 'X':
        changeChannelLeadOffDetection_maintainRunningState(2,DEACTIVATE,NCHAN); break;
      case 'C':
        changeChannelLeadOffDetection_maintainRunningState(3,DEACTIVATE,NCHAN); break;
      case 'V':
        changeChannelLeadOffDetection_maintainRunningState(4,DEACTIVATE,NCHAN); break;
      case 'B':
        changeChannelLeadOffDetection_maintainRunningState(5,DEACTIVATE,NCHAN); break;
      case 'N':
        changeChannelLeadOffDetection_maintainRunningState(6,DEACTIVATE,NCHAN); break;
      case 'M':
        changeChannelLeadOffDetection_maintainRunningState(7,DEACTIVATE,NCHAN); break;
      case '<':
        changeChannelLeadOffDetection_maintainRunningState(8,DEACTIVATE,NCHAN); break; 
      /*  
      //control the bias generation
      case '`':
        OBCI.setAutoBiasGeneration(true); break;
      case '~': 
        OBCI.setAutoBiasGeneration(false); break; 
      */  
      //control test signals
      case '0':
        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
      case '-':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
      case '+':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
      case '=':
        //repeat the line above...just for human convenience
        activateAllChannelsToTestCondition(ADSINPUT_NORMAL,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); 
		OBCI.setSRB1(true);
		break;
      case 'p':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
      case '[':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
      case ']':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;
        
      //other commands
      case 'n':
        toggleRunState(OUTPUT_BINARY_WITH_AUX);
        startBecauseOfSerial = is_running;
        if (is_running) Serial.println(F("Arduino: Starting binary (including AUX value)..."));
        break;
      case 'b':
        toggleRunState(OUTPUT_BINARY);
        //toggleRunState(OUTPUT_BINARY_SYNTHETIC);
        startBecauseOfSerial = is_running;
        if (is_running) Serial.println(F("Arduino: Starting binary..."));
        break;
      case 'v':
        toggleRunState(OUTPUT_BINARY_4CHAN);
        startBecauseOfSerial = is_running;
        if (is_running) Serial.println(F("Arduino: Starting binary 4-chan..."));
        break;
     case 's':
        stopRunning();
        startBecauseOfSerial = is_running;
        break;
     case 'x':
        toggleRunState(OUTPUT_TEXT);
        startBecauseOfSerial = is_running;
        if (is_running) Serial.println(F("Arduino: Starting text..."));
        break;
     case 'f':
        useFilters = true;
        Serial.println(F("Arduino: enabaling filters"));
        break;
     case 'g':
        useFilters = false;
        Serial.println(F("Arduino: disabling filters"));
        break;
     case '?':
        //print state of all registers
        OBCI.printAllRegisters();
        break;
      default:
        break;
    }
  }
}

boolean toggleRunState(int OUT_TYPE)
{
  if (is_running) {
    return stopRunning();
  } else {
    return startRunning(OUT_TYPE);
  }
}

boolean stopRunning(void) {
  SPI.setDataMode(SPI_MODE1);
  OBCI.stop();                    // stop the data acquisition
  is_running = false;
  return is_running;
}

boolean startRunning(int OUT_TYPE) {
    outputType = OUT_TYPE;
	SPI.setDataMode(SPI_MODE1);
    OBCI.start();    //start the data acquisition
    is_running = true;
    return is_running;
}

int changeChannelState_maintainRunningState(int chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    Serial.print(F("Activating channel "));
    Serial.println(chan);
    OBCI.activateChannel(chan,gainCode,inputType);
  } else {
    Serial.print(F("Deactivating channel "));
    Serial.println(chan);
    OBCI.deactivateChannel(chan);
  }
  
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

int activateAllChannelsToTestCondition(int testInputCode, byte amplitudeCode, byte freqCode)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //set the test signal to the desired state
  OBCI.configureInternalTestSignal(amplitudeCode,freqCode);
  
  //must stop running to change channel settings
  stopRunning();
    
  //loop over all channels to change their state
  for (int Ichan=1; Ichan <= 8; Ichan++) {
    OBCI.activateChannel(Ichan,gainCode,testInputCode);  //Ichan must be [1 8]...it does not start counting from zero
  }
      
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

int changeChannelLeadOffDetection_maintainRunningState(int chan, int start, int code_P_N_Both)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    Serial.print(F("Activating channel "));
    Serial.print(chan);
    Serial.println(F(" Lead-Off Detection"));
    OBCI.changeChannelLeadOffDetection(chan,ON,code_P_N_Both);
  } else {
    Serial.print(F("Deactivating channel "));
    Serial.print(chan);
    Serial.println(F(" Lead-Off Detection"));
    OBCI.changeChannelLeadOffDetection(chan,OFF,code_P_N_Both);
  }
  
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

long int runningAve[MAX_N_CHANNELS];
int applyFilters(void) {
  //scale factor for these coefficients was 32768 = 2^15
  const static long int a0 = 32360L; //16 bit shift?
  const static long int a1 = -2L*a0;
  const static long int a2 = a0;
  const static long int b1 = -64718L; //this is a shift of 17 bits!
  const static long int b2 = 31955L;
  static long int z1[MAX_N_CHANNELS], z2[MAX_N_CHANNELS];
  long int val_int, val_in_down9, val_out, val_out_down9;
  float val;
  for (int Ichan=0; Ichan < MAX_N_CHANNELS; Ichan++) {
    switch (1) {
      case 1:
        //use BiQuad
        val = (float) OBCI.channelData[Ichan]; //get the stored value for this sample
        val = stopDC_filter.process(val,Ichan);    //apply DC-blocking filter
        break;
      case 2:
        //do fixed point, 1st order running ave
        val_int = OBCI.channelData[Ichan]; //get the stored value for this sample
        //runningAve[Ichan]=( ((512-1)*(runningAve[Ichan]>>2)) + (val_int>>2) )>>7;  // fs/0.5Hz = ~512 points..9 bits
        //runningAve[Ichan]=( ((256-1)*(runningAve[Ichan]>>2)) + (val_int>>2) )>>6;  // fs/1.0Hz = ~256 points...8 bits
        runningAve[Ichan]=( ((128-1)*(runningAve[Ichan]>>1)) + (val_int>>1) )>>6;  // fs/2.0Hz = ~128 points...7 bits
        val = (float)(val_int - runningAve[Ichan]);  //remove the DC
        break;
//      case 3:
//        val_in_down9 = OBCI.channelData[Ichan] >> 9; //get the stored value for this sample...bring 24-bit value down to 16-bit
//        val_out = (val_in_down9 * a0  + (z1[Ichan]>>9)) >> (16-9);  //8bits were already removed...results in 24-bit value
//        val_out_down9 = val_out >> 9;  //remove eight bits to go from 24-bit down to 16 bit
//        z1[Ichan] = (val_in_down9 * a1 + (z2[Ichan] >> 9) - b1 * val_out_down9  ) >> (16-9);  //8-bits were pre-removed..end in 24 bit number
//        z2[Ichan] = (val_in_down9 * a2  - b2 * val_out_down9) >> (16-9); //8-bits were pre-removed...end in 24-bit number
//        val = (float)val_out;
//        break;
    }
    val = notch_filter1.process(val,Ichan);     //apply 60Hz notch filter
    val = notch_filter2.process(val,Ichan);     //apply it again
    OBCI.channelData[Ichan] = (long) val;  //save the value back into the main data-holding object
  }
  return 0;
}