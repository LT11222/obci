#include "OpenBCI_04.h"

typedef long int32;
//typedef byte uint8_t;

void OpenBCI::initialize_ads(void) {
  ads.initialize(PIN_DRDY, PIN_RST, _CS1);
}

void OpenBCI::initialize_accel(void) {
  accel.initialize();
}

void OpenBCI::printAllRegisters(void) {
	ads.printAllRegisters();
	accel.readAllRegs();
}