#ifndef NextionDriver_h
#define NextionDriver_h

#include <Arduino.h>
#include <SoftwareSerial.h>



class NextionDriver {
public:
  NextionDriver();
  ~NextionDriver();

  const char* nextionText(int arrayNum);
  void sendChargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time);
  void sendDischargehargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time);
  void startCommand();
  void endCommand();
};

#endif