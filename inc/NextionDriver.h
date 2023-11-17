#ifndef NextionDriver_h
#define NextionDriver_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "microTuple.h"

union values{
  uint16_t Soc_totVal[3];
  float TotalVoltage;
  float Current1;
  int temp1;
  int temp2;
};

enum DataType {
    INT_TYPE,
    FLOAT_TYPE,
    UINT16_TYPE
};

struct MyValues {
    DataType type;
    values value;
};

using MyData = MicroTuple<unsigned long, unsigned long, unsigned long, unsigned long, int, uint16_t, uint16_t>;


class NextionDriver {
public:
  NextionDriver();
  ~NextionDriver();

  const int screens[7] = {4, 6, 12, 14, 79, 19, 20};

  const char* nextionTextP(int arrayNum);
  const char* nextionTextT(int arrayNum);
  void sendChargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time);
  void sendDischargehargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time);
  void writeNextionScreen2(float *cellFrame, uint16_t *Soc_totVal, float TotalVoltage, float Current1, int temp1, int temp2);
  void reWritePage1(unsigned long chargeCurrent, unsigned long chargeTime, unsigned long dischargeCurrent, unsigned long dischargeTime, int totalTime, uint16_t seriesBattery, uint16_t seriesBatterySet, float* Current, int* Time);
  void resetNextionScreen();
  void startCommand();
  void endCommand();
  
  

};

#endif