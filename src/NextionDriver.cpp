#include "..\inc\NextionDriver.h"
#include <string.h>

using namespace std;
NextionDriver::NextionDriver() {
    
}

NextionDriver::~NextionDriver() {
    
}

const char* NextionDriver::nextionText(int arrayNum) {
    static String str;
    str = "p" + String(arrayNum) + ".txt=";
    return str.c_str();
}


void NextionDriver::sendChargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time) {

    startCommand();
    String str;
    str = nextionText(Sayici != 18 ? firstRow + Sayici : 83);
    Serial.println(nextionText(Sayici != 18 ? firstRow + Sayici : 83));
    Serial1.print(str);
    Serial1.write(0x22);
    Serial1.print("C");
    endCommand();

    Serial1.print(nextionText(Sayici != 18 ? secRow + Sayici : 81));
    Serial1.write(0x22);
    Serial1.print(Current[Sayici]);
    endCommand();

    if(Sayici < 18)
        Serial1.print(nextionText(Sayici >= 4 ? (thirdRow + Sayici + 18) : thirdRow + Sayici));
    else
        Serial1.print(nextionText(82));
    Serial1.write(0x22);
    Serial1.print(Time[Sayici]);
    endCommand();
}

void NextionDriver::sendDischargehargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time)
{

    startCommand();

    Serial1.print(nextionText(Sayici != 18 ? firstRow + Sayici : 83));
    Serial1.write("\x22");
    Serial1.print("D");
    endCommand();

    Serial1.print(nextionText(Sayici != 18 ? secRow + Sayici : 81));
    Serial1.write("\x22");
    Serial1.print(Current[Sayici]);
    endCommand();

    if (Sayici == 18)
        Serial1.print(nextionText(82));

    Serial1.print(nextionText(Sayici >= 4 ? (thirdRow + Sayici + 18) : thirdRow + Sayici));
    Serial1.write("\x22");
    Serial1.print(Time[Sayici]);
    endCommand();
}

void NextionDriver :: startCommand()
{
      Serial1.write(0x22);
      Serial1.write(0x22);
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
}

void NextionDriver :: endCommand()
{
      Serial1.write(0x22);
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
}
