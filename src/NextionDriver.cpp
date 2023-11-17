#include "..\inc\NextionDriver.h"
#include <string.h>

using namespace std;
NextionDriver::NextionDriver() {
    
}

NextionDriver::~NextionDriver() {
    
}

const char* NextionDriver::nextionTextP(int arrayNum) {
    static String str;
    str = "p" + String(arrayNum) + ".txt=";
    return str.c_str();
}

const char* NextionDriver::nextionTextT(int arrayNum) {
    static String str;
    str = "t" + String(arrayNum) + ".txt=";
    return str.c_str();
}


void NextionDriver :: sendChargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time) {

    startCommand();
    String str;
    str = nextionTextP(Sayici != 18 ? firstRow + Sayici : 83);
    Serial.println(nextionTextP(Sayici != 18 ? firstRow + Sayici : 83));
    Serial1.print(str);
    Serial1.write(0x22);
    Serial1.print("C");
    endCommand();

    Serial1.print(nextionTextP(Sayici != 18 ? secRow + Sayici : 81));
    Serial1.write(0x22);
    Serial1.print(Current[Sayici]);
    endCommand();

    if(Sayici < 18)
        Serial1.print(nextionTextP(Sayici >= 4 ? (thirdRow + Sayici + 18) : thirdRow + Sayici));
    else
        Serial1.print(nextionTextP(82));
    Serial1.write(0x22);
    Serial1.print(Time[Sayici]);
    endCommand();
}

void NextionDriver :: sendDischargehargeCommand(int firstRow, int secRow, int thirdRow, float* Current, int Sayici, int* Time)
{

    startCommand();

    Serial1.print(nextionTextP(Sayici != 18 ? firstRow + Sayici : 83));
    Serial1.write("\x22");
    Serial1.print("D");
    endCommand();

    Serial1.print(nextionTextP(Sayici != 18 ? secRow + Sayici : 81));
    Serial1.write("\x22");
    Serial1.print(Current[Sayici]);
    endCommand();

    if (Sayici == 18)
        Serial1.print(nextionTextP(82));

    Serial1.print(nextionTextP(Sayici >= 4 ? (thirdRow + Sayici + 18) : thirdRow + Sayici));
    Serial1.write("\x22");
    Serial1.print(Time[Sayici]);
    endCommand();
}

void NextionDriver :: writeNextionScreen2(float *cellFrame, uint16_t *Soc_totVal, float TotalVoltage, float Current1, int temp1, int temp2)
{
    MyValues values;
    int count = 0;
    startCommand();
    for (int i = 71; i < 107; i++)
    {
        if (i == 71 || i == 80 ||i == 89 || i == 98)
        {
            i++;
        }

        Serial1.print(nextionTextT(i));
        Serial1.write(0x22);
        Serial1.print(cellFrame[count]);
        endCommand();
        count++;
    }

    values.type = UINT16_TYPE;
    values.value.Soc_totVal[0] = Soc_totVal[0];
    Serial1.print(nextionTextT(71));
    Serial1.write(0x22);
    Serial1.print(Soc_totVal[0]);
    endCommand();

    values.type = FLOAT_TYPE;
    values.value.TotalVoltage = TotalVoltage;
    Serial1.print(nextionTextT(80));
    Serial1.write(0x22);
    Serial1.print(TotalVoltage);
    endCommand();

    values.type = FLOAT_TYPE;
    values.value.Current1 = Current1;
    Serial1.print(nextionTextT(89));
    Serial1.write(0x22);
    Serial1.print(Current1);
    endCommand();

    values.type = INT_TYPE;
    values.value.temp1 = temp1;
    Serial1.print(nextionTextT(98));
    Serial1.write(0x22);
    Serial1.print(temp1);
    endCommand();

    values.type = INT_TYPE;
    values.value.temp2 = temp2;
    Serial1.print(nextionTextT(107));
    Serial1.write(0x22);
    Serial1.print(temp2);
    endCommand();

}

void NextionDriver :: resetNextionScreen()
{
    startCommand();
    for (int i = 22; i <= 77; i ++)
    {
        if (i == 58)
            i++;
        Serial1.print(nextionTextP(i));
        Serial1.write(0x22);
        endCommand();
    }
}

void NextionDriver :: reWritePage1(unsigned long chargeCurrent, unsigned long chargeTime, unsigned long dischargeCurrent, unsigned long dischargeTime, int totalTime, uint16_t seriesBattery, uint16_t seriesBatterySet, float* current, int* time)
{
    startCommand();
    MyData myDataArray[1];
    myDataArray[0] = {chargeCurrent, chargeTime, dischargeCurrent, dischargeTime, totalTime, seriesBattery, seriesBatterySet};
    int i = 0;
    for (int index : screens)
    {
        Serial1.print(nextionTextP(index));
        Serial1.write(0x22);
        switch (i)
        {
        case 0:
            Serial1.print(myDataArray[0].get<0>());
            endCommand();
            break;
        case 1:
            Serial1.print(myDataArray[0].get<1>());
            endCommand();
            break;
        case 2:
            Serial1.print(myDataArray[0].get<2>());
            endCommand();
            break;
        case 3:
            Serial1.print(myDataArray[0].get<3>());
            endCommand();
            break;
        case 4:
            Serial1.print(myDataArray[0].get<4>());
            endCommand();
            break;
        case 5:
            Serial1.print(myDataArray[0].get<5>());
            endCommand();
            break;
        case 6:
            Serial1.print(myDataArray[0].get<6>());
            endCommand();
            break;
        default:
            break;
        }
        i++;
    }
    int index = 0;
    for (int i = 26; i <= 44; i++)
    {
        if (i == 44)
        {
            i = 81;
        }
        Serial1.print(nextionTextP(i));
        Serial1.write(0x22);
        Serial1.print(current[index]);
        endCommand();
        index++;
    }
    index = 0;
    for (int i = 22; i <= 58; i++)
    {
        if (i > 25)
            i += 18;
        if (i == 58)
            i = 82;
        Serial1.print(nextionTextP(i));
        Serial1.write(0x22);
        Serial1.print(time[index]);
        endCommand();
        index++;
    }

    for (int i = 59; i <= 77; i++)
    {
        if (i == 77)
            i = 83;
        Serial1.print(nextionTextP(i));
        Serial1.write(0x22);
        Serial1.print(" ");
        endCommand();
    }
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


