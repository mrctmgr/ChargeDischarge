#include <stdio.h>
#include <string.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "..\inc\NextionDriver.h"
#include "avr8-stub.h"

#define   RESPONSE_SIZE_Nextion  15
#define   RESPONSE_SIZE          150
#define   BMS_SIZE               13
#define   ChargePin              11
#define   TCA_CHNL1              1
#define   TCA_CHNL2              6
#define   TCA_CHNL3              7

const byte interruptPin = 2;

//Nextion Haberleşme Ayarlaması
byte              NextionMsg[RESPONSE_SIZE_Nextion];
uint8_t           ekran = 0, buton = 0;
unsigned long     ChargeCurrentsayi = 0;
unsigned long     DischargeCurrentsayi = 0;
unsigned long     ChargeTimesayi = 0;
unsigned long     DischargeTimesayi = 0;
float             Current[20];
int               Time[20];
int               Durum[20];
int               TotalTime = 0;
int               Sayici;
int               i = 0;

//MasterBMS Haberleşme Ayarlaması
const int BUFFER_SIZE = 13;
uint8_t   responseMsg[RESPONSE_SIZE];
float     cellFrame[32] = {.0F};
float     cellFrame1[32] = {.0F};
int       temp1, temp2;
float     Current1;
int       CurrentSign = 0;
float     TotalVoltage;
uint8_t   ReqCellVolt[BMS_SIZE] = { 0xA5, 0x40, 0x95, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82 };
uint8_t   ReqSoC[BMS_SIZE]      = { 0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D };
uint8_t   ReqTemp[BMS_SIZE]     = { 0xA5, 0x40, 0x92, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F };
uint8_t   ReqCurrent[BMS_SIZE]  = { 0xA5, 0x40, 0x93, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E };
uint16_t  Soc_totVal[3];
uint16_t  SeriesBattery = 0, SeriesBatterySet = 0;
float     BatVoltage = 0.0F;
float     MaxLoadCurrent = 0.0F;
int       x = 0;

//DAC Ayarlaması
Adafruit_MCP4725 Load_Current;
Adafruit_MCP4725 PSU_Voltage;
Adafruit_MCP4725 PSU_Current;

uint16_t  MCP47250_value;
uint16_t  MCP47251_value;
uint16_t  MCP47252_value;
uint16_t  MCP47253_value;
uint16_t  MCP47254_value;
uint16_t  MCP47255_value;
uint16_t  MCP47256_value;
uint16_t  sayi = 0;
uint32_t  MCP4725_value;

//Sıcaklık Ayarlaması
int       toggle = 0;
int       SensStart = 0;
int       DataStart = 0;
int       GetData = 0;
int       Text5 = 0;
int       Text7 = 0;
String    go;
String    msg = "";//Incomming message
int       ResetState = 0;
int       progres = 1;
int       State = 1;
int       Scale = 10;

NextionDriver nextion;
void RESET_System();
void ft_delay(unsigned long sec);
void tcaselect(uint8_t bus);
void serialEvent1();
void ChargeDischarge();
void SetArray();
void ReWriteNextionPage1(void);
void ReWriteNextionPage2(void);
void NextionReset(void);
void SendCell(void);
void SendTemp(void);
void SendSOC(void);
void SendCurrent(void);
void WriteNextion(void);

void setup()
{
  // initialize serial:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  //Serial.println("Setup Başı");
  /*GPIO Initialize*/
  pinMode(ChargePin, OUTPUT);
  digitalWrite(ChargePin, HIGH);
  /*-----------------------------------*/

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), RESET_System, FALLING);

  /*Digital Analog Converter Initialize*/

  tcaselect(TCA_CHNL1);
  Load_Current.begin(0x62);
  tcaselect(TCA_CHNL2);
  PSU_Voltage.begin(0x62);
  tcaselect(TCA_CHNL3);
  PSU_Current.begin(0x62);

  tcaselect(TCA_CHNL1);
  Load_Current.setVoltage(0, false);
  tcaselect(TCA_CHNL2);
  PSU_Voltage.setVoltage(0, false);
  tcaselect(TCA_CHNL3);
  PSU_Current.setVoltage(0, false);

  //Serial.println("Setup Sonu");
  /*-----------------------------------*/
}//void setup bitiş


void RESET_System(void)
{
 // Serial.println("Interrupt Oluştu");
  i = 20;
  TotalTime = 0;
  Sayici = 0;
  for (int k = 0; k < 19; k++)
  {
    Time[k] = 0;
   // Serial.println(Time[k]);
    Current[k] = 0;
    Durum[k] = 0;
  }

  Serial1.print("p79.txt=");
  Serial1.write(0x22);
  Serial1.print(TotalTime);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("j0.val=");
  Serial1.print(0);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  NextionReset();

  ekran = 0;
  buton = 0;
  State = 0;
}

void ft_delay(unsigned long sec)
{
  unsigned long previousTime = millis();
  unsigned long currentTime = previousTime;
  while (millis() - previousTime <= (1000 * sec))
  {
    if (State == 0)
    {
      break;
    }
  }
}

void loop()
{

  //  Ekran 1 mi/////- Charge Cur//////+ Charge Cur//////- Charge Time/// + Charge Time/// + Dscharge Cur// - Dscharge Cur//+ Dscharge Time/// - Dscharge Time
  if ((ekran == 1) && ((buton == 6) || (buton == 7) || (buton == 102) || (buton == 101) || (buton == 21) || (buton == 20) || (buton == 26) || (buton == 27)))
  {
    ChargeDischarge();
    ekran = 0;
    buton = 0;
  }///////Ekran 1 mi//////Charge Set mi//// Discharge Set mi
  else if ((ekran == 1) && ((buton == 13) || (buton == 25)))
  {
    SetArray() ;
    ekran = 0;
    buton = 0;
  }
  ////////ekran 1 mi///// Buton Start mı?
  else if ((ekran == 1) && (buton == 93))
  {
    for (int j = 0; j < 19; j++)
    {
      TotalTime = TotalTime + Time[j];
    }
    Serial1.print("p79.txt=");
    Serial1.write(0x22);
    Serial1.print(TotalTime);
    Serial1.write(0x22);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    for (i = 0; i < 19 ; i++)
    {
      if (Durum[i] == 1) //Charge
      {
        digitalWrite(ChargePin, LOW);
        progres = (i + 1) * 5.26;
        Serial1.print("j0.val=");
        Serial1.print(progres);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.print("j0.val=");
        Serial1.print(progres);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.write(0xff);

     
        MCP4725_value = ((819.0 * Current[i]) / 3.0) * 1.07;
        tcaselect(TCA_CHNL1);
        Load_Current.setVoltage(0, false);
        tcaselect(TCA_CHNL2);
        PSU_Voltage.setVoltage(BatVoltage, false);
        tcaselect(TCA_CHNL3);
        PSU_Current.setVoltage(MCP4725_value, false);
        ft_delay(Time[i]);
        State = 1;
        TotalTime = TotalTime - Time[i];
        Serial1.print("p79.txt=");
        Serial1.write(0x22);
        Serial1.print(TotalTime);
        Serial1.write(0x22);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial.print("Total time şarj : ");
        Serial.println(TotalTime);

      }
      
      else if (Durum[i] == 2) //Discharge
      {
        digitalWrite(ChargePin, HIGH);
        
        if (Current[i] >= MaxLoadCurrent)
        {
          Current[i] = MaxLoadCurrent;
        }
        
        



       // Serial.print(i);
        MCP4725_value = ((4095.0 * Current[i]) / 100.0);
        //Serial.println(Current[i]);
        tcaselect(TCA_CHNL2);
        PSU_Voltage.setVoltage(0, false);
        tcaselect(TCA_CHNL3);
        PSU_Current.setVoltage(0, false);

        
    unsigned long previousTime = millis();
    unsigned long currentTime = previousTime;
    unsigned long elapsedTime = 0;
    while ((currentTime - previousTime) / 1000 <= TotalTime)
    {
      SendCell();
      TotalVoltage=0;
      Serial.print("time i : ");
      Serial.println(currentTime - previousTime);
      Serial.print("current Time : ");
      currentTime = millis();
      Serial.println((currentTime - previousTime) / 1000);
      elapsedTime = currentTime - previousTime;
        progres = (elapsedTime / 1000 + 1) * 5.26 / 10;
        Serial.print("progres: ");
        Serial.println(progres);
        Serial1.print("j0.val=");
        Serial1.print(progres);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.write(0xff);

        Serial1.print("j0.val=");
        Serial1.print(progres);
        Serial1.write(0xff);
        Serial1.write(0xff);
        Serial1.write(0xff);
      if (TotalTime <= (currentTime - previousTime) / 1000)
        break;
      
      for (int a = 0; a < SeriesBatterySet; a++)
    {
      cellFrame[a] = cellFrame1[a];
      TotalVoltage = TotalVoltage + cellFrame[a];
    }

      Serial.print("voltaj : ");
      Serial.println(TotalVoltage);
       if(TotalVoltage >(SeriesBattery*3) && TotalTime >= 0 ) // SeriesBattery yerine  *2.5 yazılacak
        {
        tcaselect(TCA_CHNL1);
        Load_Current.setVoltage(MCP4725_value, false);
        }
        else
        {
        tcaselect(TCA_CHNL1);
        Load_Current.setVoltage(0, false);
        Serial.println("Break yaptı");
        break;  //while döngüsünü kıracak
       
        }
        
     //TotalTime = TotalTime - ((Time[i] / 30));
     //TotalTime = (currentTime - previousTime) / 1000;
     Serial1.print("p79.txt=");
     Serial1.write(0x22);
     Serial1.print(TotalTime);
     Serial1.write(0x22);
     Serial1.write(0xff);
     Serial1.write(0xff);
     Serial1.write(0xff);
        Serial.print("Total Time : ");
        Serial.println(TotalTime);
        
        }  //WHILE SONU 
        
        if(TotalVoltage <= (SeriesBattery*3))  // SeriesBattery yerine  *2.5 yazılacak
        {
        i = 20;
        TotalTime = 0;
        Sayici = 0;
        for (int k = 0; k < 19; k++)
        {
          Time[k] = 0;
         // Serial.println(Time[k]);
          Current[k] = 0;
          Durum[k] = 0;
        }
        
          Serial1.print("p79.txt=");
          Serial1.write(0x22);
          Serial1.print(TotalTime);
          Serial1.write(0x22);
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        
          Serial1.print("j0.val=");
          Serial1.print(0);
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);

          NextionReset();

          ekran = 0;
          buton = 0;
          State = 0;
   
 
          }
      }
    }
  
    TotalTime        = 0;
    digitalWrite(ChargePin, HIGH);
  
    tcaselect(TCA_CHNL1);
    Load_Current.setVoltage(0, false);
    tcaselect(TCA_CHNL2);
    PSU_Voltage.setVoltage(0, false);
    tcaselect(TCA_CHNL3);
    PSU_Current.setVoltage(0, false);

    Serial1.print("p79.txt=");
    Serial1.write(0x22);
    Serial1.print(TotalTime);
    Serial1.write(0x22);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    Serial1.print("j0.val=");
    Serial1.print(0);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    ekran = 0;
    buton = 0;
  }


  else if ((ekran == 1) && ((buton == 31) || (buton == 32)))
  {
    if ((ekran == 1) && (buton == 32))
    {
      SeriesBattery = SeriesBattery + 1;
    }
    if ((ekran == 1) && (buton == 31))
    {
      SeriesBattery = SeriesBattery - 1;
    }

    if ( SeriesBattery >= 32)
    {
      SeriesBattery = 32;
    }
    else if ( SeriesBattery < 32 && SeriesBattery > 0)
    {
      SeriesBattery = SeriesBattery + 0;
    }
    else
    {
      SeriesBattery = 0;
    }

    Serial1.print("p19.txt=");
    Serial1.write(0x22);
    Serial1.print(SeriesBattery);
    Serial1.write(0x22);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    ekran = 0;
    buton = 0;
  }
  else if ((ekran == 1) && (buton == 33))
  {
    SeriesBatterySet = SeriesBattery;
    BatVoltage = ((SeriesBatterySet * 4.2 * 4095.0) / 100.0) * 1.015;
    MaxLoadCurrent = ((1500.0 / (SeriesBatterySet * 4.2)));
//    Serial.print("BatVoltage :");
//    Serial.println(BatVoltage);
//    Serial.print("MaxLoadCurrent :");
//    Serial.print(MaxLoadCurrent);

    Serial1.print("p20.txt=");
    Serial1.write(0x22);
    Serial1.print(SeriesBatterySet);
    Serial1.write(0x22);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    ekran = 0;
    buton = 0;
  }
  else if ((ekran == 1) && (buton == 97))
  {
    //ReWriteNextionPage2();
    SendCell();
    TotalVoltage = 0;
    for (int a = 0; a < SeriesBatterySet; a++)
    {
      cellFrame[a] = cellFrame1[a];
      TotalVoltage = TotalVoltage + cellFrame[a];
    }
    SendTemp();
    SendCurrent();
    WriteNextion();

    ekran = 0;
    buton = 0;
  }

  else if ((ekran == 1) && ((buton == 104) || (buton == 105)))
  {
    if (buton == 104)
    {
      Scale = Scale - 10;
      if (Scale <= 0)
      {
        Scale = 0;
      }
      else
      {
        Scale = Scale + 0;
      }
    }
    else
    {
      Scale = Scale + 10;
      if (Scale >= 60)
      {
        Scale = 60;
      }
      else
      {
        Scale = Scale + 0;
      }
    }

    Serial1.print("p85.txt=");
    Serial1.write(0x22);
    Serial1.print(Scale);
    Serial1.write(0x22);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    ekran = 0;
    buton = 0;
  }

  else if ((ekran == 2) && (buton == 1))
  {
    ReWriteNextionPage1();

    ekran = 0;
    buton = 0;
  }
}//VOİD LOOP BİTİŞ


void tcaselect(uint8_t bus)
{
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void serialEvent1()
{
  while ((Serial1.available() > 0))
  {
    Serial1.readBytes(NextionMsg, RESPONSE_SIZE_Nextion);
    ekran = NextionMsg[1];
    Serial.print("Ekran");
    Serial.println(ekran);
    buton = NextionMsg[2];
    Serial.print("buton");
    Serial.println(buton);
  }
}

void ChargeDischarge(void)
{
  /*Charge Current*/
  if (ekran == 1 && buton == 7)
  {
    ChargeCurrentsayi = ChargeCurrentsayi + 1;
    if (ChargeCurrentsayi > 15)
    {
      ChargeCurrentsayi = 15;
    }
  }

  if (ekran == 1 && buton == 6)
  {
    ChargeCurrentsayi = ChargeCurrentsayi - 1;
    if (ChargeCurrentsayi < 0)
    {
      ChargeCurrentsayi = 0;
    }
  }

  Serial1.print("p4.txt=");
  Serial1.write(0x22);
  Serial1.print(ChargeCurrentsayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  /*-----------------*/

  /*Charge Time*/
  if (ekran == 1 && buton == 101)
  {
    ChargeTimesayi = ChargeTimesayi + Scale;
    if (ChargeTimesayi > 600)
    {
      ChargeTimesayi = 600;
    }
  }
  if (ekran == 1 && buton == 102)
  {
    ChargeTimesayi = ChargeTimesayi - Scale;
    if (ChargeTimesayi < 0)
    {
      ChargeTimesayi = 0;
    }
  }
  Serial1.print("p6.txt=");
  Serial1.write(0x22);
  Serial1.print(ChargeTimesayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  /*-----------------*/

  /*Discharge Current*/
  if (ekran == 1 && buton == 20)
  {
    DischargeCurrentsayi = DischargeCurrentsayi + 1;
    if (DischargeCurrentsayi > 100)
    {
      DischargeCurrentsayi = 100;
    }
  }
  if (ekran == 1 && buton == 21)
  {
    DischargeCurrentsayi = DischargeCurrentsayi - 1;
    if (DischargeCurrentsayi < 0)
    {
      DischargeCurrentsayi = 0;
    }
  }
  Serial1.print("p12.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeCurrentsayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);


  /*-----------------*/

  /*Discharge Time*/
  if (ekran == 1 && buton == 26)
  {

    DischargeTimesayi = DischargeTimesayi + Scale;
    if (DischargeTimesayi > 600)
    {
      DischargeTimesayi = 600;
    }
  }
  if (ekran == 1 && buton == 27)
  {

    DischargeTimesayi = DischargeTimesayi - Scale;
    if (DischargeTimesayi < 0)
    {
      DischargeTimesayi = 0;
    }
  }
  Serial1.print("p14.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeTimesayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.print("p14.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeTimesayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  ekran = 0;
  buton = 0;
  /*-----------------*/
}

void SetArray(void)
{
  if (ekran == 1 && buton == 13)
  {
    if (Sayici == 19)
    {
      Sayici = 0;
    }
    Current[Sayici] = ChargeCurrentsayi;
    Time[Sayici]    = ChargeTimesayi;
    Durum[Sayici]   = 1;

    int firstRow = 59;
    int secRow   = 26;
    int thirdRow = 22;
    nextion.sendChargeCommand(firstRow,secRow,thirdRow,Current,Sayici,Time);
    Sayici++;
  }
  else if (ekran == 1 && buton == 25)
  {
    if (Sayici == 19)
    {
      Sayici = 0;
    }

    Current[Sayici] = DischargeCurrentsayi;
    Time[Sayici]    = DischargeTimesayi;
    Durum[Sayici]   = 2;

    int firstRow = 59;
    int secRow   = 26;
    int thirdRow = 22;
    nextion.sendDischargehargeCommand(firstRow,secRow,thirdRow,Current,Sayici,Time);
    Sayici++;
  }
}

void ReWriteNextionPage1(void)
{
  Serial1.write(0x22);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p4.txt=");
  Serial1.write(0x22);
  Serial1.print(ChargeCurrentsayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p6.txt=");
  Serial1.write(0x22);
  Serial1.print(ChargeTimesayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p12.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeCurrentsayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p14.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeTimesayi);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p79.txt=");
  Serial1.write(0x22);
  Serial1.print(TotalTime);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p19.txt=");
  Serial1.write(0x22);
  Serial1.print(SeriesBattery);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p20.txt=");
  Serial1.write(0x22);
  Serial1.print(SeriesBatterySet);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  //--- Birinci Satır ---///
  Serial1.print("p26.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[0]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p27.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[1]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p28.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[2]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p29.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[3]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p30.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[4]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p31.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[5]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p32.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[6]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p33.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[7]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p34.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[8]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p35.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[9]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p36.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[10]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p37.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[11]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p38.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[12]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p39.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[13]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p40.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[14]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p41.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[15]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p42.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[16]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p43.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[17]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p81.txt=");
  Serial1.write(0x22);
  Serial1.print(Current[18]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- Birinci Satır Sonu---///

  //--- İkinci Satır ---///
  Serial1.print("p22.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[0]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p23.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[1]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p24.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[2]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p25.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[3]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p44.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[4]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p45.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[5]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p46.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[6]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p47.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[7]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p48.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[8]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p49.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[9]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p50.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[10]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p51.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[11]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p52.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[12]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p53.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[13]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p54.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[14]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p55.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[15]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p56.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[16]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p57.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[17]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p82.txt=");
  Serial1.write(0x22);
  Serial1.print(Time[18]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- İkinci Satır Sonu---///

  //--- Üçüncü Satır ---///
  Serial1.print("p59.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p60.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p61.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p62.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p63.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p64.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p65.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p66.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p67.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p68.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p69.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p70.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p71.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p72.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p73.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p74.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p75.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p76.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p83.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- Üçüncü Satır Sonu---///

  Serial1.print("p85.txt=");
  Serial1.write(0x22);
  Serial1.print(Scale);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}
void ReWriteNextionPage2(void)
{
  WriteNextion();
}
void NextionReset(void)
{
  Sayici = 0;

  Serial1.write(0x22);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  //--- Birinci Satır ---///
  Serial1.print("p26.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p27.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p28.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p29.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p30.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p31.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p32.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p33.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p34.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p35.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p36.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p37.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p38.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p39.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p40.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p41.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p42.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p43.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p81.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- Birinci Satır Sonu---///

  //--- İkinci Satır ---///
  Serial1.print("p22.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p23.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p24.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p25.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p44.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p45.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p46.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p47.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p48.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p49.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p50.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p51.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p52.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p53.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p54.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p55.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p56.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p57.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p82.txt=");
  Serial1.write(0x22);
  Serial1.print(0);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- İkinci Satır Sonu---///

  //--- Üçüncü Satır ---///
  Serial1.print("p59.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p60.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p61.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p62.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p63.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p64.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p65.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p66.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p67.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p68.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p69.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p70.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p71.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p72.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p73.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p74.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p75.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p76.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("p83.txt=");
  Serial1.write(0x22);
  Serial1.print(" ");
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  //--- Üçüncü Satır Sonu---///
  //Serial.println("Resetledik");
}
/*MasterBMS ile Haberleşme Bölümü*/
void SendCell(void)
{
  for (int j = 0; j < BMS_SIZE; j++)
  {
    Serial2.write(ReqCellVolt[j]);
  }
  while (!(Serial2.available() > 0));
  Serial2.readBytes(responseMsg, RESPONSE_SIZE);
  for (int k = 0; k < RESPONSE_SIZE; k++)
  {
    if ((responseMsg[k] == 0x95) && (responseMsg[k + 1] == 0x08))
    {
      if (responseMsg[k + 2] == 0x01)
      {
        cellFrame1[0] = responseMsg[k + 3] / 10.0;
        cellFrame1[1] = responseMsg[k + 4] / 10.0;
        cellFrame1[2] = responseMsg[k + 5] / 10.0;
        cellFrame1[3] = responseMsg[k + 6] / 10.0;
        cellFrame1[4] = responseMsg[k + 7] / 10.0;
        cellFrame1[5] = responseMsg[k + 8] / 10.0;
        cellFrame1[6] = responseMsg[k + 9] / 10.0;
      }
      else if (responseMsg[k + 2] == 0x02)
      {
        cellFrame1[7] = responseMsg[k + 3] / 10.0;
        cellFrame1[8] = responseMsg[k + 4] / 10.0;
        cellFrame1[9] = responseMsg[k + 5] / 10.0;
        cellFrame1[10] = responseMsg[k + 6] / 10.0;
        cellFrame1[11] = responseMsg[k + 7] / 10.0;
        cellFrame1[12] = responseMsg[k + 8] / 10.0;
        cellFrame1[13] = responseMsg[k + 9] / 10.0;
      }
      else if (responseMsg[k + 2] == 0x03)
      {
        cellFrame1[14] = responseMsg[k + 3] / 10.0;
        cellFrame1[15] = responseMsg[k + 4] / 10.0;
        cellFrame1[16] = responseMsg[k + 5] / 10.0;
        cellFrame1[17] = responseMsg[k + 6] / 10.0;
        cellFrame1[18] = responseMsg[k + 7] / 10.0;
        cellFrame1[19] = responseMsg[k + 8] / 10.0;
        cellFrame1[20] = responseMsg[k + 9] / 10.0;
      }
      else if (responseMsg[k + 2] == 0x04)
      {
        cellFrame1[21] = responseMsg[k + 3] / 10.0;
        cellFrame1[22] = responseMsg[k + 4] / 10.0;
        cellFrame1[23] = responseMsg[k + 5] / 10.0;
        cellFrame1[24] = responseMsg[k + 6] / 10.0;
        cellFrame1[25] = responseMsg[k + 7] / 10.0;
        cellFrame1[26] = responseMsg[k + 8] / 10.0;
        cellFrame1[27] = responseMsg[k + 9] / 10.0;
      }
      else if (responseMsg[k + 2] == 0x05)
      {
        cellFrame1[28] = responseMsg[k + 3] / 10.0;
        cellFrame1[29] = responseMsg[k + 4] / 10.0;
        cellFrame1[30] = responseMsg[k + 5] / 10.0;
        cellFrame1[31] = responseMsg[k + 6] / 10.0;
        //TotalVoltage   = responseMsg[k + 7];
      }
      else
      {

      }
    }
  }
  for (int a = 0; a < RESPONSE_SIZE; a++)
  {
    responseMsg[a] = 0;
  }
}


void SendTemp(void)
{
  for (int j = 0; j < BMS_SIZE; j++)
  {
    Serial2.write(ReqTemp[j]);
  }
  while (!(Serial2.available() > 0));
  Serial2.readBytes(responseMsg, RESPONSE_SIZE);
  for (int k = 0; k < RESPONSE_SIZE; k++)
  {
    if ((responseMsg[k] == 0x92) && (responseMsg[k + 1] == 0x08))
    {
      temp1 = responseMsg[k + 2];
      temp2 = responseMsg[k + 3];
    }
  }
  for (int a = 0; a < RESPONSE_SIZE; a++)
  {
    responseMsg[a] = 0;
  }
}


void SendSOC(void)
{
  for (int j = 0; j < BMS_SIZE; j++)
  {
    Serial2.write(ReqSoC[j]);
  }
  while (!(Serial2.available() > 0));
  Serial2.readBytes(responseMsg, RESPONSE_SIZE);
  for (int k = 0; k < RESPONSE_SIZE; k++)
  {
    if ((responseMsg[k] == 0x90) && (responseMsg[k + 1] == 0x08))
    {
      Soc_totVal[0] = responseMsg[k + 2];
    }
  }
  for (int a = 0; a < RESPONSE_SIZE; a++)
  {
    responseMsg[a] = 0;
  }
}

void SendCurrent(void)
{
  for (int j = 0; j < BMS_SIZE; j++)
  {
    Serial2.write(ReqCurrent[j]);
  }
  while (!(Serial2.available() > 0));
  Serial2.readBytes(responseMsg, RESPONSE_SIZE);
  for (int k = 0; k < RESPONSE_SIZE; k++)
  {
    if ((responseMsg[k] == 0x93) && (responseMsg[k + 1] == 0x08))
    {
      Current1    = responseMsg[k + 2];
      CurrentSign = responseMsg[k + 3];
    }
  }
  for (int a = 0; a < RESPONSE_SIZE; a++)
  {
    responseMsg[a] = 0;
  }
  if (CurrentSign == '-')
  {
    Current1 = Current1 * -1;
  }
  else
  {
    Current1 = Current1 + 0;
  }
}
void WriteNextion(void)
{
  Serial1.write(0x22);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t72.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[0]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t73.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[1]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t74.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[2]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t75.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[3]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t76.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[4]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t77.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[5]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t78.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[6]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t79.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[7]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t81.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[8]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t82.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[9]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t83.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[10]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t84.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[11]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t85.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[12]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t86.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[13]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t87.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[14]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t88.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[15]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  //-------
  Serial1.print("t90.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[16]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t91.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[17]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t92.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[18]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t93.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[19]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t94.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[20]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t95.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[21]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t96.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[22]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t97.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[23]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t99.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[24]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t100.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[25]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t101.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[26]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t102.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[27]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t103.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[28]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);


  Serial1.print("t104.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[29]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);


  Serial1.print("t105.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[30]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t106.txt=");
  Serial1.write(0x22);
  Serial1.print(cellFrame[31]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  //Serial.println(TotalVoltage);
  Serial1.print("t80.txt=");
  Serial1.write(0x22);
  Serial1.print(TotalVoltage);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t89.txt=");
  Serial1.write(0x22);
  Serial1.print(Current1);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t98.txt=");
  Serial1.write(0x22);
  Serial1.print(temp1);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t107.txt=");
  Serial1.write(0x22);
  Serial1.print(temp2);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t71.txt=");
  Serial1.write(0x22);
  Serial1.print(Soc_totVal[0]);
  Serial1.write(0x22);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}