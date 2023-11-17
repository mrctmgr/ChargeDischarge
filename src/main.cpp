#include <stdio.h>
#include <string.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "..\inc\NextionDriver.h"


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
  nextion.endCommand();

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
    nextion.endCommand();

    for (i = 0; i <= 19 ; i++)
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
        nextion.endCommand();
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
        tcaselect(TCA_CHNL1);
        Load_Current.setVoltage(MCP4725_value, false);
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
       if(/*TotalVoltage >(SeriesBattery*3) &&*/ TotalTime >= 0 ) // SeriesBattery yerine  *2.5 yazılacak
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
     nextion.endCommand();
        
    }  //WHILE SONU 
        
        if(TotalVoltage > (SeriesBattery*3))  // SeriesBattery yerine  *2.5 yazılacak
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
          nextion.endCommand();
        
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
    nextion.endCommand();

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
    nextion.endCommand();

    ekran = 0;
    buton = 0;
  }
  else if ((ekran == 1) && (buton == 33))
  {
    SeriesBatterySet = SeriesBattery;
    BatVoltage = ((SeriesBatterySet * 4.2 * 4095.0) / 100.0) * 1.015;
    MaxLoadCurrent = ((1500.0 / (SeriesBatterySet * 4.2)));

    Serial1.print("p20.txt=");
    Serial1.write(0x22);
    Serial1.print(SeriesBatterySet);
    nextion.endCommand();

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
    nextion.endCommand();

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
  nextion.endCommand();
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
  nextion.endCommand();
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
  nextion.endCommand();


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
  nextion.endCommand();
  Serial1.print("p14.txt=");
  Serial1.write(0x22);
  Serial1.print(DischargeTimesayi);
  nextion.endCommand();

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
  nextion.reWritePage1(ChargeCurrentsayi,ChargeTimesayi,DischargeCurrentsayi,DischargeTimesayi,TotalTime,SeriesBattery,SeriesBatterySet,Current,Time);
  Serial1.print("p85.txt=");
  Serial1.write(0x22);
  Serial1.print(Scale);
  nextion.endCommand();
}
void ReWriteNextionPage2(void)
{
  WriteNextion();
}
void NextionReset(void)
{
  Sayici = 0;
  nextion.resetNextionScreen();
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
  nextion.writeNextionScreen2(cellFrame, Soc_totVal, TotalVoltage, Current1, temp1, temp2);
}