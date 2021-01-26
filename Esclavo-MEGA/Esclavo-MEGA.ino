#include <Arduino.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

/*------------------------------------------OXIGENO-------------------------------------------------------------------------*/

#define DoSensorPin A2 
#define VREF 5000 
float doValue; 
float temperature = 25; 
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1]; 
byte receivedBufferIndex = 0;
#define SCOUNT 30 
int analogBuffer[SCOUNT]; 
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
#define SaturationDoVoltageAddress 12 
#define SaturationDoTemperatureAddress 16 
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = { 
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86, 9.66, 9.46, 9.27,
9.08, 8.90, 8.73, 8.57, 8.41,
8.25, 8.11, 7.96, 7.82, 7.69,
7.56, 7.43, 7.30, 7.18, 7.07,
6.95, 6.84, 6.73, 6.63, 6.53,
6.41,};

/*------------------------------------------------------------------------------------------*/

SoftwareSerial espSerial(2,3);
String str;

/*---------TURBIDEZ-------------------------------------------------------------------------*/
#define Turbidy_sensor A0   
float Tension = 0.0;  
float NTU = 0.0;  
/*------------------------------------------------------------------------------------------*/

/*----------POTENCIAL_HIDRÓGENO-------------------------------------------------------------*/
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.05            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
/*------------------------------------------------------------------------------------------*/

/*---------------------TEMPERATURA-------------------------*/
int DS18S20_Pin = 7; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
/*--------------------------------------------------------*/


void setup(){
Serial.begin(9600);
espSerial.begin(9600);
pinMode(DoSensorPin,INPUT);
readDoCharacteristicValues();
delay(2000);
}



void loop()
{
/*---------turbidez---------*/
  Tension = 0;  
  Tension = analogRead(Turbidy_sensor)/1024*5; // Mapeo de la lectura analógica  
  //Para compensar el ruido producido en el sensor tomamos 500 muestras y obtenemos la media  
 for(int i=0; i<500; i++)  
    {  
      Tension += ((float)analogRead(Turbidy_sensor)/1024)*5;  
    }  
    Tension = Tension/500;  
    Tension = redondeo(Tension,1);  
    //Para ajustarnos a la gráfica de la derecha  
    if(Tension < 2.5){  
      NTU = 3000;  
    }else{  
      NTU = -1120.4*square(Tension)+5742.3*Tension-4352.9;   
    }  
/*--------------------------*/

/*-----------------------------POTENCIAL-HIDRÓGENO-------------------------------*/
static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
        printTime=millis();
  }
/*-------------------------------------------------------------------------------*/

/*-------------------TEMPERATURA---------------------*/
float temperatura = getTemp();
/*---------------------------------------------------*/

/*-----------------OXYGENO DISUELTO---------------------------------------------*/

static unsigned long analogSampleTimepoint = millis();
 if(millis()-analogSampleTimepoint > 30U)
 {
 analogSampleTimepoint = millis();
 analogBuffer[analogBufferIndex] = analogRead(DoSensorPin); 
 analogBufferIndex++;
 if(analogBufferIndex == SCOUNT)
 analogBufferIndex = 0;
 }
 
 static unsigned long tempSampleTimepoint = millis();
 if(millis()-tempSampleTimepoint > 500U)
 {
 tempSampleTimepoint = millis();
 }

 static unsigned long printTimepoint = millis();
 if(millis()-printTimepoint > 1000U)
 {
 printTimepoint = millis();
 for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
 {
 analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
 }
 averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
 doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage; 
//Serial.println(doValue,2);
 }

 if(serialDataAvailable() > 0)
 {
 byte modeIndex = uartParse(); //parse the uart command received
 doCalibration(modeIndex); // If the correct calibration command is received, the calibration function should be called.
 }




/*------------------------------------------------------------------------------*/
/*----- leyendo data --- */
float temp = 1.20;
str =String(temperatura)+String(",")+String(pHValue,2)+String(",")+String(NTU)+String(",")+String(doValue,2)+String(",")+String(temp);
//str =String(temperatura)+String(",")+String(random2)+String(",")+String(random3)+String(",")+String(random4)+String(",")+String(temp);
Serial.println(str);
espSerial.println(str);
delay(7000);

}






/*------------------CLASES----------------------------------------------------------------------------------------------------*/

float redondeo(float p_entera, int p_decimal)  
{  
  float multiplicador = powf( 10.0f, p_decimal);  //redondeo a 2 decimales  
  p_entera = roundf(p_entera * multiplicador) / multiplicador;  
  return p_entera;  
}   

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

boolean serialDataAvailable(void)
{
 char receivedChar;
 static unsigned long receivedTimeOut = millis();
 while ( Serial.available() > 0 )
 {
 if (millis() - receivedTimeOut > 500U)
 {
 receivedBufferIndex = 0;
 memset(receivedBuffer,0,(ReceivedBufferLength+1));
 }
 receivedTimeOut = millis();
 receivedChar = Serial.read();
 if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
 {
receivedBufferIndex = 0;
strupr(receivedBuffer);
return true;
 }else{
 receivedBuffer[receivedBufferIndex] = receivedChar;
 receivedBufferIndex++;
 }
 }
 return false;
}
byte uartParse()
{
 byte modeIndex = 0;
 if(strstr(receivedBuffer, "CALIBRATION") != NULL)
 modeIndex = 1;
 else if(strstr(receivedBuffer, "EXIT") != NULL)
 modeIndex = 3;
 else if(strstr(receivedBuffer, "SATCAL") != NULL)
 modeIndex = 2;
 return modeIndex;
}
void doCalibration(byte mode)
{
 char *receivedBufferPtr;
 static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
 float voltageValueStore;
 switch(mode)
 {
 case 0:
 if(enterCalibrationFlag)
 Serial.println(F("Command Error"));
 break;

 case 1:
 enterCalibrationFlag = 1;
 doCalibrationFinishFlag = 0;
 Serial.println();
 Serial.println(F(">>>Enter Calibration Mode<<<"));
 Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
 Serial.println();
 break;

 case 2:
 if(enterCalibrationFlag)
 {
 Serial.println();
 Serial.println(F(">>>Saturation Calibration Finish!<<<"));
 Serial.println();
 EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
 EEPROM_write(SaturationDoTemperatureAddress, temperature);
 SaturationDoVoltage = averageVoltage;
 SaturationDoTemperature = temperature;
 doCalibrationFinishFlag = 1;
 }
 break;
 case 3:
 if(enterCalibrationFlag)
 {
 Serial.println();
 if(doCalibrationFinishFlag)
 Serial.print(F(">>>Calibration Successful"));
 else
 Serial.print(F(">>>Calibration Failed"));
 Serial.println(F(",Exit Calibration Mode<<<"));
 Serial.println();
 doCalibrationFinishFlag = 0;
 enterCalibrationFlag = 0;
 }
 break;
 }
}
int getMedianNum(int bArray[], int iFilterLen)
{
 int bTab[iFilterLen];
 for (byte i = 0; i<iFilterLen; i++)
 {
 bTab[i] = bArray[i];
 }
 int i, j, bTemp;
 for (j = 0; j < iFilterLen - 1; j++)
 {
 for (i = 0; i < iFilterLen - j - 1; i++)
 {
 if (bTab[i] > bTab[i + 1])
 {
bTemp = bTab[i];
 bTab[i] = bTab[i + 1];
bTab[i + 1] = bTemp;
 }
 }
 }
 if ((iFilterLen & 1) > 0)
bTemp = bTab[(iFilterLen - 1) / 2];
 else
bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
 return bTemp;
}
void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);  
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }    
}
