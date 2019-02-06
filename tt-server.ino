/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
                                                    tt-server.ino 
                              Copyright © 2018-2019, Zigfred & Nik.S
05.12.2018 1
06.12.2018 2 add DS18B20
20.12.2018 3 dell PT1000
23.12.2018 4 PT100 nominalR = 220 om и 1 ком
24.12.2018 5 flow sensor calc switch to pulses per second
24.12.2018 6 повышение розрядности измерения PT100 и датчика давления
24.12.2018 7 json structure updated
30.12.2018 8 в json замена на ds18In, ds18Out, ds18FromTA
09.01.2019 9 static int flowSensorPulsesPerSecond на unsigned long
04.02.2019 v10 добавлена функция freeRam()
06.02.2019 v11 добавлен префикс к переменным "boiler-wood-"
06.02.2019 v12 структура JSON без <ArduinoJson.h>
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*******************************************************************\
Сервер tt-server ArduinoJson выдает данные: 
  аналоговые: 
    датчик давления
    датчик температуры PT100
  цифровые: 
    датчик скорости потока воды YF-B5
    датчики температуры DS18B20
/*******************************************************************/

//#include <ArduinoJson.h>
#include <Ethernet2.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBD_Timer.h>

#define DEVICE_ID "boiler-wood"
#define VERSION 12

#define RESET_UPTIME_TIME 43200000 //  = 30 * 24 * 60 * 60 * 1000 \
                                   // reset after 30 days uptime

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xED};
EthernetServer httpServer(40246);

// YF-B5 - flow sensor
// PT100 - temperature sensors
// pressure sensor
// DS18D20 - array of temperature sensors

#define PIN_ONE_WIRE_BUS 9
uint8_t ds18Precision = 11;
#define DS18_CONVERSION_TIME 750 / (1 << (12 - ds18Precision))
unsigned short ds18DeviceCount;
bool isDS18ParasitePowerModeOn;
OneWire ds18wireBus(PIN_ONE_WIRE_BUS);
DallasTemperature ds18Sensors(&ds18wireBus);

#define PIN_FLOW_SENSOR 2
#define PIN_INTERRUPT_FLOW_SENSOR 0
volatile long flowSensorPulseCount = 0;
// time
unsigned long currentTime;
unsigned long flowSensorLastTime;

#define PT100_1_PIN A1
#define PT100_2_PIN A2
//#define PT100_1_CALIBRATION 165
//#define PT100_2_CALIBRATION 1005
#define PT100_1_CALIBRATION 128
#define PT100_2_CALIBRATION 1000
#define koefB 2.6           // B-коэффициент 0.385 (1/0.385=2.6)
#define data0PT100 100      // сопротивления PT100 при 0 градусах
#define data25PT100 109.73  // сопротивления PT100 при 25 градусах
uint16_t temp;
RBD::Timer ds18ConversionTimer;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            setup
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup() {
  Serial.begin(9600);
  while (!Serial) continue;

  pinMode( A1, INPUT );

  pinMode(PIN_FLOW_SENSOR, INPUT);
  //digitalWrite(PIN_FLOW_SENSOR, HIGH);
  attachInterrupt(PIN_INTERRUPT_FLOW_SENSOR, flowSensorPulseCounter, FALLING);
  sei();

  if (!Ethernet.begin(mac)) {
    Serial.println(F("Failed to initialize Ethernet library"));
    return;
  }
  httpServer.begin();
  Serial.println(F("Server is ready."));
  Serial.print(F("Please connect to http://"));
  Serial.println(Ethernet.localIP());

//  initOneWire();
  ds18Sensors.begin();
  ds18DeviceCount = ds18Sensors.getDeviceCount();

  getSettings();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            Settings
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void getSettings()
{
//  String responseText = doRequest(settingsServiceUri, "");
  // TODO parse settings and fill values to variables
  //intervalLogServicePeriod = 10000;
  //settingsServiceUri
  //intervalLogServiceUri
  //ds18Precision
  ds18Sensors.requestTemperatures();
  //intervalLogServiceTimer.setTimeout(intervalLogServicePeriod);
  //intervalLogServiceTimer.restart();
  ds18ConversionTimer.setTimeout(DS18_CONVERSION_TIME);
  ds18ConversionTimer.restart();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            loop
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void loop() {
  
  currentTime = millis();
  resetWhen30Days();

  realTimeService();
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            my functions
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            UTILS
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void resetWhen30Days()
{
  if (millis() > (RESET_UPTIME_TIME))
  {
    // do reset
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            realTimeService
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void realTimeService()
{

  EthernetClient reqClient = httpServer.available();
  if (!reqClient)
    return;

  while (reqClient.available())
    reqClient.read();
  ds18RequestTemperatures();

      String data = createDataString();

  reqClient.println(F("HTTP/1.1 200 OK"));
  reqClient.println(F("Content-Type: application/json"));
  reqClient.print(F("Content-Length: "));
  reqClient.println(data.length());
  reqClient.println();
  reqClient.print(data);

  reqClient.stop();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            createDataString
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String createDataString()
{
  String resultData;
  resultData.concat(F("{"));
  resultData.concat(F("\n\"deviceId\":"));
  //  resultData.concat(String(DEVICE_ID));
  resultData.concat(F("\"boiler-wood\""));
  resultData.concat(F(","));
  resultData.concat(F("\n\"version\":"));
  resultData.concat((int)VERSION);
  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRam\":"));
  resultData.concat(freeRam());
  resultData.concat(F(","));
  resultData.concat(F("\n\"data\": {"));
  resultData.concat(F("\n\"boiler-wood-pressure\":"));
  resultData.concat(String(getPressureData(), 2));
  resultData.concat(F(","));
  resultData.concat(F("\n\"boiler-wood-tempSmoke\":"));
  resultData.concat(String(getPT100Data(PT100_1_PIN, PT100_1_CALIBRATION)));
  resultData.concat(F(","));
  resultData.concat(F("\n\"boiler-wood-tempPT100\":"));
  resultData.concat(String(getPT100Data(PT100_2_PIN, PT100_2_CALIBRATION)));

  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRamDS-1\":"));
  resultData.concat(freeRam());

  for (uint8_t index = 0; index < ds18DeviceCount; index++)
  {
    DeviceAddress deviceAddress;
    ds18Sensors.getAddress(deviceAddress, index);
    String stringAddr = dsAddressToString(deviceAddress);
    resultData.concat(F(",\n\""));
    resultData.concat(stringAddr);
    resultData.concat(F("\":"));
    resultData.concat(ds18Sensors.getTempC(deviceAddress));
  }

  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRamDS-2\":"));
  resultData.concat(freeRam());

  for (uint8_t index = 0; index < ds18DeviceCount; index++)
  {
    DeviceAddress deviceAddress;
    ds18Sensors.getAddress(deviceAddress, index);

    resultData.concat(F(",\n\""));
    for (uint8_t i = 0; i < 8; i++)
    {

      resultData.concat(deviceAddress[i]);
    }
    resultData.concat(F("\":"));
    resultData.concat(ds18Sensors.getTempC(deviceAddress));
  }
  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRamDS-3\":"));
  resultData.concat(freeRam());

  for (uint8_t index = 0; index < ds18DeviceCount; index++)
  {
    DeviceAddress deviceAddress;
    ds18Sensors.getAddress(deviceAddress, index);
    
    resultData.concat(F(",\n\""));
    for (uint8_t i = 0; i < 8; i++)
    {
  //         if (deviceAddress[i] < 16) resultData.concat("0");
      
      resultData.concat(String(deviceAddress[i], HEX));
    }
    resultData.concat(F("\":"));
    resultData.concat(ds18Sensors.getTempC(deviceAddress));
  }
  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRamDS-4\":"));
  resultData.concat(freeRam());

  resultData.concat(F(","));
  resultData.concat(F("\n\"boiler-wood-flow\":"));
  resultData.concat(String(getFlowData()));
  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRamEnd\":"));
  resultData.concat(freeRam());

  resultData.concat(F("\n}"));
  resultData.concat(F("\n}"));

  return resultData;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            ds18RequestTemperatures
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void ds18RequestTemperatures()
{
  if (ds18ConversionTimer.onRestart())
  {
    ds18Sensors.requestTemperatures();
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            dsAddressToString
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String dsAddressToString(DeviceAddress deviceAddress)
{
  String address;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16)
      address += "0";
    address += String(deviceAddress[i], HEX);
  }
  return address;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement pressure
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
float getPressureData() {

  unsigned int avg_sum=0;
  for(byte i=0; i<8; i++){
  avg_sum += analogRead(A0);
  }
  float aA0 = avg_sum / 8;
  Serial.print(" aA0");
  Serial.println(aA0);
 
// перевод значений в атм [(sensorPressTankFrom - 0,1*1023) / (1,6*1023/9,8)]
  float pressure = ((aA0 - 102.3) / 167);
  //  sensorPressTankFrom = (sensorPressTankFrom * 0.0048875);    //  Напряжение в вольтах 0-5В
  //  sensorPressTankFrom = (sensorPressTankFrom * 0.0259);
  Serial.print("   sensorPress = ");
  Serial.println(pressure);

  return pressure;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement temperature PT100
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int getPT100Data (int analogPin, int nominalR) {

  // float koefB = 2.6; // B-коэффициент 0.385 (1/0.385=2.6)
  // int nominalR = 212; // сопротивление дополнительного резистора
  // int data0PT100 = 100; // сопротивления PT100 при 0 градусах
  //Формула R = nominalR * analogRead(Pin) / (1023 - analogRead(Pin))

  unsigned int avg_sum=0;
  for(byte i=0;i<8;i++){
  avg_sum+=analogRead(analogPin);
  }
  float valuePT100 = avg_sum / 8;
//  Serial.print("  valuePT100 = ");
//  Serial.print(valuePT100);

  float resistancePT100 = 1023.0 - valuePT100;
  resistancePT100 = nominalR / resistancePT100;
  resistancePT100 *= valuePT100;
      //  Serial.print("  resistancePT100 = ");
      //  Serial.print(resistancePT100);

 //     float temp = log(resistancePT100 - data0PT100) + resistancePT100;
  float temp = resistancePT100;
  temp -= data0PT100;
  int tempPT100 = temp * koefB; // *B
  Serial.print("   tempPT100 = ");
  Serial.println(tempPT100);

  return tempPT100;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement flow water
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

int getFlowData() {
  //  static int flowSensorPulsesPerSecond;
  unsigned long flowSensorPulsesPerSecond;

  unsigned long deltaTime = millis() - flowSensorLastTime;
  //  if ((millis() - flowSensorLastTime) < 1000) {
  if (deltaTime < 1000)
  {
    return;
  }

  //detachInterrupt(flowSensorInterrupt);
  detachInterrupt(PIN_INTERRUPT_FLOW_SENSOR);
  //     flowSensorPulsesPerSecond = (1000 * flowSensorPulseCount / (millis() - flowSensorLastTime));
  //    flowSensorPulsesPerSecond = (flowSensorPulseCount * 1000 / deltaTime);
  flowSensorPulsesPerSecond = flowSensorPulseCount;
  flowSensorPulsesPerSecond *= 1000;
  flowSensorPulsesPerSecond /= deltaTime; //  количество за секунду

  flowSensorLastTime = millis();
  flowSensorPulseCount = 0;
  //attachInterrupt(flowSensorInterrupt, flowSensorPulseCounter, FALLING);
  attachInterrupt(PIN_INTERRUPT_FLOW_SENSOR, flowSensorPulseCounter, FALLING);

  return flowSensorPulsesPerSecond;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to counting pulse
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void flowSensorPulseCounter()
{
  // Increment the pulse counter
  flowSensorPulseCount++;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            Количество свободной памяти
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            end
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/