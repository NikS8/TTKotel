/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
                                                               boiler-wood.ino 
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
06.02.2019 v13 изменение вывода №№ DS18 и префикс заменен на "bw-"
14.02.2019 v14 заменен датчик PT100, откалиброван
02.03.2019 v15 add PT1000-smoke
06.03.2019 v16 add HX711 for PT100, dell PT1000 
10.03.2019 v17 время работы после включения
13.11.2019 v18 переход на статические IP
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*****************************************************************************\
Сервер tt-server ArduinoJson выдает данные: 
  аналоговые: 
    датчик давления
    датчик температуры PT100
  цифровые: 
    датчик скорости потока воды YF-B5
    датчики температуры DS18B20
/*****************************************************************************/

#include <Ethernet2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBD_Timer.h>
#include <HX711.h>    //https://github.com/bogde/HX711

#define DEVICE_ID "boiler-wood"
#define VERSION 18

#define RESET_UPTIME_TIME 43200000 //  = 30 * 24 * 60 * 60 * 1000 \
                                   // reset after 30 days uptime
byte mac[] = {0xCA, 0x74, 0xC0, 0xFF, 0xBD, 0x01};
IPAddress ip(192, 168, 1, 111);
EthernetServer httpServer(40111);

#define PIN_PRESSURE_SENSOR A0

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

#define UMIN  100000
#define UMAX 9000000
#define RMIN    40.0
#define RMAX   700.0

const long  Uu = 1557878;//1563838;//1568248;//1842181;    // Rohmesswert unteres Ende  
const long  Uo = 4792792;//4777449;    // Rohmesswert oberes Ende 
const float Ru = 110.5;  // Widerstandswert unteres Ende
const float Ro = 333.0;  // Widerstandswert oberes Ende

long Umess;
float Rx, tempPT100;

HX711 get_U;

RBD::Timer ds18ConversionTimer;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            setup
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup() {
  Serial.begin(9600);
  Serial.println("Serial.begin(9600)"); 

  Ethernet.begin(mac,ip);
  
  Serial.println(F("Server is ready."));
  Serial.print(F("Please connect to http://"));
  Serial.println(Ethernet.localIP());
  
  httpServer.begin();

  pinMode(PIN_PRESSURE_SENSOR, INPUT);
 // pinMode(PIN_PT100_1, INPUT);

  pinMode(PIN_FLOW_SENSOR, INPUT);
  //digitalWrite(PIN_FLOW_SENSOR, HIGH);
  attachInterrupt(PIN_INTERRUPT_FLOW_SENSOR, flowSensorPulseCounter, FALLING);
  sei();
  
//  initOneWire();
  ds18Sensors.begin();
  ds18DeviceCount = ds18Sensors.getDeviceCount();

  getSettings();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            Settings
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void getSettings()
{
  ds18Sensors.requestTemperatures();
  ds18ConversionTimer.setTimeout(DS18_CONVERSION_TIME);
  ds18ConversionTimer.restart();
  ////////////////
  int i;
  long buf = 0;
  long U = 0;
  Serial.println(F("HX711 Temperaturmessung mit Pt100 Widerstandsthermometer"));
  get_U.begin(6, 5, 32);

  // Abgleich Messbereichsgrenze unten
  if (Uu <= UMIN || Ru == RMIN) {
    Serial.println(F("Widerstand fuer untere Messbereichsgrenze eingesetzt ??"));
    Serial.println(F("Messung fuer Messbereichsgrenze unten laeuft - bitte warten ..."));
    for (i = 0; i < 5; i++) {
      U = get_U.read_average(40);
      buf = buf + U;
      Serial.print(U); Serial.print(" ");
    }
    if (buf / 5 < UMIN || buf / 5 > UMAX) {
      Serial.println(); Serial.println();
      Serial.println(F("Messfehler, bitte Schaltung ueberpruefen"));
      while (1);
    }
    else {
      Serial.println(); Serial.println();
      Serial.print(F( "Bitte "));
      Serial.print(buf / 5);
      Serial.println(F( " bei Variable 'Uu' und den Wert des eingesetzten Widerstandes in Ohm bei Variable 'Ru' eintragen "));
      Serial.println(F("Dann das Programm neu hochladen"));
      Serial.println();
      while (1);
    }
  }

  // Abgleich Messbereichsgrenze oben
  if (Uo <= UMIN || Ro == RMIN) {
    Serial.println(F("Widerstand fuer obere Messbereichsgrenze eingesetzt ??"));
    Serial.println(F("Messung fuer Messbereichsgrenze oben laeuft - bitte warten ..."));
    for (i = 0; i < 5; i++) {
      U = get_U.read_average(40);
      buf = buf + U;
      Serial.print(U); Serial.print(" ");
    }
    if (buf / 5 < UMIN || buf / 5 > UMAX) {
      Serial.println(); Serial.println();
      Serial.println(F("Messfehler, bitte Schaltung ueberpruefen"));
      while (1);
    }
    else {
      Serial.println(); Serial.println();
      Serial.print(F( "Bitte "));
      Serial.print(buf / 5);
      Serial.println(F( " bei Variable 'Uo' und den Wert des eingesetzten Widerstandes in Ohm  bei Variable 'Ro' eintragen "));
      Serial.println(F("Dann das Programm neu hochladen"));
      Serial.println();
      while (1);
    }
  }

  // Prüfung der Abgleichwerte
  if ( Uu < UMIN || Uu > UMAX || Uo < UMIN || Uo > UMAX || Ru < RMIN || Ru > RMAX || Ro < RMIN || Ro > RMAX ||
       Uu > Uo || Uo < Uu || Ru > Ro || Ro < Ru) {
    Serial.println(F( "Abgleichfehler - Bitte Abgleich wiederholen"));
    Serial.println();
    while (1);
  }
  else {
    Serial.println(F( "Abgleich plausiebel"));
    Serial.println();
  }
  //////////////
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            loop
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void loop() {
  
  currentTime = millis();
  resetWhen30Days();

  realTimeService();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            my functions
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            UTILS
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void resetWhen30Days()
{
  if (millis() > (RESET_UPTIME_TIME))
  {
    // do reset
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            realTimeService
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            createDataString
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
  resultData.concat(F("\n\"data\": {"));

  resultData.concat(F("\n\"bw-pressure\":"));
  resultData.concat(String(getPressureData(), 2));

  resultData.concat(F(","));
  resultData.concat(F("\n\"bw-tPT100-smoke\":"));
  resultData.concat(String(getPT100Data()));

  for (uint8_t index = 0; index < ds18DeviceCount; index++)
  {
    DeviceAddress deviceAddress;
    ds18Sensors.getAddress(deviceAddress, index);

    resultData.concat(F(",\n\""));
    for (uint8_t i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) resultData.concat("0");

      resultData.concat(String(deviceAddress[i], HEX));
    }
    resultData.concat(F("\":"));
    resultData.concat(ds18Sensors.getTempC(deviceAddress));
  }
 
  resultData.concat(F(","));
  resultData.concat(F("\n\"bw-flow\":"));
  resultData.concat(String(getFlowData()));
  resultData.concat(F("\n}"));

  resultData.concat(F(","));
  resultData.concat(F("\n\"freeRam\":"));
  resultData.concat(freeRam());
  resultData.concat(F(",\n\"upTime\":\""));
  resultData.concat(upTime(millis()));
  
  resultData.concat(F("\"\n}"));

  return resultData;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            ds18RequestTemperatures
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void ds18RequestTemperatures()
{
  if (ds18ConversionTimer.onRestart())
  {
    ds18Sensors.requestTemperatures();
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement pressure
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
float getPressureData() {

  unsigned int avg_sum=0;
  for(byte i=0; i<8; i++){
    avg_sum += analogRead(PIN_PRESSURE_SENSOR);
  }
  float aA0 = avg_sum / 8;
 // перевод значений в атм [(sensorPressTankFrom - 0,1*1023) / (1,6*1023/9,8)]
  float pressure = ((aA0 - 102.3) / 167);
  //  sensorPressTankFrom = (sensorPressTankFrom * 0.0048875);    //  Напряжение в вольтах 0-5В
  //  sensorPressTankFrom = (sensorPressTankFrom * 0.0259);
  Serial.print(F("   sensorPress = "));
  Serial.println(pressure);

  return pressure;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement flow water
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

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
  flowSensorPulsesPerSecond = flowSensorPulseCount;
  flowSensorPulsesPerSecond *= 1000;
  flowSensorPulsesPerSecond /= deltaTime; //  количество за секунду

  flowSensorLastTime = millis();
  flowSensorPulseCount = 0;
  //attachInterrupt(flowSensorInterrupt, flowSensorPulseCounter, FALLING);
  attachInterrupt(PIN_INTERRUPT_FLOW_SENSOR, flowSensorPulseCounter, FALLING);

  return flowSensorPulsesPerSecond;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to counting pulse
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void flowSensorPulseCounter()
{
  // Increment the pulse counter
  flowSensorPulseCount++;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement temperature PT100
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int getPT100Data () {

  float k1, k2, k3, k4, k5, k6, k7, k8, k9;
  
  // Messwert für die Spannung an Rx einlesen
  Umess = get_U.read();

  // Aus der gemessenen Spannung den Widerstand Rx ausrechnen
  if (Umess >= UMIN && Umess <= UMAX) {
    Rx = ((((Ro - Ru) / (Uo - Uu)) * (Umess - Uu)) + Ru );
    //Serial.print("Umess = "); Serial.print(Umess); Serial.print("  ");
    Serial.print("R = ");     Serial.print(Rx, 3);   Serial.print(" Ohm   ->   ");

    // Temperatur für Rx >= 100 Ohm berechnen
    if (Rx >= 100.0) {
      k1 = 3.90802 * pow(10, -1);
      k2 = 2 * 5.802 * pow(10, -5);
      k3 = pow(3.90802 * pow(10, -1), 2);
      k4 = 4.0 * (pow(5.802 * pow(10, -5), 2));
      k5 = Rx - 100.0;
      k6 = 5.802 * pow(10, -5);

      k7 = k1 / k2;
      k8 = (k3 / k4) - (k5 / k6);
      k9 = sqrt(k8);

      tempPT100 = k7 - k9;
    }
    // Temperatur für Rx < 100 Ohm berechnen
    else {
      k1 = pow (Rx, 5) * 1.597 * pow(10, -10);
      k2 = pow (Rx, 4) * 2.951 * pow(10, -8);
      k3 = pow (Rx, 3) * 4.784 * pow(10, -6);
      k4 = pow (Rx, 2) * 2.613 * pow(10, -3);
      k5 = 2.219 * Rx - 241.9;

      tempPT100 = k1 - k2 - k3 + k4 + k5;
    }
    Serial.print("tempPT100 = ");     Serial.print(tempPT100, 3);   Serial.println(" GrdC");
  }
  else {
    Serial.println("Messfehler");
  }
  return tempPT100;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            Время работы после старта или рестарта
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String upTime(uint32_t lasttime)
{
  lasttime /= 1000;
  String lastStartTime;
  
  if (lasttime > 86400) {
    uint8_t lasthour = lasttime/86400;
    lastStartTime.concat(lasthour);
    lastStartTime.concat(F("d "));
    lasttime = (lasttime-(86400*lasthour));
  }
  if (lasttime > 3600) {
    if (lasttime/3600<10) { lastStartTime.concat(F("0")); }
  lastStartTime.concat(lasttime/3600);
  lastStartTime.concat(F(":"));
  }
  if (lasttime/60%60<10) { lastStartTime.concat(F("0")); }
lastStartTime.concat((lasttime/60)%60);
lastStartTime.concat(F(":"));
  if (lasttime%60<10) { lastStartTime.concat(F("0")); }
lastStartTime.concat(lasttime%60);
//lastStartTime.concat(F("s"));

return lastStartTime;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            Количество свободной памяти
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            end
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
