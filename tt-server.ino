/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
                                                    tt-server.ino 
                                Copyright © 2018, Zigfred & Nik.S
05.12.2018 1
06.12.2018 2 add DS18B20
20.12.2018 3 dell PT1000
23.12.2018 4 PT100 nominalR = 220 om и 1 ком
24.12.2018 5 flow sensor calc switch to pulses per second
24.12.2018 6 повышение розрядности измерения PT100 и датчика давления
24.12.2018 7 json structure updated
30.12.2018 8 в json замена на ds18In, ds18Out, ds18FromTA
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

#include <ArduinoJson.h>
#include <Ethernet2.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEVICE_ID "boilerWood"
#define VERSION 7

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xED};
EthernetServer server(40246);

// YF-B5 - flow sensor
// PT100 - temperature sensors
// pressure sensor
// DS18D20 - array of temperature sensors

#define ONE_WIRE_BUS 9
#define TEMPERATURE_PRECISION 11
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsDS(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;

byte flowSensorInterrupt = 0;  // 0 = digital pin 2
byte flowSensorPin       = 2;
long flowSensorLastTime = 0;
volatile byte flowSensorPulseCount = 0;

#define PT100_1_PIN A1
#define PT100_2_PIN A2
#define PT100_1_CALIBRATION 165
#define PT100_2_CALIBRATION 1005
#define koefB 2.6           // B-коэффициент 0.385 (1/0.385=2.6)
#define data0PT100 100      // сопротивления PT100 при 0 градусах
#define data25PT100 109.73  // сопротивления PT100 при 25 градусах
uint16_t temp;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            setup
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup() {
  Serial.begin(9600);
  while (!Serial) continue;

  pinMode( A1, INPUT );
  
  pinMode(flowSensorPin, INPUT);
  digitalWrite(flowSensorPin, HIGH);
  attachInterrupt(flowSensorInterrupt, flowSensorPulseCounter, FALLING);
  
  if (!Ethernet.begin(mac)) {
    Serial.println(F("Failed to initialize Ethernet library"));
    return;
  }
  server.begin();
  Serial.println(F("Server is ready."));
  Serial.print(F("Please connect to http://"));
  Serial.println(Ethernet.localIP());

//  initOneWire();
  sensorsDS.begin();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            loop
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void loop() {

  httpResponse();
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            my functions
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to httpResponse()
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void httpResponse() {

   // Wait for an incomming connection
  EthernetClient client = server.available();

  // Do we have a client?
  if (!client) return;

  Serial.println(F("New client"));

  // Read the request (we ignore the content in this example)
  while (client.available()) client.read();

  sensorsDS.requestTemperatures();    // Command to get temperatures
  
  // Allocate JsonBuffer
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonBuffer<300> jsonBuffer;

  // Create the root object
  JsonObject& root = jsonBuffer.createObject();

  root["deviceId"] = DEVICE_ID;
  root["version"] = VERSION;
  root["pressure"] = getPressureData(); //  давление у насоса ТТ
  root["tempSmoke"] = getPT100Data(PT100_1_PIN, PT100_1_CALIBRATION); //  темп-ра выходящих газов
  root["temp"] = getPT100Data(PT100_2_PIN, PT100_2_CALIBRATION); //  темп-ра дымохода
  root["L/min"] = getFlowData();  //  скорость потока воды в контуре ТТ
  root["ds18Out"] = sensorsDS.getTempCByIndex(1);  //  темп-ра на выходе ТТ
  root["ds18In"] = sensorsDS.getTempCByIndex(0);  //  темп-ра на входе ТТ
  root["ds18FromTA"] = sensorsDS.getTempCByIndex(2);  //  темп-ра воды от ТА
 
  Serial.print(F("Sending: "));
  root.printTo(Serial);
  Serial.println();

  // Write response headers
  client.println("HTTP/1.0 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  // Write JSON document
  root.prettyPrintTo(client);

  // Disconnect
  client.stop();
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

  unsigned int avg_sum=0;
  for(byte i=0;i<8;i++){
  avg_sum+=analogRead(analogPin);
  }
  float valuePT100 = avg_sum / 8;
//  Serial.print("  valuePT100 = ");
//  Serial.print(valuePT100);

 float resistancePT100 = 1023.0 / valuePT100 - 1;
  resistancePT100 = nominalR / resistancePT100;
//  Serial.print("  resistancePT100 = ");
//  Serial.print(resistancePT100);

  float temp = log(resistancePT100 - data0PT100) + resistancePT100; 
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
  static int flowSensorPulsesPerSecond;

  if (flowSensorLastTime + 5000 > millis()) { // just return previous value if last measure was less than 5sec ago
    return flowSensorPulsesPerSecond;
    flowSensorPulsesPerSecond = (millis() - flowSensorLastTime) / 1000 * flowSensorPulseCount;

  }

  flowSensorLastTime = millis();
  flowSensorPulseCount = 0;

  return flowSensorPulsesPerSecond;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to counting pulse
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void flowSensorPulseCounter() {
  // Increment the pulse counter
  flowSensorPulseCount++;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            end
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/