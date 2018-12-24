/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
                                                    tt-server.ino 
                                Copyright © 2018, Zigfred & Nik.S
    05.12.2018 v0.1
    06.12.2018 v0.2 add DS18B20
    20.12.2018 v0.3 dell PT1000
    23.12.2018 v0.4 PT100 nominalR = 212 om
    повышение розрядности измерения PT100
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
float flowSensorCalibrationFactor = 6.6;
volatile byte flowSensorPulseCount = 0;  
float flowSensorRate = 0;
unsigned int flowSensorMilliLitres = 0;
unsigned long flowSensorTotalMilliLitres = 0;
unsigned long flowSensorOldTime = 0;

#define PT100_1_PIN A0
#define PT100_2_PIN A1
#define PT100_1_CALIBRATION 165
#define PT100_2_CALIBRATION 975
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
  
  root["TTKotel"] =" v0.4 ";
  root["pressure"] = getPressureData(); //  давление у насоса ТТ
  root["tempSmoke"] = getPT100Data(PT100_1_PIN, PT100_1_CALIBRATION); //  температура выходящих газов
  root["temp"] = getPT100Data(PT100_2_PIN, PT100_2_CALIBRATION); //  температура выходящих газов
  root["L/min"] = getFlowData();  //  скорость потока воды в контуре ТТ
//  root["litersTotal"] = getFlowData();  //  объем прокачанной воды в ТТ
  root["tempTToutIndx"] = sensorsDS.getTempCByIndex(1);  //  темп-ра на выходе ТТ
  root["tempTTinIndx"] = sensorsDS.getTempCByIndex(0);  //  темп-ра на входе ТТ
  root["tempInverseIndx"] = sensorsDS.getTempCByIndex(2);  //  темп-ра обратной воды
 
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
long getFlowData() {
 if((millis() - flowSensorOldTime) > 1000)    // Only process counters once per second
  {
  
    detachInterrupt(flowSensorInterrupt);
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowSensorRate = ((1000.0 / (millis() - flowSensorOldTime)) * flowSensorPulseCount) / flowSensorCalibrationFactor;
  /*    flowSensorRate = millis() - flowSensorOldTime;
      flowSensorRate = 1000.0 / flowSensorRate;
      flowSensorRate *= flowSensorPulseCount;
      flowSensorRate /= flowSensorCalibrationFactor;
  */  
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    flowSensorOldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowSensorMilliLitres = (flowSensorRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    flowSensorTotalMilliLitres += flowSensorMilliLitres;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowSensorRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(flowSensorTotalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");       // Print tab space
    Serial.print(flowSensorTotalMilliLitres/1000);
    Serial.print("L");

    // Reset the pulse counter so we can start incrementing again
    flowSensorPulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(flowSensorInterrupt, flowSensorPulseCounter, FALLING);

  // return flowSensorTotalMilliLitres;
   return flowSensorRate;
  }
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