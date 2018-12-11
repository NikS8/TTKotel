/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
                                                    tt-server.ino 
                                Copyright © 2018, Zigfred & Nik.S
    05.12.2018 v0.1
    06.12.2018 v0.2 add DS18B20
    06.12.2018 v0.3 
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*******************************************************************\
    Сервер tt-server ArduinoJson выдает данные: 
        аналоговые: 
            датчик давления
            датчик температуры PT100
            датчик температуры PT1000
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
// PT100
// PT1000
// pressure sensor
// DS18D20 - array of temperature sensors

#define ONE_WIRE_BUS 9
#define TEMPERATURE_PRECISION 9
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            setup
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup() {
  Serial.begin(9600);
  while (!Serial) continue;

  pinMode( A1, INPUT );
  pinMode( A2, INPUT );

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
  
  root["TTKotel"] =" v0.3 ";
  root["pressure"] = getPressureData();
  root["tempSmoke"] = getPT100Data();
  root["tempTTOut"] = getPT1000Data();
  root["L/min"] = getFlowData();
//  root["litersTotal"] = getFlowData();
  root["tempTTinIndx"] = sensorsDS.getTempCByIndex(0);
  root["tempInverseIndx"] = sensorsDS.getTempCByIndex(1);

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
  int aA0 = analogRead(A0);
  
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
float getPT100Data() {

  float koefB = 0.3850; // B-коэффициент
  int nominalR = 1025; // сопротивление дополнительного резистора
  int data0PT100 = 100; // сопротивления PT100 при 0 градусах

  int valuePT100 = analogRead(A1);
  Serial.print("  valuePT100 = ");
  Serial.print(valuePT100);

//   Rpt=(float)( A1 * R / (1023 - A1 ));
  float resistancePT100 = 1023 - valuePT100; 
  resistancePT100 = (float)(1.0 / resistancePT100);
 // resistancePT100 /= (float)(resistancePT100);
  resistancePT100 *= valuePT100;
  resistancePT100 *= nominalR;
  Serial.print("  resistancePT100 = ");
  Serial.print(resistancePT100);

  //tempCpt = ((Rpt-100) / 0.385 );
  float tempPT100 = resistancePT100;
  tempPT100 -= data0PT100;
  tempPT100 /= koefB;
  Serial.print("   tempPT100 = ");
  Serial.println(tempPT100);

  return tempPT100;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
            function to measurement temperature PT1000  
\*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
float getPT1000Data() {

 float koefB = 0.3850; // B-коэффициент
  int nominalR = 1505; // сопротивление дополнительного резистора
  int data0PT1000 = 1000; // сопротивления PT1000 при 0 градусах

  int valuePT1000 = analogRead(A2);
 // valuePT1000 -= 100;
  Serial.print("  valuePT1000 = ");
  Serial.print(valuePT1000);

//   Rpt=(float)( A1 * R / (1024 - A1 ));
  float resistancePT1000 = 1023 - valuePT1000; 
  resistancePT1000 = (float)(1.0 / resistancePT1000);
 // resistancePT1000 /= (float)(resistancePT1000);
  resistancePT1000 *= valuePT1000;
  resistancePT1000 *= nominalR;
  Serial.print("  resistancePT1000 = ");
  Serial.print(resistancePT1000);

  //tempCpt = ((Rpt-1000) / 0.385 );
  float tempPT1000 = resistancePT1000;
  tempPT1000 -= data0PT1000;
  tempPT1000 /= koefB;
  Serial.print("   tempPT1000 = ");
  Serial.println(tempPT1000);

  return tempPT1000;
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