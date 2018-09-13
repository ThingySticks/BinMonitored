#include <MQTT.h>
#include <MQTTClient.h>
#include <system.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiUdp.h>

#include <SparkFun_VL53L1X_Arduino_Library.h>
#include <vl53l1_register_map.h>

#include <SparkFun_MMA8452Q.h>

#include <Adafruit_BME680.h>
#include <bme680.h>
#include <bme680_defs.h>

#include "Secrets.h"

#define BIN_DAY_LED 14
#define STATUS_LED 12
#define ACCEL_INT 13

// BME680 :  0x76
Adafruit_BME680 bme; 
uint8_t bme680Address = 0x76;

MMA8452Q accel;

VL53L1X distanceSensor;

void setup() {
  pinMode(13, OUTPUT);     
  
  pinMode(BIN_DAY_LED, OUTPUT);
  digitalWrite(BIN_DAY_LED, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  Serial.begin(115200);
  Serial.println("");
  Serial.println("");

  Serial.println("BME680 setup...");
  if (bme.begin(bme680Address)) {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  } else {
    Serial.println("BME680 sensor failed!");
  }

  Serial.println("Accelerometer setup...");
  if (!accel.init()) {
    Serial.println("Accelerometer failed");
  }

  Serial.println("Distance sensor setup...");
  if (!distanceSensor.begin()) {
      Serial.println("Distance sensor failed"); 
  }

  Serial.println();

  Serial.println("Connect WiFi...");
  connectWiFi();

  digitalWrite(STATUS_LED, LOW);
}

void loop() {
  Serial.println("----------------------------------------------------");
  
  // put your main code here, to run repeatedly:
  digitalWrite(BIN_DAY_LED, HIGH);
  delay(250);
  digitalWrite(STATUS_LED, HIGH);
  delay(250);
  digitalWrite(BIN_DAY_LED, LOW);
  delay(250);
  digitalWrite(STATUS_LED, LOW);
  delay(250);

  int battery = analogRead(A0);
  Serial.print("Battery = ");
  Serial.print(battery);
  Serial.println(" ADC");

  if (bme.performReading()) {
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" *C");
  
    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");
  
    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");
  
    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");  
  } else {
    Serial.println("Failed to perform reading :(");
  }

  if (accel.available()) 
  { 
      // First, use accel.read() to read the new variables: 
      accel.read(); 

      // accel.read() will update two sets of variables.  
      // * int's x, y, and z will store the signed 12-bit values  
      //   read out of the accelerometer. 
      // * floats cx, cy, and cz will store the calculated  
      //   acceleration from those 12-bit values. These variables  
      //   are in units of g's. 
      // Check the two function declarations below for an example 
      // of how to use these variables. 

        printCalculatedAccels(); 
        //printAccels(); // Uncomment to print digital readings 
  } else {
    Serial.println("Failed to get accelerometer readings");
  }

  if (distanceSensor.newDataReady() ) {
    printDistance();
  }else {
    Serial.println("Failed to get distance readings");
  }

  distanceSensor.softReset();
  delay(100);
  ESP.deepSleep(20e6); 
}

void printDistance() {
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor 
 
  Serial.print("Distance(mm): "); 
  Serial.print(distance); 
  Serial.println();
}

void printAccels() 
{ 
    Serial.print(accel.x, 3); 
    Serial.print("\t"); 
    Serial.print(accel.y, 3); 
    Serial.print("\t"); 
    Serial.print(accel.z, 3); 
    Serial.print("\t"); 
    Serial.println();
}
  
 
  // This function demonstrates how to use the accel.cx, accel.cy, 
  //  and accel.cz variables. 
  // Before using these variables you must call the accel.read() 
  //  function! 
void printCalculatedAccels() 
{  
    Serial.print(accel.cx, 3); 
    Serial.print("\t"); 
    Serial.print(accel.cy, 3); 
    Serial.print("\t"); 
    Serial.print(accel.cz, 3); 
    Serial.print("\t"); 
    Serial.println();
} 

void connectWiFi() {
  long connectStart = millis();
  
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    timeout++;
    if (timeout>60) {
      Serial.println(" TIMEOUT!");
      return;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  long timeTakenToConnect = millis() - connectStart;
  Serial.print("Took: ");
  Serial.print(timeTakenToConnect, DEC);
  Serial.println(" ms");
}

