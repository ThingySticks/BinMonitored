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

// Set true to force EEPROM reset and use defaults.
#define FORCE_EEPROM_RESET false

// When programming for the first time
// of if the batttery has been flat for a bit
// setup how far from the last bin day we are.
#define BIN_DAY_OFFSET 11

// defined the default bin collection interval
// this is only used on first load (until EEPROM
// configuration is set.
#define DEFAULT_BIN_COLLECTION_INTERVAL 14

#define DEEP_SLEEP_INTERVAL_MILLISECONDS 60000
#define BIN_DAY_LED 14
#define STATUS_LED 12
#define ACCEL_INT 13

#define BME680_DEBUG 1

// BME680 :  0x76
Adafruit_BME680 bme; 
uint8_t bme680Address = 0x76;

MMA8452Q accel;

VL53L1X distanceSensor;

//Address of EEPROM. Base address (A0-A2 grounded -> 0x50)
#define disk1 0x50    

// Fake RTC.
byte day = 0;
byte hour = 0;
byte minute = 0;
byte second = 0;
int binCollectionInterval = 14;

// Measured parameters.

// Number of times the lid has been opened since 
// the last WiFi send.
byte lidOpenedCount = 0;
// If the lid has been opened (set using 
// the interrupt from the accelerometer)
bool lidOpened = false;

// 1: Lid opened (vertical)
// 2: On Side (left)
// 3: On Side (right)
// 64: Bin vertical, Lid closed, or lid fully back horizontal.
byte orientation = 0;
byte lastOrientation = 0;

float xAcceleration;
float yAcceleration;
float zAcceleration;
bool hasAccelerationMeasurement;

// Environmental
unsigned long bmeEndTime ;
bool bme680Fault = false;
bool hasEnvironmentMeasurement = false;
float temperature;
float humidity;
float pressure;

unsigned long gasStart;
int gasMeasureTime;
int rawGas;

// Distance
bool hasDistance;
int rawDistance;
int percentageFull;
int distanceSignalRate ;
byte distanceRangeStatus;

// Battery
float batteryVoltage;

// WiFi quality
int rssi;
int wiFiTimeTakenToConnect;
int cloudTimeTakenToConnect;

// Cloud
bool shouldConnect = false;
bool cloudConnected = false;

bool measurementsSent;

// EEPROM settings.
int configurationBaseAddress = 10;

void setup() {   
  pinMode(ACCEL_INT, INPUT_PULLUP);
    
  pinMode(BIN_DAY_LED, OUTPUT);
  digitalWrite(BIN_DAY_LED, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // TODO: Only use serial during debugging.
  Serial.begin(115200);
  Serial.println("");
  Serial.println("====================================================");
  
  bool shouldSleep = true;

  Wire.setClock(400000);
  Wire.begin();

  // Read the configuration early.
  readConfiguration();
  
  // Check accelerometer interrupt to see
  // if something interesting has happened.
  if (hasAccelerometerInterrupt()) {
    // Handle accelerometer event...
    // most likely lid opened.
    // Motion setting lost when accelerometer is initialised.
    shouldSleep = false;
    lidOpenedCount++;
    // Cap at 100 to keep the number within a byte
    // so we don't overflow to 0.
    if (lidOpenedCount>100) {
      lidOpenedCount = 100;
    }
    // Only connect/send on the first open.
    shouldConnect = lidOpenedCount == 1;
    lidOpened = true;
  } else {
    // Lid closed, reset the counter.
    lidOpenedCount = 0;
  }

  // read the RTC value from EEPROM, update it based on wake interval
  // then write it back to keep track of time in a really
  // not very good way...
  readRtc();
  updateRtc();
  writeRtc();

  // Check to see if the bin status should be checked.
  // this happens on a periodic interval.
  if (shouldCheckBin()) {
    shouldSleep = false;
  }

  // Check if the bin status should be sent
  // again, this happens periodically.
  if (shouldUploadStatus()) {
    shouldSleep = false;
    shouldConnect = true;
  }

  // If nothing to do put the micro back to sleep 
  // as soon as possible to 
  if (shouldSleep) {
    Serial.println("Nothing to do. Sleeping.");
    deepSleep();
    // ESP will be reset to wake up so nothing after this happens.
  }

  // --------------------------------------------
  // The below only gets called if BinMonitored
  // has decided something needs to be monitored.
  // --------------------------------------------

  setupAccelerometer();

  setupBme680();

  // By definition, day 0 is bin collection day
  // and hence day == binCollectionInterval is the 
  // day before bin day, so indicate that the bin
  // should be put out for collection.
  if (isBinDay()) {
    digitalWrite(BIN_DAY_LED, HIGH);
  } else {
    // Use bin day LED to indicate setup in progress
    // it should blink quickly.
    digitalWrite(BIN_DAY_LED, LOW);
  }
}


// Loop only happens once, but holds the measurements.
void loop() {
  Serial.println("--------------------------------------------------------------------------"); 

  measureAcceleration();

  measureBattery();

  measureEnvironment();

  measureGas();
  
  // Need Z orientation first to determine if 
  // the laser can should enabled.
  setupDistanceSensor();

  // Measure after env. to allow the sensor 
  // time to measure the distance.
  measureBinFullness();

  Serial.println("--------------------------------------------------------------------------"); 

  sleepEnvironmentalSensor();
  sleepDistanceSensor();
  sleepAccelerometer();   
 
  // Only connect to the internet if we need to.
  // otherwise the radio should be put in low power/off mode
  // whilst 
  if (shouldConnect) {
    connectToCloud();
    measureRSSI();
    printWiFi();
  } 

  if (lastOrientation != orientation) {
    updateForOrientation();
  }

  // Only on first instance.
  if (lidOpenedCount == 1) {
    publishTinamousStatus("Lid has been opened.");
  }

  if (shouldUploadStatus()) {
    sendMeasurements();
  }

  if (cloudConnected) {
    // short delay to ensure the WiFi
    // data has been sent before going to deep sleep.
    // Handle MQTT updates whilst we wait.
    for (int i = 0; i<10; i++) {
        mqttLoop();
        delay(100);
    }
  }

  writeConfiguration();
  
  // Trun off the status LED now we've done the work.
  digitalWrite(STATUS_LED, LOW);

  // If it's bin day, and the lid is open
  // then stay awake for longer to show the bin day LED
  if (lidOpened && isBinDay()) {
    
    // Handle MQTT updates, a
    // and maybe check for a new firmware
    // version whilst we're waiting.
    for (int i = 0; i<200; i++) {
        mqttLoop();
        delay(100);
    }
  }

  if (cloudConnected) {
    disconnectFromMqtt();
    delay(500);
  }
 
  deepSleep();
}

void deepSleep() {
  Serial.print("Processing took :");
  Serial.print(millis());
  Serial.println(" ms");
  
  // Sleep interval 1 minute.
  // Can be longer to save power on deployed
  // but notifications (bin over/lid opened/etc)
  // will be delayed so not too long.
  long sleepInterval = DEEP_SLEEP_INTERVAL_MILLISECONDS;
  
  // Reduce the sleep interval by the time we've been running
  // to /try/ and keep the interval uniform.
  sleepInterval -= millis();
  
  // Ensure a minimum of 1 second sleep
  // During firmware update etc might
  // have been awake longer than 1s.
  if (sleepInterval< 1000) {
    sleepInterval = 1000;
  }
  
  Serial.print("Deep Sleeping for: ");
  Serial.print(sleepInterval);
  Serial.println(" ms");
  
  // To keep the wake cycle once per minute, this needs
  // to be 60s - how long it's taken to get here (millis();
  ESP.deepSleep(sleepInterval * 1000); 
}

void updateForOrientation() {
  if (orientation != 0) {
    // Pulish a status message to Tinamous to indicate that 
    // the bin has fallen over.
  
    switch (orientation) {
      case 0:
      case 1:
        publishTinamousStatus("BIN DOWN. Your bin is on its side.");
        break;
      case 2:
        publishTinamousStatus("BIN DOWN. Your bin is on its front.");
        break;
      case 3:
        publishTinamousStatus("BIN DOWN. Your bin is on its back.");
        break;        
      default:
        publishTinamousStatus("BIN DOWN!!!");
        break;
    }
  } else {
    publishTinamousStatus("Bin restored.");
    // Although the lid might be flipped back.
  }
  
  // Store the orientation so we don't keep sending bin down messages.
  writeOrientation();
}

// ================================================================
// Configuratgion
// ================================================================
void readConfiguration() {
  Serial.print("Configuration version: ");
  byte configurationVersion = readEEPROM(disk1, 0);
  
  if (configurationVersion == 255 || FORCE_EEPROM_RESET) {
    Serial.println("Not set or fault EEPROM. Using defaults.");   

    lastOrientation = 0;
    binCollectionInterval = DEFAULT_BIN_COLLECTION_INTERVAL;
    measurementsSent = false;
    return;
  }

  Serial.println(configurationVersion);   

  lastOrientation = readEEPROM(disk1, configurationBaseAddress);
  binCollectionInterval = readEEPROM(disk1, configurationBaseAddress+1);
  measurementsSent = readEEPROM(disk1, configurationBaseAddress+2) == 0  ? false : true;
  lidOpenedCount  = readEEPROM(disk1, configurationBaseAddress+3);  
}

void writeConfiguration() {
  // Byte  0: configuration versionining.
  writeEEPROM(disk1, 0, 0x01) ;
  // Configuration values.
  writeEEPROM(disk1, configurationBaseAddress, (byte)orientation) ;
  writeEEPROM(disk1, configurationBaseAddress+1, binCollectionInterval) ;
  writeEEPROM(disk1, configurationBaseAddress+2, measurementsSent ? 1 : 0) ;
  writeEEPROM(disk1, configurationBaseAddress+3, lidOpenedCount); // LSB - max value is 100.
}

void writeOrientation() {
  //configurationBaseAddress
  writeEEPROM(disk1, configurationBaseAddress, (byte)orientation) ;
}

// ================================================================
// Distance / bin fullness sensors.
// ================================================================

bool distanceSensorDisabled = false;

void setupDistanceSensor() {

  // If the distance sensor is not pointing down
  // then disable to avoid firing
  // the laser at people.
  /*
  if (zAcceleration > -0.8) {
    distanceSensorDisabled = true;
    Serial.println("Skipping distance as bin is not vertical.");
    return;
  }
  */
  
  Serial.println("Distance sensor setup...");
  if (!distanceSensor.begin()) {
    distanceSensorDisabled = true;
    Serial.println("Distance sensor failed"); 
    return;
  }

  uint8_t shortRange = 0; // 136cm
  //uint8_t midRange = 1; // 290cm
  //uint8_t longRange = 2; // 360cm

  // Call setDistanceMode with 0, 1, or 2 to change the sensing range.
  // use short range (0..) as it uses less power and takes less time.
  distanceSensor.setDistanceMode(shortRange);
}

void measureBinFullness() {

  // Don't meaure if the distance sensor
  // is disabled (fault or lid open).
  if (distanceSensorDisabled) {
    Serial.println("Skipping distance as the sensor is disabled.");
    hasDistance = false;
    return;
  }

  hasDistance = distanceSensor.newDataReady();
  
  if (!hasDistance) {
    // Wait for a short time to allow 
    // the distance sensor to measure.
    delay(200);
    hasDistance = distanceSensor.newDataReady();
  }

  if (hasDistance) {
    rawDistance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor 

    // https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/examples/Example3_StatusAndRate/Example3_StatusAndRate.ino
    distanceSignalRate = distanceSensor.getSignalRate();
    distanceRangeStatus = distanceSensor.getRangeStatus();
    

    // HACK: Assume bin is 1 meter tall (capacity).
    // space (distance to nearest object) / capacity => percentage empty.
    // total space - distance to nearest -> space used.
    percentageFull = ((1000 - rawDistance) / 1000.0) * 100;
    
    printBinFullness();  
  } else {
    Serial.println("*** Distance sensor not ready ***");
  }
}

void printBinFullness() {
  Serial.print("\t Distance: "); 
  Serial.print(rawDistance); 
  Serial.print(" mm"); 
  
  Serial.print("\t Fullness: "); 
  Serial.print(percentageFull); 
  Serial.print(" %"); 
  Serial.println();

  Serial.print("\t Signal rate: ");
  Serial.print(distanceSignalRate);
    
  Serial.print("\t Range Status: ");
  Serial.print(distanceRangeStatus);

  Serial.println();
}

void sleepDistanceSensor() {
  if (distanceSensorDisabled) {
    return;
  }
  
  distanceSensor.softReset();
}

// ================================================================
// Lid and orientation sensing.
// ================================================================

// Determine through the ACCEL_INT pin read
// if the accelerometer has an event we are interested in.
// e.g. motion change
// this allows for a quick exit if nothing has changed
// without reading the sensors.
bool hasAccelerometerInterrupt() {
  bool interrupt1Input = !digitalRead(ACCEL_INT);
  digitalWrite(STATUS_LED, interrupt1Input);
  
  if (interrupt1Input) {
    Serial.println("INT1 - Accelerometer event occured (Lid opened).");
  } else {
    Serial.println("No interupt");
  }

  return interrupt1Input;
}

void setupAccelerometer() {
  
  Serial.println("Setup Accelerometer");
  if (!accel.init(SCALE_8G, ODR_400)) {
    Serial.println("Accelerometer failed");
  }

  // Accelerometer needs to be in standby mode
  // to be configured.
  accel.standby();

  // 0.063 g/LSB for threshold
  // Look for > 6.3g on the Y axis due to gravity, this 
  // happens when the lid is lifted.
  // Enable an interrupt so we can tell when 
  // waking the processor that the lid was lifted.
  byte threshold = 10;
  Serial.print("Motion Threshold: ");
  Serial.println(threshold * 0.063, DEC);

  // Y Axis only for lid opening.
  // latching.
  accel.setupMotion(false, true, false, true, threshold);

  // Use tap (Pulse) detection to get an event
  // when the lid is closed.
  // X/Y disabled, Tap detect in Z axis only
  // accel.setupTap(0x80, 0x80, 4);

  // Setup interrpt so that...
  // Source = freefall/motion (0b 0000 0100)
  //byte source = 0x04; // Motion
  //byte source = 0x08; // Pulse
  //byte source = 0x10; // Landscape/Portrate (not firing for some reason).
  byte source = 0x04; // Motion
  
  // Pin for any interrupt is INT1 (0b 1011 1101)
  // DO NOT USE INT2 as it is wired to reset
  // and interrupts are not cleared until read!
  byte pin = 0xBD;
  
  // Interrupt doesn't wake the system.
  bool wake = false;
  
  // interrupt is active low
  bool activeHigh = false;
  
  // open drain to allow multiple sources
  // because reset pin is INT2 we need to use open 
  // drain rather than push/pull.
  bool openDrain = true;
  
  accel.setupInterrupts(source, pin, wake, activeHigh, openDrain);

  // Now the accelerometer has been configured
  // we can run it.
  accel.active();
}


// Checks to see if the lid has been opened.
// once the check is complete lidOpened variable
// should be used as the accelerometer interrupt
// will have been cleared.
bool hasLidBeenOpened() {
  byte motion = accel.readMotion();
  
  if (motion > 0) {
    /*
    // Z 0b 00 11 0000
    if (motion & 0x20) {
      if (motion & 0x10) {
        Serial.print("-");
      }
      Serial.print("Z ");
    }
    */

    // Y 0b 0000 11 00
    if (motion & 0x08) {
      if (motion & 0x04) {
        Serial.print("-");
      }
      Serial.print("Y ");

      // Y axis motion caused by the lid being up.
      return true;
    }

    /*
    // X 0b 0000 00 11
    if (motion & 0x02) {
      if (motion & 0x01) {
        Serial.print("-");
      }
      Serial.print("X ");
    }
    */    
  } else {
    Serial.println("No Motion");
  }
  
  return false;
}

void measureAcceleration() {

  // Note: Lid opened checking is done as part of every startup

  // TODO: Is the lid open now?

  delay(300);
  hasAccelerationMeasurement = accel.available();
  if (hasAccelerationMeasurement) { 
    accel.read(); 

    // floats cx, cy, and cz will store the calculated  
    //   acceleration from those 12-bit values. These variables  
    //   are in units of g's. 
    xAcceleration = accel.cx;
    yAcceleration = accel.cy;
    zAcceleration = accel.cz;

    orientation = computeBinOrientation();

    // accel.read() will update two sets of variables.  
    // * int's x, y, and z will store the signed 12-bit values  
    //   read out of the accelerometer. 
    //
    // Check the two function declarations below for an example 
    // of how to use these variables. 

    printCalculatedAccels(); 
  } else {
    Serial.println("Failed to get accelerometer readings");
  }
}

// Compute the orientation of the bin using the accelerometer
// 0: Vertical
// 1: On Side
// 2: On front
// 3: On back
int computeBinOrientation() {
  // If the bin is at-least 80% of gravity in the Z axis assume it's vertical.  
  if (zAcceleration < -0.8) {
    // Vertical
    return 0;
  }

  // If X axis > 0.5G then it's probably 45° on the X axis so 
  // assume it's on its side.
  if (xAcceleration > 0.5 || xAcceleration < -0.5) {
    return 1;
  }

  // Y acceleration may be because the lid is opened, or because the
  // lid is open (vertical).
  // Lid vertical (open) ~ 1G - ignore here.
  // Lid all the way over ~ -1G
  // 
  if (yAcceleration < -0.5) {
    return 2;
  }

  // If the zAcceleration is +ve then the lid is either open 
  // or the bin has fallen on its back and the lid is flat on the floow.
  if (zAcceleration > 0.5 ) {
    return 3;
  }

  if (yAcceleration > 0.5) {
    lidOpened = true;
    // Return bin OK, but assume the lid is open.
    return 0;
  }

  // Error, unknown condition
  return 255;
}

// This function demonstrates how to use the accel.cx, accel.cy, 
// and accel.cz variables. 
void printCalculatedAccels() 
{  
    Serial.print("\t X: "); 
    Serial.print(xAcceleration, 3); 
    Serial.print("\t Y: "); 
    Serial.print(yAcceleration, 3); 
    Serial.print("\t Z: "); 
    Serial.print(zAcceleration, 3);    
    Serial.print("\t Orientation: ");
    Serial.println(orientation);
} 

void sleepAccelerometer() {
  //accel.init(SCALE_2G, ODR_1);
  accel.standby();
  accel.setODR(ODR_50);
  accel.active();
}

// ================================================================
// WiFi / Cloud
// ================================================================

void connectToCloud() {
  if (!cloudConnected) {
    long connectStart = millis();
    
    connectWiFi();
    setupSenML();
    setupMqtt();
    cloudConnected = true;

    cloudTimeTakenToConnect = millis() - connectStart;
  }
}


void connectWiFi() {
  long connectStart = millis();
  
  Serial.println("Connect WiFi...");
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
    delay(250);
    Serial.print(".");
    timeout++;
    if (timeout>40) {
      Serial.println(" TIMEOUT!");
      return;
    }
  }

  Serial.println("");
  wiFiTimeTakenToConnect = millis() - connectStart;
}

void measureRSSI() {
  rssi = WiFi.RSSI();
}

void printWiFi() {
  Serial.println("WiFi connected");  
  Serial.print("\t IP address: ");
  Serial.print(WiFi.localIP());
  
  Serial.print("\t Took: ");
  Serial.print(wiFiTimeTakenToConnect, DEC);
  Serial.print(" ms");

  Serial.print("\t RSSI: ");
  Serial.print(rssi, DEC);
  Serial.print(" ms");

  Serial.print("Cloud Took: ");
  Serial.print(cloudTimeTakenToConnect, DEC);
  Serial.print(" ms");
  
  Serial.println();
}

// ================================================================
// Battery Monitor
// ================================================================

void measureBattery() {
  int batteryLevel = analogRead(A0);
  // Battery goes via a 39k + 10k divider to give approx 1/5th of 
  // the actual battery voltage, this then translates to 1024 bits 
  // over 1 volt (i.e. 1 but is approx 1mV, which is approx 5mv for 
  // the battery).
  batteryVoltage = (5 * batteryLevel) / 1000.0; 

  printBatteryMeasurements();
}

void printBatteryMeasurements() {
  Serial.print("\t Batt: ");
  Serial.print(batteryVoltage);
  Serial.println(" mV");
}

// ================================================================
// BME680 Environmental measurements;
// ================================================================

void setupBme680() {
  Serial.println("BME680 setup...");
  if (bme.begin(bme680Address)) {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_2X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_2X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_15);

    // Disable the gas sensor for initial reading.
    // to get a fast temperature and remove
    // heating due to ESP module.
    bme.setGasHeater(0, 0); 
    
    bme680Fault = false;
    bmeEndTime = bme.beginReading();
  } else {
    Serial.println("BME680 sensor failed!");
    bme680Fault = true;
  }
}


void measureEnvironment() {

  if (bme680Fault) {
    return;
  }

  if (bmeEndTime == 0) {
    Serial.println("BME error beginning reading");
    return;
  }

  // For temperature/Humidity/Pressure only
  // we should not need to delay.
  while (millis() < bmeEndTime) {
    delay(5);
    Serial.println("Waiting for bme...");
  }

  hasEnvironmentMeasurement = bme.endReading();
  
  if (hasEnvironmentMeasurement) {
    temperature = bme.temperature ;
    humidity = bme.humidity;
    pressure = bme.pressure / 100.0;
    
    printEnvironmentalMeasurements();
  } else {
    Serial.println("Failed to perform reading :(");
  }
}

void beginGasMeasurements() {
  if (bme680Fault) {
    return;
  }

  gasStart = millis();
  
  // Gas reading may be unstable, so we need to try a few times.
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  bmeEndTime = bme.beginReading();
  //Serial.print("BME680 delay: ");
  //Serial.print(bmeEndTime - millis());
  //Serial.println("ms");
}

void measureGas() {
  if (bme680Fault) {
    return;
  }

  beginGasMeasurements();

  rawGas = getGasReading(5);
  
  printGasMeasurements(); 
}

int getGasReading(int maxRetries) {
    
  for (int i = 0; i<maxRetries; i++) {    
    // force a new reading, this blocks.
    bme.performReading();
    
    rawGas = bme.gas_resistance;
    
    if (rawGas != 0) {
      gasMeasureTime = millis() - gasStart;      
      return rawGas;
    }   
  }
  
  Serial.println("BME680 ges reading unstable. Giving up....");
  return 0;
}

void printEnvironmentalMeasurements() {
    Serial.print("\t T: ");
    Serial.print(temperature);
    Serial.print(" *C");
  
    Serial.print("\t P: ");
    Serial.print(pressure,1);
    Serial.print(" hPa");
  
    Serial.print("\t\t RH: ");
    Serial.print(humidity);
    Serial.print(" %");
    Serial.println();
}

void printGasMeasurements() { 
    Serial.print("\t Gas: ");
    // rawGas = 0 indicates unstable readings.
    Serial.print(rawGas / 1000.0);
    Serial.print(" K"); 

    Serial.print("\t Took:");
    Serial.print(gasMeasureTime);
    Serial.print(" ms");    
    
    Serial.println();
}

void sleepEnvironmentalSensor() {
  // Nothing to do. However we may wish to 
  // keep the gas sensor heating going as 
  // it takes time to stabalise....
  bme.setGasHeater(0, 0); // 320*C for 150 ms
  bme.setTemperatureOversampling(BME680_OS_NONE);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_NONE);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
}

