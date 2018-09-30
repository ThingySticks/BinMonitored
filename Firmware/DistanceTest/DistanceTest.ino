#include <SparkFun_VL53L1X_Arduino_Library.h>
#include <vl53l1_register_map.h>

#define BIN_DAY_LED 14
#define STATUS_LED 12

VL53L1X distanceSensor;

// Distance
int rawDistance;
int percentageFull;
int distanceSignalRate ;
byte distanceRangeStatus;

void setup() {
  pinMode(BIN_DAY_LED, OUTPUT);
  digitalWrite(BIN_DAY_LED, HIGH);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  Serial.begin(115200);
  delay(2000);

  setupDistanceSensor();
}

void loop() {
  // Measure after env. to allow the sensor 
  // time to measure the distance.
  measureBinFullness();
}


void setupDistanceSensor() {

  // If the distance sensor is not pointing down
  // then disable to avoid firing
  // the laser at people.
  if (zAcceleration > -0.8) {
    distanceSensorDisabled = true;
    Serial.println("Skipping distance as bin is not vertical.");
    return;
  }
  
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
 
  bool hasDistance = distanceSensor.newDataReady();
  
  if (!hasDistance) {
    // Wait for a short time to allow 
    // the distance sensor to measure.
    Serial.println("Waiting for distance");
    delay(50);
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

