#include <MQTTClient.h>
#include <system.h>

// Be sure to use WiFiSSLClient for an SSL connection.
// for a non ssl (port 1883) use regular WiFiClient.
WiFiClient networkClient; 
//WiFiSSLClient networkClient; 

// MQTT Settings defined in secrets.h
// Set buffer size to ensure we have space for messages.
MQTTClient mqttClient(1024); 

// converted to lower case in setup.
String lowerDeviceAtName = "@" DEVICE_USERNAME;

// ===============================================
// Setup the connection to the MQTT server.
// ===============================================
bool setupMqtt() {
  senml.reserve(4096);
  lowerDeviceAtName.toLowerCase();
  
  Serial.print("Connecting to Tinamous MQTT Server on port:");
  Serial.println(MQTT_SERVERPORT);
  mqttClient.begin(MQTT_SERVER, MQTT_SERVERPORT, networkClient);

  // Handle received messages.
  mqttClient.onMessage(messageReceived);

  connectToMqttServer();
}

// ===============================================
// Connect/reconnect to the MQTT servier.
// This may be called repeatedly and does nothing
// if already connected.
// ===============================================
bool connectToMqttServer() { 
  if (mqttClient.connected()) {
    return true;
  }

  Serial.println("checking wifi..."); 
  if (WiFi.status() != WL_CONNECTED) { 
    Serial.print("WiFi Not Connected. Status: "); 
    Serial.print(WiFi.status(), HEX); 
    Serial.println();
    
    //WiFi.begin(ssid, pass);
    //delay(1000); 
    return false;
  } 
 
  Serial.println("Connecting to MQTT Server..."); 
  if (!mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) { 
    Serial.println("Failed to connect to MQTT Server."); 
    Serial.print("Error: "); 
    Serial.print(mqttClient.lastError()); 
    Serial.print(", Return Code: "); 
    Serial.print(mqttClient.returnCode()); 
    Serial.println(); 

    if (mqttClient.lastError() == LWMQTT_CONNECTION_DENIED) {
      Serial.println("Access denied. Check your username and password. Username should be 'DeviceName.AccountName' e.g. MySensor.MyHome"); 
    }
    
    delay(10000); 
    return false;
  } 
 
  Serial.println("MQTT Connected!"); 

  // Only subscribe a few times during the hour
  // to prevent RTC correction of time causing 
  // 00:00:xx -> 25:59:xx and triggering a second day addition
  // this also reduces WiFi activity and saves battery power
  if (minute > 10 && minute < 50) {
    // Subscribe to status messages sent to this device.
    mqttClient.subscribe("/Tinamous/V1/Status.To/" DEVICE_USERNAME); 
  
    // Subsribe to all of the command's for this device.
    //mqttClient.subscribe("/Tinamous/V1/Commands/" DEVICE_USERNAME "/#");
    mqttClient.subscribe("/Tinamous/V1/Time");
  }
  
  return true;
} 

bool disconnectFromMqtt() {
  return mqttClient.disconnect();
}


// ===============================================
// Send measurements (Temperature/RH/etc) to the MQTT server
// ===============================================
void sendMeasurements() {
  if (!mqttClient.connected()) {
    Serial.println("Not connected, not sending measurements at this time.");
    return;
  }

  // Send the measurements
  Serial.println("Sending sensor measurements to Tinamous");

  beginSenML();

  // TODO: Clear this after successful send.
  appendIntSenML("lidOpenedCount", lidOpenedCount);

  if (hasAccelerationMeasurement) {
    appendFloatSenML("X", xAcceleration);
    appendFloatSenML("Y", yAcceleration);
    appendFloatSenML("Z", zAcceleration);
    // 0: Vertical, 1: Side, 2: Front, 3: Back.
    appendIntSenML("Orient", orientation);
  }

  // Environmental
  if (hasEnvironmentMeasurement) {
    appendFloatSenML("T", temperature);
    appendFloatSenML("H", humidity);
    appendFloatSenML("P", pressure);
    if (rawGas > 0) {
      appendFloatSenML("GasR", rawGas);
      appendIntSenML("GasTime", gasMeasureTime);
    }
  }

  if (bme680Fault) {
    // TODO: Add fault indicator to measurements.
    appendIntSenML("bme680Fault", 1);
  }
  
  // RSSI (WiFi signal stength)
  appendFloatSenML("rssi", rssi);
  appendIntSenML("wifiConnectTime", wiFiTimeTakenToConnect);
  appendIntSenML("cloudTimeTakenToConnect", cloudTimeTakenToConnect);

  // Battery Voltage
  appendFloatSenML("BV", batteryVoltage);
  
  // Bin fullness...
  if (hasDistance) {
    appendIntSenML("PercentageFull", percentageFull);
    appendIntSenML("Distance", rawDistance);
    appendIntSenML("SignalRate", distanceSignalRate);
    appendIntSenML("RangeStatus", distanceRangeStatus);
  }
  
  char buffer[20]; 
  sprintf(buffer,"%d.%d:%d:%d",day, hour, minute, second);
  appendStringSenML("date", String(buffer));

  sendSenML();

  measurementsSent = true;
}

void sendSenML() {
  // Terminate the json and send the senml to Tinamous
  terminateSenML();
  
  Serial.println("Senml:");
  Serial.println(senml);
  Serial.println();
  Serial.print("Length:");
  Serial.println(senml.length(), DEC);

  if (senml.length() > 4096) {
    Serial.println("*** SENML too long. It will overflow the buffer ***");
  }
  
  publishTinamousSenMLMeasurements(senml);
}

// ===============================================
// Pubish a status message to the Tinamous MQTT
// topic.
// ===============================================
void publishTinamousStatus(String message) {
  if (!mqttClient.connected()) {
    Serial.println("Not connected, not sending status at this time!");
    // Status message might be important (bin down, lid opened etc)
    // so this may need to be handled better.
    return;
  }
  
  Serial.println("MQTT Publish Status Message: " + message);
  mqttClient.publish("/Tinamous/V1/Status", message); 

  if (mqttClient.lastError() != 0) {
    Serial.print("MQTT Error: "); 
    Serial.print(mqttClient.lastError()); 
    Serial.println(); 
  }

  if (!mqttClient.connected()) {
    Serial.println("Not connected after publishing status. What happened?");
  }
}

void publishTinamousSenMLMeasurements(String senml) {
  if (senml.length()> 4096) {
    Serial.println("senml longer than buffer. Ignoring!!!");
    return;
  }
  
  mqttClient.publish("/Tinamous/V1/Measurements/SenML", senml); 
}

// ===============================================
// Loop processor for MQTT functions
// ===============================================
void mqttLoop() {
  // Check inbound and keep alive.
  mqttClient.loop(); 
}

// ===============================================
// Process messages received from the MQTT server
// ===============================================
void messageReceived(String &topic, String &payload) { 
  Serial.print("Message from Tinamous:"); 
  Serial.print("Topic: " + topic + " - " + payload); 
  Serial.println();

  if (topic == "/Tinamous/V1/Time") {
    Serial.print("Time update: ");
    
    // Expect a nicely formatted string like: 
    // 2018-09-17T18:03:51.0497246Z
    int index = payload.indexOf("T");
    if (index < 10) {
      Serial.println("Missing time!");
      return;
    }

    // Note: "Day" in BinMonitor RTC terms is not day of
    // the month, so actual day is ignored.
    String time = payload.substring(index+1);   
    Serial.println(time);
    
    hour = time.substring(0,2).toInt();      
    minute = time.substring(3,5).toInt();      
    second = time.substring(6,8).toInt();   

    printDate();
    
    writeRtc();
  }
}


