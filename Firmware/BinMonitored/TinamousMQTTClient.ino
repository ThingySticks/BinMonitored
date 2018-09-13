#include <MQTTClient.h>
#include <system.h>

// Be sure to use WiFiSSLClient for an SSL connection.
// for a non ssl (port 1883) use regular WiFiClient.
WiFiClient networkClient; 
//WiFiSSLClient networkClient; 

// MQTT Settings defined in secrets.h
// Set buffer size as the default is WAY to small!.
MQTTClient mqttClient(4096); 

// If we have been connected since powered up 
bool was_connected = false;
String senml = "";
unsigned long nextSendMeasurementsAt = 0;

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
 
  Serial.println("Connected!"); 

  // Subscribe to status messages sent to this device.
  mqttClient.subscribe("/Tinamous/V1/Status.To/" DEVICE_USERNAME); 

  // Subsribe to all of the command's for this device.
  //mqttClient.subscribe("/Tinamous/V1/Commands/" DEVICE_USERNAME "/#");
  
  // Say Hi.
  // for the first connect, give a "Hello" message.
  publishTinamousStatus("Hello World!");

  was_connected = true;
  return true;
} 


// ===============================================
// Pubish a status message to the Tinamous MQTT
// topic.
// ===============================================
void publishTinamousStatus(String message) {
  Serial.println("Status: " + message);
  mqttClient.publish("/Tinamous/V1/Status", message); 

  if (mqttClient.lastError() != 0) {
    Serial.print("MQTT Error: "); 
    Serial.print(mqttClient.lastError()); 
    Serial.println(); 
  }

  if (!mqttClient.connected()) {
    Serial.println("Not connected after publishing status. What happened?");
  } else {
    Serial.println("Status message sent.");
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
  // Call anyway, does nothing if already connected.
  connectToMqttServer();

  // Check inbound and keep alive.
  mqttClient.loop(); 

  // Send measurements (if the time interval is appropriate).
  //sendMeasurements();
}

// =========================================================================================

// ===============================================
// Process messages received from the MQTT server
// ===============================================
void messageReceived(String &topic, String &payload) { 
  Serial.print("Message from Tinamous:"); 
  Serial.println("Topic: " + topic + " - " + payload); 
}

