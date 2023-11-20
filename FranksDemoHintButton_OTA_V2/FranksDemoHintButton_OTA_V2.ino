#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#ifndef STASSID
#define STASSID "Control booth"
#define STAPSK  "MontyLives"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;
const char* mqtt_server = "192.168.86.101";

#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"

bool hintR=false;        //True when hint requested via Hint button in room
bool flashHint=false;    //True when hint has been requested and not yet given
bool mqttHint=false;     //True when operator wants to suggest a hint resets to false after 10 seconds of flashing
bool estop=false;        //True when Estop button pressed down
unsigned long previousMillis = 0;
unsigned long interval = 30000;
WiFiClient wifiClient;

PubSubClient client(wifiClient); 

void setup_wifi() { 
  /*
   * Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
*/
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  /*
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  */
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("DemoHintButton_OTA");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Demo/", "Demo Hint Button Online");
      // ... and resubscribe
      client.subscribe("/Egypt/Demo/");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte *payload, unsigned int length) {
  payload[length] = '\0';
  String message = (char*)payload;
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
  if (message == "hintSuggested") {
    checkConnection();
    client.publish("/Egypt/Demo/","Mqtt hint suggested");
    mqttHint=true;
  }
  else if (message == "on"){
    
    //digitalWrite(D6, LOW);
    digitalWrite(D4, LOW);
    checkConnection();
    client.publish("/Egypt/Demo/","Hint button light is on");
    flashHint=false;
    mqttHint=false;
    hintR=false;
  }
   else if (message == "off"){
    
    //digitalWrite(D6, HIGH);
    digitalWrite(D4,HIGH);
    checkConnection();
    client.publish("/Egypt/Demo/","Hint button light is off");
    flashHint=false;
    mqttHint=false;
    hintR=false;
  }
}

void setup() {
  Serial.begin(115200);
  //pinMode (D1, INPUT_PULLUP);  //Estop input. pull to ground when pressed
  pinMode(D5, INPUT_PULLUP);   //Hint button input. pull to ground when pressed
  //pinMode(D7, INPUT_PULLUP);   //Hint button input. pull to ground when pressed
  //pinMode(D6, OUTPUT);         //Hint button LED output1
  pinMode(D4, OUTPUT);         //Hint button LED output2
  //digitalWrite(D6,HIGH);       //Button LED's default to off
  digitalWrite(D4,HIGH);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

void checkConnection() {
  unsigned long currentMillis = millis();
// if WiFi is down, try reconnecting
if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
  Serial.print(millis());
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  WiFi.reconnect();
  previousMillis = currentMillis;
}
  if (!client.connected()) {
    Serial.println("Lost connection");
    reconnect();
  }
}
void wait(uint16_t msWait)
{
  uint32_t start = millis();

  while ((millis() - start) < msWait)
  {
    client.loop();
    checkConnection();
    readButtons();
  }
}

void flash() {
  while (flashHint){
    digitalWrite(D6,HIGH);
   digitalWrite(D4,HIGH);   
   wait(1000);
   if (!flashHint){
    break;
    }
    digitalWrite(D6,LOW);
   digitalWrite(D4,LOW);   
   wait(1000);
   if (!flashHint){
    break;
    }
   ArduinoOTA.handle();
    }
  }
  void mqttFlash() {
    int i=1;
  while (mqttHint && i<40){
    //digitalWrite(D6,HIGH);
   digitalWrite(D4,HIGH);   
   wait(125);
   if(!mqttHint){
    break;
    }   
   //digitalWrite(D6,LOW);
   digitalWrite(D4,LOW);   
   wait(125);
   i++;
   ArduinoOTA.handle();
   if(!mqttHint){
    break;
    }
    }
    mqttHint=false;
  }
  
void readButtons(){
  if (digitalRead (D5)==LOW && !hintR){
    hintR=true;
    client.publish("/Egypt/Demo/","Demo hint button pressed");
    mqttHint=false;
    flashHint=true;
    flash();
    }
    
    if (digitalRead (D1)==LOW && !estop){
    estop=true;
    client.publish("/Egypt/Demo/","Demo Estop button pressed");
    }
  }
  
void loop() {
  ArduinoOTA.handle();
  checkConnection();
  client.loop();
  readButtons();
  mqttFlash();
  
}
