#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//#include <Servo.h>
Servo servo1;

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

#define servoPin 33
#define Puzzle_Input 16
#define Lock 21
#define Door 22
#define Handle_Input 17
boolean Puzzle_Solved=false;
boolean Handle_Up=false;

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

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Tomb_Entrance_Esp32_OTA");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
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
      client.publish("/Egypt/Tomb/", "Tomb Door (Archive Entrance) Online");
      // ... and resubscribe
      client.subscribe("/Egypt/Tomb/Entrance/");
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
  if (message == "open") {
    digitalWrite(Lock, LOW);
    digitalWrite(Door, LOW);
    checkConnection();
    client.publish("/Egypt/Tomb/","Entrance Opened");
  }
  else if (message == "close"){
    digitalWrite(Door, HIGH);
    wait(3750);
    digitalWrite(Lock, HIGH);
    checkConnection();
    client.publish("/Egypt/Tomb/","Entrance Closed");
  }
  else if (message == "reset" && digitalRead(Puzzle_Input) == LOW){
      client.publish("/Egypt/Tomb/","Switch puzzle needs physical reset");
      }
  else if (message == "reset" && digitalRead(Handle_Input) == LOW){
      client.publish("/Egypt/Tomb/","Handle needs physical reset");
      }
      
  else if (message == "reset"){
    digitalWrite(Door, HIGH);
    wait(3750);
    digitalWrite(Lock, HIGH);
    checkConnection();
    Handle_Up=false;
    Puzzle_Solved=false;
    //servoWrite(70);//Set to proper position
    client.publish("/Egypt/Tomb/","Entrance Closed, Switch Box Reset");
  }
  
}

void setup() {
  /*
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  */
  Serial.begin(115200);
  //servoWrite(70);
  pinMode(Puzzle_Input, INPUT_PULLUP);
  pinMode(Handle_Input, INPUT_PULLUP);
  pinMode(Lock, OUTPUT);
  pinMode(Door, OUTPUT);
  
  digitalWrite(Lock, HIGH);
  digitalWrite(Door,HIGH);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

void checkConnection() {
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
  }
}
void servoWrite(int microseconds) {
  servo1.attach(servoPin,1000, 2000);//myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  servo1.write(microseconds);
  wait(650);
  servo1.detach();
}
void loop() {
  
  if (digitalRead(Puzzle_Input) == LOW && Puzzle_Solved==false) {
    Puzzle_Solved=true;
    //servoWrite(150);
    client.publish("/Egypt/Tomb/","Switch Puzzle Solved");
  }
  if (digitalRead(Handle_Input) == LOW && Handle_Up==false) {
    Handle_Up=true;
    client.publish("/Egypt/Tomb/","Handle Up");
    
  }
  
  
  
  ArduinoOTA.handle();
  checkConnection();
  client.loop();
}
