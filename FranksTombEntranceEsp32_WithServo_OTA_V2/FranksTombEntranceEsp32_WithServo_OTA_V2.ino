//This code was current as of 11-22-22. Added Watchdog


#include <PubSubClient.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <FastLED.h>
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

#define DATA_PIN    15
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    2
CRGB leds[NUM_LEDS];
#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

#define servoPin 33
#define Puzzle_Input 16
#define Lockout_Input 18
#define Lock 21
#define Door 22
#define Handle_Input 17
#define handleSfx 23
#define switchSfx 14
#define hintButton 32
#define hintLight 27
#define eStop 26
boolean Puzzle_Solved=false;
boolean Handle_Up=false;
unsigned long previousMillis = 0;
unsigned long interval = 10000;
bool hintR=false;        //hint requested via Hint button in room
bool flashHint=false;    //True when hint has been requested and not yet given
bool mqttHint=false;     //True when operator wants to suggest a hint resets to false after 10 seconds of flashing
bool estop=false;
bool handleMoved=false;

WiFiClient wifiClient;

PubSubClient client(wifiClient); 

void setup_wifi() { 
  
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
  ArduinoOTA.setHostname("Tomb_Entrance_Esp32_WithServo_OTAv2");

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
    String clientId = "Tomb_Entrance_Esp32-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Tomb/", "Tomb Door (Archive Entrance) Online");
      // ... and resubscribe
      client.subscribe("/Egypt/Tomb/Entrance/");
      client.subscribe("/Egypt/Tomb/Entrance/Hint/");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      wait(5000);
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
    checkConnection();
    Handle_Up=false;
    Puzzle_Solved=false;
    handleMoved=false;
    fill_solid( leds, NUM_LEDS, CHSV(0, 255, 100));
    FastLED.show(); 
    servo1.write(1900);//Set to proper positionservoWrite(1950);//
    client.publish("/Egypt/Tomb/","Tomb Switch Box Reset");    
  }
  else if (message == "hintSuggested") {
    checkConnection();
    client.publish("/Egypt/Tomb/","Tomb Mqtt hint suggested");
    mqttHint=true;
  }
  else if (message == "on"){
    
    digitalWrite(hintLight, LOW);
    checkConnection();
    client.publish("/Egypt/Tomb/","Tomb hint button light is on");
    flashHint=false;
    mqttHint=false;
    hintR=false;
  }
   else if (message == "off"){
    
    digitalWrite(hintLight, HIGH);    
    checkConnection();
    client.publish("/Egypt/Tomb/","Tomb hint button light is off");
    flashHint=false;
    mqttHint=false;
    hintR=false;
  }
  
}

void setup() {
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  
  servo1.attach(servoPin, 500, 2400);
  servo1.write(1900);
  
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  
  Serial.begin(115200);
  //servoWrite(1950);
  pinMode(Puzzle_Input, INPUT_PULLUP);
  pinMode(Handle_Input, INPUT_PULLUP);
  pinMode(Lockout_Input,INPUT_PULLUP);
  pinMode(Lock, OUTPUT);
  pinMode(Door, OUTPUT);
  pinMode(handleSfx, OUTPUT);
  pinMode(switchSfx, OUTPUT);
  pinMode(hintLight, OUTPUT);
  pinMode(hintButton, INPUT_PULLUP);
  pinMode(eStop, INPUT_PULLUP);
  
  digitalWrite(Lock, HIGH);
  digitalWrite(Door,HIGH);
  digitalWrite(handleSfx,HIGH);
  digitalWrite(switchSfx,HIGH);
  digitalWrite(hintLight,LOW);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid( leds, NUM_LEDS, CHSV(0, 255, 100));
    FastLED.show();
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
  if ((currentMillis - previousMillis >=interval)) {
    client.publish("/Egypt/Tomb/", "Tomb Entrance Watchdog");
    previousMillis = currentMillis;
    
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
void servoWrite(int microseconds) {
  servo1.attach(servoPin,1000, 2900);//myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  servo1.write(microseconds);
  wait(650);
  servo1.detach();
}
void readButtons(){
  if (digitalRead (hintButton)==LOW && !hintR){
    hintR=true;
    client.publish("/Egypt/Tomb/","Tomb hint button pressed");
    mqttHint=false;
    flashHint=true;
    flash();
    }
    if (digitalRead (eStop)==LOW && !estop){
    estop=true;
    client.publish("/Egypt/Tomb/","Tomb Estop button pressed");
    }
    if (digitalRead(Puzzle_Input) == LOW && !Puzzle_Solved) {
    digitalWrite(switchSfx,LOW);
    Puzzle_Solved=true;
    fill_solid( leds, NUM_LEDS, CHSV(95, 255, 100));
    FastLED.show(); 
    servo1.write(1300);//servoWrite(1300);//
    client.publish("/Egypt/Tomb/","Switch Puzzle Solved");
    delay(20);
    digitalWrite(switchSfx,HIGH);
  }
  if (digitalRead(Handle_Input) == LOW && !Handle_Up && Puzzle_Solved) {
    Handle_Up=true;
    client.publish("/Egypt/Tomb/","Handle Up");    
  }
  if (digitalRead(Lockout_Input) == LOW && !Puzzle_Solved && !handleMoved) {
    digitalWrite(handleSfx,LOW);
    client.publish("/Egypt/Tomb/","Handle Moved");
    handleMoved=true; 
    delay(100); 
    digitalWrite(handleSfx,HIGH);  
  }
  if (digitalRead(Lockout_Input) == HIGH && !Puzzle_Solved && handleMoved) {
    client.publish("/Egypt/Tomb/","Handle Released");
    handleMoved=false; 
    delay(20);  
  }
  }
  void flash() {
  while (flashHint){
    digitalWrite(hintLight,HIGH);
   wait(1000);
   if (!flashHint){
    break;
    }
    digitalWrite(hintLight,LOW);      
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
    digitalWrite(hintLight,HIGH);
   wait(125);
   if(!mqttHint){
    break;
    }   
   digitalWrite(hintLight,LOW);
   wait(125);
   i++;
   ArduinoOTA.handle();
   if(!mqttHint){
    break;
    }
    }
    mqttHint=false;
  }
void loop() {  
  ArduinoOTA.handle();
  readButtons();
  checkConnection();
  client.loop();
  mqttFlash();
  
}
