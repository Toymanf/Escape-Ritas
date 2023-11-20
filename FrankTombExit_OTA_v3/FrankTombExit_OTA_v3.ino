#include <PubSubClient.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FastLED.h>

#ifndef STASSID
#define STASSID "Control booth"
#define STAPSK  "MontyLives"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;
const char* mqtt_server = "192.168.86.101";

#define DATA_PIN    15
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    14
CRGB leds[NUM_LEDS];
#define BRIGHTNESS          75
#define FRAMES_PER_SECOND  120

#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"

#define Door 19 //pin numbers to use for controlling relays   
#define Drawer 21
#define Pusher 5
#define Relic_Slide 4
#define Lock 26
#define Mummy 16
#define Ob_Light 17
#define Mum_Light 18
#define Limit_Switch 33 
boolean HoldOpen=false;
unsigned long previousMillis = 0;
unsigned long interval = 10000;


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
  ArduinoOTA.setHostname("Tomb_Exit_OTA");

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
    String clientId = "ESP32-Tomb-Exit";
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Tomb/", "Tomb Exit Online");
      // ... and resubscribe
      client.subscribe("/Egypt/Tomb/Exit/");
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
  if (message == "opendoor") {
    digitalWrite(Door, LOW);
    digitalWrite(Lock, LOW);  // Release MagLock  NO contacts
    client.publish("/Egypt/Tomb/", "Exit Opened");
  }
  else if (message == "closedoor") {
    digitalWrite(Door, HIGH);
    digitalWrite(Lock, HIGH);  // Activate MagLock NO contacts
    client.publish("/Egypt/Tomb/", "Exit Closed");
  }
  if (message == "mummyUp") {
    digitalWrite(Mummy, HIGH);  
    digitalWrite(Mum_Light, LOW);  // 
    client.publish("/Egypt/Tomb/", "Mummy Up");
  }
  else if (message == "mummyDown") {
    digitalWrite(Mummy, LOW);
    wait(5000);
    digitalWrite(Mum_Light, HIGH);  // Off
    client.publish("/Egypt/Tomb/", "Mummy Down");
  }
  else if (message == "opendrawer") {
    digitalWrite(Drawer, LOW);
    fill_solid( leds, NUM_LEDS, CHSV(100, 0, 255));
    FastLED.show();
    
    Serial.println("Wall drawer opened");
    client.publish("/Egypt/Tomb/", "Wall drawer opened");
  }
  else if (message == "closedrawer") {
    digitalWrite(Drawer, HIGH);
    fill_solid( leds, NUM_LEDS, CHSV(0, 0, 0));
    FastLED.show();

    client.publish("/Egypt/Tomb/", "Wall drawer closed");
  }
  else if (message == "pushrelic") {
    digitalWrite(Door, LOW);

    client.publish("/Egypt/Tomb/", "Retrieval pusher actuated");
  }
  else if (message == "retractrelic") {
    digitalWrite(Door, HIGH);

    client.publish("/Egypt/Tomb/", "Retrieval pusher retracted");
  }
  else if (message == "pushslide") {
    digitalWrite(Relic_Slide, HIGH);
    HoldOpen=false;

    client.publish("/Egypt/Tomb/", "Relic slides closed");
  }
  else if (message == "pullslide") {
    digitalWrite(Relic_Slide, LOW);
    HoldOpen=false;

    client.publish("/Egypt/Tomb/", "Relic slides opened");
  }
  else if (message == "openslide") {
    digitalWrite(Relic_Slide, LOW);
    HoldOpen=true;

    client.publish("/Egypt/Tomb/", "Relic slides opened");
  }
  else if (message == "obLightOn") {
    digitalWrite(Ob_Light, HIGH);

    client.publish("/Egypt/Tomb/", "Obelisk Spotlight On");
  }
  else if (message == "obLightOff") {
    digitalWrite(Ob_Light, LOW);

    client.publish("/Egypt/Tomb/", "Obelisk Spotlight Off");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Door, OUTPUT);
  pinMode(Drawer, OUTPUT);
  pinMode(Relic_Slide, OUTPUT);
  pinMode(Pusher, OUTPUT);
  pinMode(Ob_Light, OUTPUT);
  pinMode(Mum_Light, OUTPUT);
  pinMode(Lock, OUTPUT);
  pinMode(Mummy, OUTPUT);
  pinMode(Limit_Switch, INPUT_PULLUP);

  digitalWrite(Door, HIGH);
  digitalWrite(Drawer, HIGH);
  digitalWrite(Relic_Slide, HIGH);
  digitalWrite(Pusher, HIGH);
  digitalWrite(Ob_Light, LOW);
  digitalWrite(Mum_Light, HIGH);
  digitalWrite(Lock, LOW);
  digitalWrite(Mummy, LOW);
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
  fill_solid( leds, NUM_LEDS, CHSV(0, 0, 0));
    FastLED.show();
}

void checkConnection() {
  unsigned long currentMillis = millis();
// if WiFi is down, try reconnecting
if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
  Serial.print(millis());
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  //WiFi.reconnect();
  previousMillis = currentMillis;
}
if ((!client.connected() ) && (WiFi.status() == WL_CONNECTED) ){
    Serial.println("Lost mqtt connection");
    reconnect();
  }
  if ((currentMillis - previousMillis >=interval)) {
    client.publish("/Egypt/Tomb/", "Exit Watchdog");
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
  }
}

void loop() {
  if (digitalRead(Limit_Switch) == LOW && !HoldOpen) {
    wait(1200); //Set drop duration here
    digitalWrite(Relic_Slide, HIGH);
  }
  ArduinoOTA.handle();
  checkConnection();
  reconnect();
  client.loop();
}
