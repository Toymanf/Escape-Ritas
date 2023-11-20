
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "Control booth";
const char* password = "MontyLives";
const char* mqtt_server = "192.168.86.101";
#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"

WiFiClient wifiClient;

PubSubClient client(wifiClient);
#define P1 12
#define P2 13
#define P3 14
#define P4 15
#define P5 16
#define P6 17
#define P7 21
#define P8 22
boolean solved = false;






/*
   Initialize.
*/

void setup_wifi() {
  delay(10);
  
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
  ArduinoOTA.setHostname("Tomb_Esp32_SlidingTilePuzzle_OTA");

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
    byte mac[5];
    WiFi.macAddress(mac);  // get mac address
    String clientId = String(mac[0]) + String(mac[4]);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Tomb/", "Tomb sliding puzzle online");
      // ... and resubscribe
      client.subscribe("/Egypt/Tomb/SlidingPuzzle/");
      } 
      else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = (char*)payload;
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
  if (message == "StatusCheck") {
    if (!solved){
    client.publish("/Egypt/Tomb/", "Tomb Sliding Puzzle Ready");
    }
    else {
      client.publish("/Egypt/Tomb/", "Tomb Sliding Puzzle in Solved State");
      }
    
  } 
  else if (message == "reset") {
    client.publish("/Egypt/Tomb/", "Tomb Sliding Puzzle Reset request received");
    if (!solved){
    client.publish("/Egypt/Tomb/", "Tomb Sliding Puzzle Ready");
    }
    else {
      client.publish("/Egypt/Tomb/", "Tomb Sliding Puzzle in Solved State");
      }
    solved = false;
  }
}

void checkConnection() {
  if (!client.connected()) {
    delay(3000);
    Serial.println("Lost connection");
    reconnect();
  }
}

void setup() {
  
  pinMode(P1, INPUT_PULLUP);
  pinMode(P2, INPUT_PULLUP);
  pinMode(P3, INPUT_PULLUP);
  pinMode(P4, INPUT_PULLUP);
  pinMode(P5, INPUT_PULLUP);
  pinMode(P6, INPUT_PULLUP);
  pinMode(P7, INPUT_PULLUP);
  pinMode(P8, INPUT_PULLUP);
  

  Serial.begin(115200);
  Serial.setTimeout(500);  // Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  while (!Serial);           // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  
}

void wait(int ms) {
  for (int i = 0; i < ms; i++) {
    ArduinoOTA.handle();
   checkConnection();
    client.loop();
    delay(1);
  }
}
  

/*
   Main loop.
*/
void loop() {
  ArduinoOTA.handle();
  checkConnection();
  client.loop();
  boolean complete = false;
  while (!complete){
  int Pc1=digitalRead(P1);
  int Pc2=digitalRead(P2);
  int Pc3=digitalRead(P3);
  int Pc4=digitalRead(P4);
  int Pc5=digitalRead(P5);
  int Pc6=digitalRead(P6);
  int Pc7=digitalRead(P7);
  int Pc8=digitalRead(P8);
  
    if (!Pc1 && !Pc2 && !Pc3 && !Pc4 && !Pc5 && !Pc6 && !Pc7 && !Pc8 && !solved){//hall near magnet
  Serial.println ("Solved");
  client.publish("/Egypt/Tomb/", "Sliding puzzle solved");
  solved=true;
  complete = true;  
  } 
  wait(10);
}
while (solved){
    
  wait(200);
  
}

  Serial.println ("Not solved");
  client.publish("/Egypt/Tomb/", "Sliding puzzle reset");
  solved=false;
  complete = false;
 



  }
