/**
   --------------------------------------------------------------------------------------------------------------------
   Example sketch/program showing how to read data from more than one PICC to serial.
   --------------------------------------------------------------------------------------------------------------------
   This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
   Example sketch/program showing how to read data from more than one PICC (that is: a RFID Tag or Card) using a
   MFRC522 based RFID Reader on the Arduino SPI interface.
   Warning: This may not work! Multiple devices at one SPI are difficult and cause many trouble!! Engineering skill
            and knowledge are required!
   @license Released into the public domain.
   Typical pin layout used:
   -----------------------------------------------------------------------------------------
               MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
               Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
   Signal      Pin          Pin           Pin       Pin        Pin              Pin
   -----------------------------------------------------------------------------------------
   RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS 1    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required *
   SPI SS 2    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required *
   SPI MOSI    MOSI         11 / ICSP-4   51        41        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        42        ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        43        ICSP-3           15
   More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
*/
#include <PubSubClient.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

boolean mqttRst = false;
boolean mqttSolve = false;
unsigned long previousMillis = 0;
const long interval = 10000;           // interval at which to send mqtt watchdog (milliseconds)
int lastValue;

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

#define RST_PIN 22   // Configurable, see typical pin layout above
#define SS_1_PIN 5   // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2
#define SS_2_PIN 17  // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 1
#define SS_3_PIN 16

#define NR_OF_READERS 3

byte ssPins[] = { SS_1_PIN, SS_2_PIN, SS_3_PIN };

MFRC522 mfrc522[NR_OF_READERS];  // Create MFRC522 instance.

int tagID;
int inputOrder[3] = { 0, 0, 0 };
const int correctOrder[3] = { 1, 2, 3 };
int counter = 0;
bool disable = false;

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
  ArduinoOTA.setHostname("Library_Esp32_OTA");

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
  SPI.end();
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
      client.publish("/Egypt/Library/", "Library RFID puzzle online");
      // ... and resubscribe
      client.subscribe("/Egypt/Library/RFIDpuzzle");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      wait(5000);
    }
  }
  SPI.begin();  // Init SPI bus
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
  if (message == "reset") {
    client.publish("/Egypt/Library/", "RFIDpuzzle Reset");
    digitalWrite(4, HIGH);
    disable = false;
    mqttRst = true;
  } else if (message == "solve") {
    mqttSolve = true;
  }
  else if (message == "Are you there?") {
    client.publish("/Egypt/Library/", "Library Puzzle OK");
  }
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
    client.publish("/Egypt/Library/", "Library Watchdog");
    previousMillis = currentMillis;
    
  }
  
}

void setup() {

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  Serial.begin(115200);
  Serial.setTimeout(500);  // Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  while (!Serial)
    ;           // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();  // Init SPI bus

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN);  // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
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

/*
   Main loop.
*/
void loop() {
  ArduinoOTA.handle();
  checkConnection();
  client.loop();  
  if (!disable) {
    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    boolean solved = true;
    byte sector = 1;
    byte blockAddr = 5;
    byte dataBlock[] = {
      0x01, 0x02, 0x03, 0x04,  //  1,  2,   3,  4,
      0x05, 0x06, 0x07, 0x08,  //  5,  6,   7,  8,
      0x09, 0x0a, 0xff, 0x0b,  //  9, 10, 255, 11,
      0x0c, 0x0d, 0x0e, 0x0f   // 12, 13, 14, 15
    };
    byte trailerBlock = 7;
    MFRC522::StatusCode status;
    byte buffer[18];
    byte size = sizeof(buffer);

    for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
      // Look for new cards

      delay(10);  //allow bus to settle

      mfrc522[reader].PCD_Init();

      if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
        status = (MFRC522::StatusCode)mfrc522[reader].MIFARE_Read(blockAddr, buffer, &size);
        if (status != MFRC522::STATUS_OK) {
          Serial.print(F("MIFARE_Read() failed: "));
          Serial.println(mfrc522[reader].GetStatusCodeName(status));
        }
        //Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
        dump_byte_array(buffer, 8);  //Serial.println();
        Serial.print("Reader ");
        Serial.print(reader);
        Serial.print(": ");
        Serial.println(tagID);
        inputOrder[reader] = tagID;
        Serial.println(inputOrder[reader]);

        for (int i = 0; i < 3; i++) {
          Serial.print(inputOrder[i]);
          Serial.print(" ");
        }

        mfrc522[reader].PICC_HaltA();
        mfrc522[reader].PCD_StopCrypto1();


      }  //if (mfrc522[reader].PICC_IsNewC
      else {
        if (counter % 3 == 0) {
          inputOrder[reader] = 0;
        }
      }
      if (inputOrder[reader] != correctOrder[reader]) {
        publishState();
        solved = false;
      }

    }  //for(uint8_t reader


    if (solved || mqttSolve) {
      digitalWrite(4, LOW);
      client.publish("/Egypt/Library/", "RFIDpuzzle Solved");
      publishState();
      disable = true;
      mqttRst = false;
      mqttSolve = false;
      for (int i = 0; i < 3; i++) {//Resets inputOrder to all 0's
        inputOrder[i] = 0;
      }
    }


    counter = counter + 1;
  }
}

void publishState() { 

  int finalValue = 0;
  for (int i = 0; i < 3; i++) {
    finalValue = finalValue + inputOrder[i] * (pow(10, (NR_OF_READERS - 1 - i)));
  }

  if (lastValue != finalValue) {
    String finalValueString = String(finalValue);
    client.publish("/Egypt/Library/Output/", (char*) finalValueString.c_str());
  }
  lastValue = finalValue;
}


//Helper routine to dump a byte array as hex values to Serial.

void dump_byte_array(byte* buffer, byte bufferSize) {
  tagID = buffer[0];
}
