


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
#include <SPI.h>
#include <FastLED.h>
#include<ESP8266WiFi.h>
#include <MFRC522.h>
#include <ArduinoOTA.h>

#define DATA_PIN    D1
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    8
CRGB leds[NUM_LEDS];
#define BRIGHTNESS         200
#define FRAMES_PER_SECOND  120

boolean mqttRst = false;
boolean mqttSolve = false;

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

#define RST_PIN         D3          // Configurable, see typical pin layout above
#define SS_1_PIN        D8         // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2

char dataTopic[32] = "/Egypt/Tomb/RFIDpuzzle/4/";
char onlineMsg[] = "RFID reader 4 online";
char lightChannel[] = "/Egypt/Tomb/RFIDpuzzle/4/LED/";

byte ssPins[] = {SS_1_PIN};

MFRC522 mfrc522(SS_1_PIN, RST_PIN);   // Create MFRC522 instance
int tagID;
boolean currentState;
boolean lastState;


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

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Tomb_ObeliskTop_OTA");

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
    FastLED.show();
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Tomb/", onlineMsg);
      // ... and resubscribe
      client.subscribe("/Egypt/Tomb/RFIDpuzzle/");
      client.subscribe(lightChannel);
    } else {
      ArduinoOTA.handle();
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte * payload, unsigned int length) {
  payload[length] = '\0';
  String message = (char*)payload;
  int value = message.toInt();
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
  if (message == "read") {
    readDataInst();
  }
  else if (!strcmp(topic, lightChannel)) {
    if (message == "off") {
      leds[0] = CHSV(255, 255, 0);
      leds[1] = CHSV(255, 255, 0);
      leds[2] = CHSV(255, 255, 0);
      leds[3] = CHSV(255, 255, 0);
      leds[4] = CHSV(255, 255, 0);
      leds[5] = CHSV(255, 255, 0);
      leds[6] = CHSV(255, 255, 0);
      leds[7] = CHSV(255, 255, 0);
    }
    else {
      leds[0] = CHSV(160, 255, 255);
      leds[1] = CHSV(96, 255, 255);
      leds[2] = CHSV(0, 255, 255);
      leds[3] = CHSV(255, 0, 255);
      leds[4] = CHSV(255, 0, 255);
      leds[5] = CHSV(255, 0, 255);
      leds[6] = CHSV(255, 0, 255);
      leds[7] = CHSV(255, 0, 255);
    }
  }
  wait(20);
}

void checkConnection() {
  if (!client.connected()) {
    reconnect();
  }
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  SPI.begin();        // Init SPI bus

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  mfrc522.PCD_Init(SS_1_PIN, RST_PIN); // Init each MFRC522 card
  Serial.print(F("Reader "));
  Serial.print(F(": "));
  mfrc522.PCD_DumpVersionToSerial();
}

/*
   Main loop.
*/
void loop() {
  EVERY_N_MILLISECONDS(25) {
    readData();
  }
  readData();
  FastLED.show();
  ArduinoOTA.handle();
  client.loop();
  checkConnection();
}


//Helper routine to dump a byte array as hex values to Serial.

void wait(int ms) {
  for (int i = 0; i < ms; i++) {
    client.loop();
    ArduinoOTA.handle();
    delay(1);
  }
}

void dump_byte_array(byte * buffer, byte bufferSize) {
  tagID = buffer[0];
}

void readData() {
  FastLED.show();
  byte sector         = 1;
  byte blockAddr      = 5;
  byte dataBlock[]    = {
    0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
    0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
    0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
    0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
  };
  byte trailerBlock   = 7;
  MFRC522::StatusCode status;
  byte buffer[18];
  byte size = sizeof(buffer);

  mfrc522.PCD_Init();

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    FastLED.show();
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("MIFARE_Read() failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
    }
    //Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
    dump_byte_array(buffer, 8); //Serial.println();
    lastState = currentState;
    currentState = true;
    if (currentState != lastState) {
      Serial.println(tagID);
      String payload = String(tagID);
      client.publish(dataTopic, (char*) payload.c_str());
    }

    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();


  } //if (mfrc522.PICC_IsNewC

  else {
    FastLED.show();
    tagID = 0;
    lastState = currentState;
    currentState = false;
    if (currentState != lastState) {
      Serial.println(tagID);
      String payload = String(tagID);
      client.publish(dataTopic, (char*) payload.c_str());
    }

  }

}

void readDataInst() {
  byte sector         = 1;
  byte blockAddr      = 5;
  byte dataBlock[]    = {
    0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
    0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
    0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
    0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
  };
  byte trailerBlock   = 7;
  MFRC522::StatusCode status;
  byte buffer[18];
  byte size = sizeof(buffer);

  mfrc522.PCD_Init();

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("MIFARE_Read() failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
    }
    //Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
    dump_byte_array(buffer, 8); //Serial.println();
    String payload = String(tagID);
    client.publish(dataTopic, (char*) payload.c_str());


    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();


  } //if (mfrc522.PICC_IsNewC

  else {
    tagID = 0;
    String payload = String(tagID);
    client.publish(dataTopic, (char*) payload.c_str());

  }
}
