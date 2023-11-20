//Archives Maze RFID OTA 10-10-22
/**
   --------------------------------------------------------------------------------------------------------------------
   Example sketch/program showing how to read data from more than one PICC to serial.
   --------------------------------------------------------------------------------------------------------------------
   This is a MFRC522 Archives example; for further details and other examples see: https://github.com/miguelbalboa/rfid

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
#include <FastLED.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
Servo servo1;

boolean mqttRst = false;
boolean mqttSolve = false;
boolean solved = false;

const char* ssid = "Control booth";
const char* password = "MontyLives";
const char* mqtt_server = "192.168.86.101";
#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"

#define servoPin 27


#define DATA_PIN    17
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    72
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

WiFiClient wifiClient;

PubSubClient client(wifiClient);
                                  //ss=5 sck=18 miso=19 mosi=23  ==VSPI bus
                                    //Can't use any of these pins as ADC when using WiFi 4,12,13,14,15,25,26,27
                                    //All of these pins are options for outputs/inputs 16,17,18,19,21,22,23
                                    //These pins are available for input only 34,35(,36,37,38,39)

#define RST_PIN         22          // Configurable, see typical pin layout above
#define SS_1_PIN        16  //4 16 pos 1     // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2
#define SS_2_PIN        15  //5 15 pos 3     // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 1
#define SS_3_PIN        14 //12 14 pos 5
#define SS_4_PIN        13 //13  pos 7
#define SS_5_PIN        14
#define SS_6_PIN        15
#define SS_7_PIN        16
#define MAZE_COMPLETE_PIN 32
#define MAZE_READY_PIN 26
#define hall_2 34
#define hall_4 35
#define hall_6 33

#define NR_OF_READERS  4
#define RST_NUM         0

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN, SS_4_PIN};//, SS_5_PIN, SS_6_PIN,SS_7_PIN
MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.

int tagID;
int lastValue;
int inputOrder[NR_OF_READERS] = {RST_NUM, RST_NUM, RST_NUM, RST_NUM};//, RST_NUM, RST_NUM,RST_NUM
const int correctOrder[NR_OF_READERS] = {1, 2, 3, 4};//, 0, 0,0 
int counter = 0;
bool disable = false;
bool RfidReady=true;
bool piece2=false;
bool piece4=false;
bool piece6=false;
bool halls=false;
bool ballHomed=false;
int i=0;
int t=0;

int pos_2=0;
int pos_4=0;
int pos_6=0;

unsigned long previousMillis = 0;
unsigned long interval = 10000;

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
  ArduinoOTA.setHostname("Archives_Maze_Esp32_OTA");

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
    String clientId = "ArchivesMazePuzzle-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Archives/", "Archives RFID puzzle online");
      // ... and resubscribe
      client.subscribe("/Egypt/Archives/RFIDpuzzle");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      wait(5000);
    }
  }
}

void callback(char* topic, byte * payload, unsigned int length) {
  payload[length] = '\0';
  String message = (char*)payload;
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
  //client.publish("/Egypt/Archives/", "in the loop");
  if (message == "reset") {
    client.publish("/Egypt/Archives/", "Archive RFID puzzle reset requested");
    puzzleReady(); //Check and report state of puzzle
    mqttRst=true;
    
  }
  else if (message == "solve") {
    
    client.publish("/Egypt/Archives/", "Archive RFID puzzle solved by operator");
    mqttSolve = true;
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
    client.publish("/Egypt/Archives/", "Maze Watchdog");
    previousMillis = currentMillis;
    
  }
  
}


void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  
  servo1.attach(servoPin, 500, 2400);
  servo1.write(90);
  
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  //servoWrite(60);
  pinMode(MAZE_COMPLETE_PIN, INPUT_PULLUP);
  pinMode(MAZE_READY_PIN, INPUT_PULLUP);
  pinMode(hall_2, INPUT);
  pinMode(hall_4, INPUT);
  pinMode(hall_6, INPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  SPI.begin();        // Init SPI bus

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid( leds, NUM_LEDS, CHSV(0, 0, 0));
  FastLED.show();

}
void servoWrite(int microseconds) {
  servo1.attach(servoPin);
  servo1.write(microseconds);
  wait(650);
  servo1.detach();
}
void readHalls(){
  
  t=t+1;
 int average2 = 0;
 for (int i=0; i < 20; i++) {
 average2 = average2 + analogRead(hall_2);
 }
 pos_2 = average2/20;
  //pos_2=analogRead(hall_2);
  if (pos_2>=2350 && pos_2<=2450 && !piece2){
    Serial.println(pos_2);
    client.publish("/Egypt/Archives/", "Archive maze piece 2 present");
    piece2=true;
    }
    else if (pos_2<=2349 && piece2){
    Serial.println(pos_2);
    client.publish("/Egypt/Archives/", "Archive maze piece 2 not present");
    piece2=false;
    halls=false;
    }
 int average4 = 0;
 for (int i=0; i < 20; i++) {
 average4 = average4 + analogRead(hall_4);
 }
 pos_4 = average4/20;
    //pos_4=analogRead(hall_4);
  if (pos_4>=2150 && pos_4<=2275 &&!piece4){
    Serial.println(pos_4);
    client.publish("/Egypt/Archives/", "Archive maze piece 4 present");
    piece4=true;
    }
    else if (pos_4<=2149 && piece4){
    Serial.println(pos_4);
    client.publish("/Egypt/Archives/", "Archive maze piece 4 not present");
    piece4=false;
    halls=false;
    }
 int average6 = 0;
 for (int i=0; i < 20; i++) {
 average6 = average6 + analogRead(hall_6);
 }
 pos_6 = average6/20;   
    //pos_6=analogRead(hall_6);
  if (pos_6<=1800 && pos_6 >=1500 && !piece6){
    Serial.println(pos_6);
    client.publish("/Egypt/Archives/", "Archive maze piece 6 present");
    piece6=true;
    halls=false;
    }
    else if (pos_6>=1801 && piece6){
    Serial.println(pos_6);
    client.publish("/Egypt/Archives/", "Archive maze piece 6 not present");
    piece6=false;
    halls=false;
    }
    if (piece2 && piece4 && piece6 && !halls){
      halls=true;
      client.publish("/Egypt/Archives/", "Archive maze all halls present");
    }

    if(t>=50){
      Serial.print("position 2: ");
      Serial.println(pos_2);
      Serial.print("position 4: ");
      Serial.println(pos_4);
      Serial.print("position 6: ");
      Serial.println(pos_6);
      t=0;
      }
  }
 void readPosition(){ 
  solved = true;
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

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    // Look for new cards
    readHalls();

    delay(20); //allow bus to settle

    mfrc522[reader].PCD_Init();

    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
      status = (MFRC522::StatusCode) mfrc522[reader].MIFARE_Read(blockAddr, buffer, &size);
      if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522[reader].GetStatusCodeName(status));
      }
      //Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
      dump_byte_array(buffer, 8); //Serial.println();
      Serial.print("Reader ");
      Serial.print(reader);
      Serial.print(": ");
      Serial.println(tagID);
      inputOrder[reader] = tagID;
      Serial.println(inputOrder[reader]);

      for (int i = 0; i < NR_OF_READERS; i++) {
        Serial.print(inputOrder[i]);
        Serial.print(" ");
      }

      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();


    } //if (mfrc522[reader].PICC_IsNewC
    else {
      if (counter % 3 == 0) {
        inputOrder[reader] = RST_NUM;
      }
    }
    if (inputOrder[reader] != correctOrder[reader]) {
      publishState();
      solved = false;
    }
    if (solved){
      publishState();
      readHalls();
      }

  } //for(uint8_t reader
 }//end of void readPosition
  
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
  //if (!disable) {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  readPosition();

  if ((solved && halls) || mqttSolve) {
    publishState();
    fill_solid( leds, NUM_LEDS, CHSV(100, 0, 50));
    FastLED.show();    
    servo1.write(45);    
    checkConnection();
    client.publish("/Egypt/Archives/", "Archive Senet Order Solved");
    mqttRst=false;    
    
    while (!mqttRst) {     //loop here until Mqtt reset or condition met   
      readPosition();   
      if (digitalRead(MAZE_COMPLETE_PIN)==LOW){     //Send message only once when maze completed
        client.publish("/Egypt/Archives/", "Archive Maze puzzle solved");                
        fill_solid( leds, NUM_LEDS, CHSV(95, 255, 100));
        FastLED.show();        
        servo1.write(90);//return servo to reset position
        break;                
        }
        if (digitalRead(MAZE_READY_PIN)==HIGH && ballHomed){     //Send message only once 
        client.publish("/Egypt/Archives/", "Archive Maze ball not homed");
        ballHomed=false;
        }
        if (digitalRead(MAZE_READY_PIN)==LOW && !ballHomed){     //Send message only once when maze reset is completed
        client.publish("/Egypt/Archives/", "Archive Maze ball is homed");
        ballHomed=true;
        }
        wait(20);
    }  
    while (!mqttRst){       //loop here until Mqtt reset or condition met
      if (digitalRead(MAZE_READY_PIN)==LOW){     //Send message only once when maze reset is completed
        client.publish("/Egypt/Archives/", "Archive Maze puzzle ready");        
        puzzleReady();
        break;           
        }            
      wait(20);
     
    }
     // When mqttRst=true, break out of above loop
    checkConnection();
    
    
    for (int i = 0; i < NR_OF_READERS; i++) {
      inputOrder[i] = RST_NUM;
    }
    
    Serial.println("RESET");

   
  }


  counter = counter + 1;
}

void publishState() { 

  int finalValue = 0;
  for (int i = 0; i < NR_OF_READERS; i++) {
    finalValue = finalValue + inputOrder[i] * (pow(10, (NR_OF_READERS - 1 - i)));
  }

  if (lastValue != finalValue) {
    String finalValueString = String(finalValue);
    client.publish("/Egypt/Archives/RFID/", (char*) finalValueString.c_str());
  }
  lastValue = finalValue;
}

//Helper routine to dump a byte array as hex values to Serial.

void dump_byte_array(byte * buffer, byte bufferSize) {
  tagID = buffer[0];
}

void puzzleReady() {

  if (digitalRead(MAZE_READY_PIN)==HIGH){
    client.publish("/Egypt/Archives/", "Archive Maze ball not registering in reset position");
    } 
    else if (digitalRead(MAZE_COMPLETE_PIN)==LOW){
      client.publish("/Egypt/Archives/", "Archive Maze ball registering in finished position");
      }
      else if(!RfidReady){
      client.publish("/Egypt/Archives/", "Archive Maze RFID not properly reset");
      }
    else{          
    client.publish("/Egypt/Archives/", "Archive RFID puzzle reset");    
    servo1.write(90); 
    fill_solid( leds, NUM_LEDS, CHSV(140, 255, 70));
    FastLED.show();
    delay(1000);
    fill_solid( leds, NUM_LEDS, CHSV(0, 0, 0));
    FastLED.show();
    client.publish("/Egypt/Archives/", "Archive Maze ball is homed");
    mqttRst = false;
    mqttSolve = false;
    ballHomed=true;    
  }
}
