//Archives Keypad With OTA Updated to add watchdog 11-22-22
// Use this example with the Adafruit Keypad products.
// You'll need to know the Product ID for your keypad.
// Here's a summary:
//   * PID3844 4x4 Matrix Keypad
//   * PID3845 3x4 Matrix Keypad
//   * PID1824 3x4 Phone-style Matrix Keypad
//   * PID1332 Membrane 1x4 Keypad
//   * PID419  Membrane 3x4 Matrix Keypad

#include "Adafruit_Keypad.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
Servo servo1;

char* ssid = "Control booth";
const char* password = "MontyLives";
const char* mqtt_server = "192.168.86.101";
#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"

#define DrawerSolenoid 33
#define doorSolenoid 22
#define Relay 18
#define doorKnob 27

#define MAX_DIGITS 8 //maximum number of inputs for a sequence
#define servoPin 32
#define limitSwitch 21
char mqttTopicPrefix[32] = "/Egypt/Archives/keypadPuzzle/";
char mqttTopic[32];

WiFiClient wifiClient;

PubSubClient client(wifiClient);

// define your specific keypad here via PID
#define KEYPAD_PID3845
// define your pins here
// can ignore ones that don't apply
#define R1    5
#define R2    16
#define R3    15
#define R4    13
#define C1    12
#define C2    4
#define C3    14
//#define C4    11
// leave this import after the above configuration
#include "keypad_config.h"
int input = 0;
int inputCode[MAX_DIGITS] = {};

int state = 0;
int lastSwitchState = 0;

int doorState = 0;
int lastState = 0;
boolean keyDone = false;
boolean report=false;

boolean killInstance = false;
boolean mrst = false;
boolean rstwait = false;
boolean knobTurned=false;
unsigned long previousMillis = 0;
unsigned long interval = 10000;

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  
  //servo1.attach(servoPin, 500, 2400);
  //servo1.write(1500);
  
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  Serial.begin(115200);
  Serial.setTimeout(500);// Set time out for
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  customKeypad.begin();
  rst(MAX_DIGITS);
  pinMode(DrawerSolenoid, OUTPUT);
  pinMode(doorSolenoid, OUTPUT);
  pinMode(Relay, OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);
  pinMode(doorKnob, INPUT_PULLUP);
  digitalWrite(Relay, LOW);
  digitalWrite(DrawerSolenoid, HIGH);
  digitalWrite(doorSolenoid, HIGH);
  servoWrite(70);
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
    Serial.print("Lost connection");
    reconnect();
  }
  if ((currentMillis - previousMillis >=interval)) {
    client.publish("/Egypt/Archives/", "Keypad Watchdog");
    previousMillis = currentMillis;
    
  }
}

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
  ArduinoOTA.setHostname("Archives_Keypad_Esp32_WithServo_OTA");

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
    String clientId = "ESP32-Keypad-Puzzle";
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/Egypt/Archives/", "Keypad puzzle online");
      // ... and resubscribe
      client.subscribe("/Egypt/Archives/keypadPuzzle/digitSelect/");
      client.subscribe("/Egypt/Archives/keypadPuzzle/");
      client.subscribe("servo");
      client.subscribe("Egypt/Library/");
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
  int value = message.toInt();
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();

  if (!strcmp(topic, mqttFullTopic("digitSelect/"))) {
    lastState = value;
    seekDigits(value);
    killInstance = true;
  }
  else if (message == "RFIDpuzzle Solved" || message == "speakeron") {
    digitalWrite(Relay, HIGH);
  }
  else if (message == "RFIDpuzzle Reset" || message == "speakeroff") {
    digitalWrite(Relay, LOW);
  }
  else if (message == "stop") {
    killInstance = true;
  }
  else if (message == "reset") {
    client.publish("/Egypt/Archives/", "Keypad reset. Reactivating.");
    killInstance = true;
    mrst = true;
    rstwait = true;
  }
  else if (message == "opendrawer") {
    digitalWrite(DrawerSolenoid, LOW);
    client.publish("/Egypt/Archives/", "Drawer opened");
  }
  else if (message == "closedrawer") {
    digitalWrite(DrawerSolenoid, HIGH);
    client.publish("/Egypt/Archives/", "Drawer closed");
  }
  else if (message == "revealknob") {
    digitalWrite(doorSolenoid, LOW);
    client.publish("/Egypt/Archives/", "Knob revealed");
    knobTurned=false;
  }
  else if (message == "retractknob") {
    digitalWrite(doorSolenoid , HIGH);
    client.publish("/Egypt/Archives/", "Knob retracted");
    knobTurned=false;
  }
  else if (message == "dropkey") {
    client.publish("/Egypt/Archives/", "Key dropped");
    servoWrite(35);
  }
  else if (message == "closekey") {
    client.publish("/Egypt/Archives/", "Key closed");
    servoWrite(70);
  }
  else if (!strcmp(topic, "servo")) {
    servoWrite(value);
  }
  else if (message == "reporting_On") {    
    client.publish("/Egypt/Archives/", "Reporting Enabled");
    report=true;
  }
  else if (message == "reporting_Off") {    
    client.publish("/Egypt/Archives/", "Reporting disabled");
    report=false;
  }
  checkConnection();

}

void rst(int digitLength) {
  for (int i = 0; i < digitLength; i++) {
    inputCode[i] = 10;
  }
}

char* mqttFullTopic(const char action[]) {
  strcpy (mqttTopic, mqttTopicPrefix);
  strcat (mqttTopic, action);
  return mqttTopic;
}

void seekDigits(int digitLength) { //Code getting stuck here until # received with digit(s)
  rst(digitLength);                //if less than digitLength is entered(without # key pressed), code resets properly and continues to work
  int finalValue = 0;              //if same or more than digitLength is entered(without # key pressed), code will not accept(pass thru)any additional digits until # key gets pressed 
  killInstance = false;            //a "reset" is sent to code from node-red 8 seconds after the last data received.
  customKeypad.tick();             //code appears to do nothing with the "reset" when digits entered >= digitLength. Monitor screen updates due to different 
  ArduinoOTA.handle();
  client.loop();
  Serial.println("Enter Passcode");
  checkConnection();  // above code only runs once per entry to loop

  for (int i = 0; i < digitLength; i++) {// for reporting id, this is loop A
    while (inputCode[i] == 10) {
      ArduinoOTA.handle();//added after crash 11-4-22
      client.loop();
      //doorKnobSwitch();
      if (report){
        client.publish("/Egypt/Archives/", "Inside loop A seekDigits");//I believe this was responding during "failed" mode
        };
      checkConnection();
      customKeypad.tick();
      lastState = state;
      state = digitalRead(limitSwitch);
      if (lastState != state) {
        killInstance = true;
      }
      if (killInstance) {
        return;
      }
      keypadEvent e = customKeypad.read();
      input = int((char)e.bit.KEY) - '0';
      String payload = String((char)e.bit.KEY);
      String iteration = String(i);
      iteration += "/";
      const char *channel = iteration.c_str();
      if (e.bit.EVENT == KEY_JUST_PRESSED) {
        inputCode[i] = input;
        if (input == -13 || input == -6) {  //If entry is # or *
          inputCode[i] = 10;                // Reset inputCode to value of 10?
          i--;                              //???
          client.publish(mqttFullTopic("Hash/"), (char*) payload.c_str());//Publish either -13 0r -6
        }
        else {
          client.publish(mqttFullTopic(channel), (char*) payload.c_str());
          Serial.println(inputCode[i]);
        }
      }
      wait(10);
    }
  }  //End of Loop "A"
  if (!killInstance) {
    while (input != -13) { //Begining of loop "B"   Enters this loop if # of inputs matches digitLength()then waits for press of # button
      if (report){
        client.publish("/Egypt/Archives/", "Inside loop B seekDigits");//
        };
      customKeypad.tick();
      keypadEvent e = customKeypad.read();
      input = int((char)e.bit.KEY) - '0';
      String payload = String((char)e.bit.KEY);
      ArduinoOTA.handle();
      client.loop();
      checkConnection();
      //doorKnobSwitch();
      if (killInstance) {
        return;
      }
    }                                      //End of loop "B"
    for (int i = 0; i < digitLength; i++) { //When code reaches this point, it populates finalValue with contents of array and publishes it
      finalValue = finalValue + inputCode[i] * (pow(10, (digitLength - 1 - i)));
    }
    String finalValueString = String(finalValue);
    client.publish(mqttFullTopic("Result/"), (char*) finalValueString.c_str());
  }
  keyDone = true;
}

void servoWrite(int microseconds) {
  servo1.attach(servoPin);
  servo1.write(microseconds);
  wait(650);
  servo1.detach();
}

void wait(int ms) {
  for (int i = 0; i < ms; i++) {
    client.loop();
    checkConnection();
    ArduinoOTA.handle();
    delay(1);
  }
}
void doorKnobSwitch() {
  if (digitalRead(doorKnob==LOW)){
  if (!knobTurned){
    client.publish("/Egypt/Archives/", "Knob Turned");
  knobTurned=true;
  }
  }
}

void loop() {
  ArduinoOTA.handle();
  client.loop();
  state = digitalRead(limitSwitch);
  //doorKnobSwitch();

  while (state == 1) {
    ArduinoOTA.handle();
    client.loop();
    checkConnection();
    client.publish("/Egypt/Archives/", "4 digit mode, drawer closed");
    seekDigits(4);
    lastSwitchState = state;
    state = digitalRead(limitSwitch);
    if (lastSwitchState != state) {
      return;
    }
  }

  while (state == 0) {
    ArduinoOTA.handle();
    client.loop();
    checkConnection();
    client.publish("/Egypt/Archives/", "5 digit mode, drawer opened");
    seekDigits(5);
    lastSwitchState = state;
    state = digitalRead(limitSwitch);
    if (lastSwitchState != state) {
      return;
    }
  }
}
