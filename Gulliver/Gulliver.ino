#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "index.h" //configuration and controls HTML webpage contents

//SSID and Password of your WiFi router
const char* ssid = "ESP8266SBR";
const char* password = "Algorithm";

AsyncWebServer server(80); //Server on port 80
AsyncWebSocket ws("/ws");
MPU6050 mpu;

volatile bool OTAstate = false;
volatile bool remote = false;
volatile bool distanceDataReady = false;
bool robotDirection = false;
bool servoDirection = false;
bool started = false;
bool turnStarted = false;
bool stopRobot = false;
volatile bool startEndReading = false;
double readingPos = 0.00;
double value = 0.00;
double output = 0.00;
double speedRobot = 0.00;
int i = 0;
int servoTrigger = 0;
int positionR = 0;
int wheelDir, servoPos = 0;
int kpAddr = 0, kiAddr = 1, kdAddr = 2, kpAddrDec = 3, kiAddrDec = 4, kdAddrDec = 5, ad = 6, add = 7, ads = 8, za = 9, zadec = 10, mml = 11, mmh = 12, mMl = 13, mMh = 14, mao = 15, mbo = 16, tsh = 17, tsl = 18;
int val, val2;
int m1 = D1, m2 = D5, m3 = D7, m4 = D6;
int offset = 0;
double Setpoint = 0.00, Input = 0.00, Output = 0.00;
double Kp = 0.00, Ki = 0.00, Kd = 0.00;
double pTemp = Kp, iTemp = Ki, dTemp = Kd;
double minimum, maximum;
double k = 0.00;
double desiredAngle = 0.00;
int turnSpeed = 0;
int speedTime = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, P_ON_M);
Servo myservo;
int trigPin = D0;
int echoPin = D8;
int OTAbutton = D2;
int servo = RX; // or D9 for node mcu
int led = TX;
int turnTime = 0;
int sampleTime = 15;
double distance = 20.00;
volatile unsigned long start;
volatile unsigned long finish;
unsigned long speedTimer;
unsigned long turnTimer;
unsigned long sr04Timer;
double adjustAngle = 0.00;
double zeroAngle = 0.00;
double motorMinVoltage = 0.00;
double motorMaxVoltage = 0.00;
double pitch = 0.00;
int offsetMotorA = 0;
int offsetMotorB = 0;
byte eepromVal = 0;
// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

//==============================================================
//                  WEB SOCKET EVENT AND NOT FOUND ROUTINE
//==============================================================


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) { // When a WebSocket message is received
  if (type == WS_EVT_CONNECT) {
    //client connected
    os_printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if (type == WS_EVT_DISCONNECT) {
    //client disconnected
    os_printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if (type == WS_EVT_ERROR) {
    //error was received from the other end
    os_printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if (type == WS_EVT_PONG) {
    //pong message was received (in response to a ping request maybe)
    os_printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
  } else if (type == WS_EVT_DATA) {
    char input;
    input = data[0];
    if (input == '1') {
      speedRobot = 2.00;
      speedTime = 500;
    }
    if (input == '2') {
      speedRobot = 4.00;
      speedTime = 500;
    }
    if (input == '3') {
      speedRobot = 6.00;
      speedTime = 500;
    }
    if (input == '4') {
      desiredAngle = 5.00;
      turnTime = 80;
      speedTime = 80;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == 'r') {
      desiredAngle = 5.00;
      turnTime = 120;
      speedTime = 120;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == 't') {
      desiredAngle = 5.00;
      turnTime = 200;
      speedTime = 200;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == '5') {
      desiredAngle = -5.00;
      turnTime = 80;
      speedTime = 80;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == 's') {
      desiredAngle = -5.00;
      turnTime = 120;
      speedTime = 120;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == 'z') {
      desiredAngle = -5.00;
      turnTime = 200;
      speedTime = 200;
      if (!robotDirection)
        speedRobot = 2.00;
      else
        speedRobot = -2.00;
    }
    if (input == '6') {
      speedRobot = -2.00;
      speedTime = 500;
    }
    if (input == '7') {
      speedRobot = -4.00;
      speedTime = 500;
    }
    if (input == '8') {
      speedRobot = -6.00;
      speedTime = 500;
    }
  }
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}


//==============================================================
//                  SETUP
//==============================================================


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(D3, D4);
  Wire.setClock(450000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(450, true);
#endif

  //initialize serial communication
  //Serial.begin(115200);

  WiFi.mode(WIFI_AP);           //Only Access point
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  boolean result = WiFi.softAP(ssid, password, 14, 0, 2);  //Start HOTspot removing password will disable security
  if (result == true)
  {
    EEPROM.begin(24);
    eepromVal = EEPROM.read(kpAddr);
    Kp = float(eepromVal);
    eepromVal = EEPROM.read(kiAddr);
    Ki = float(eepromVal);
    eepromVal = EEPROM.read(kdAddr);
    Kd = float(eepromVal);
    eepromVal = EEPROM.read(kpAddrDec);
    Kp = Kp + float(eepromVal / 100.00);
    eepromVal = EEPROM.read(kiAddrDec);
    Ki = Ki + float(eepromVal);
    eepromVal = EEPROM.read(kdAddrDec);
    Kd = Kd + float(eepromVal / 100.00);
    eepromVal = EEPROM.read(ad);
    adjustAngle = float(eepromVal);
    eepromVal = EEPROM.read(add);
    adjustAngle = adjustAngle + float(eepromVal / 100.00);
    eepromVal = EEPROM.read(ads);
    if (eepromVal == 0) {
      adjustAngle = adjustAngle * -1.00;
    }
    eepromVal = EEPROM.read(za);
    zeroAngle = float(eepromVal);
    eepromVal = EEPROM.read(zadec);
    zeroAngle = zeroAngle + float(eepromVal / 100.00);
    eepromVal = EEPROM.read(mml);
    motorMinVoltage = float(eepromVal);
    eepromVal = EEPROM.read(mmh);
    motorMinVoltage = motorMinVoltage + float(eepromVal);
    eepromVal = EEPROM.read(mMl);
    motorMaxVoltage = float(eepromVal);
    eepromVal = EEPROM.read(mMh);
    motorMaxVoltage = motorMaxVoltage + float(eepromVal);
    motorMaxVoltage = motorMaxVoltage * 2 + 3;
    eepromVal = EEPROM.read(mao);
    offsetMotorA = int(eepromVal);
    eepromVal = EEPROM.read(mbo);
    offsetMotorB = int(eepromVal);
    eepromVal = EEPROM.read(tsl);
    turnSpeed = float(eepromVal);
    eepromVal = EEPROM.read(tsh);
    turnSpeed = turnSpeed + float(eepromVal);

    ArduinoOTA.begin();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      //String s = MAIN_page; //Read HTML contents
      request->send_P(200, "text/html", MAIN_page); //Send web page
    });
    server.on("/getLoad", HTTP_GET, [](AsyncWebServerRequest * request) {
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& object = jsonBuffer.createObject();
      object["p"] = Kp;
      object["i"] = Ki;
      object["d"] = Kd;
      object["minimumVoltage"] = motorMinVoltage;
      object["maximumVoltage"] = motorMaxVoltage;
      offset = (offsetMotorA - offsetMotorB);
      object["offset"] = offset;
      object["zeroAngle"] = zeroAngle;
      object["error"] = ESP.getResetReason();
      object["turnSpeed"] = turnSpeed;

      String jsonData;
      object.prettyPrintTo(jsonData);
      request->send(200, "text/json", jsonData);
    });
    server.on("/getB", HTTP_GET, [](AsyncWebServerRequest * request) {
      int battery = analogRead(A0);
      request->send(200, "number/plane", String(battery));
    });
    server.on("/getP", HTTP_GET, [](AsyncWebServerRequest * request) {
      double tune = request->getParam("testo")->value().toFloat();
      Kp = tune;
      eepromVal = byte(tune);
      EEPROM.write(kpAddr, eepromVal);
      tune = tune - float(eepromVal);
      eepromVal = byte(tune * 100.00);
      EEPROM.write(kpAddrDec, eepromVal);
      request->send(200, "number/plane", String(Kp, 2));
    });
    server.on("/getI", HTTP_GET, [](AsyncWebServerRequest * request) {
      double tune = request->getParam("testo")->value().toFloat();
      Ki = tune;
      if (tune > 255.00) {
        eepromVal = 255;
      } else {
        eepromVal = byte(tune);
      }
      EEPROM.write(kiAddr, eepromVal);
      if (tune > 255.00) {
        eepromVal = byte(tune - 255.00);
      } else {
        eepromVal = 0;
      }
      EEPROM.write(kiAddrDec, eepromVal);
      request->send(200, "number/plane", String(Ki, 2));
    });
    server.on("/getD", HTTP_GET, [](AsyncWebServerRequest * request) {
      double tune = request->getParam("testo")->value().toFloat();
      Kd = tune;
      eepromVal = byte(tune);
      EEPROM.write(kdAddr, eepromVal);
      tune = tune - float(eepromVal);
      eepromVal = byte(tune * 100.00);
      EEPROM.write(kdAddrDec, eepromVal);
      request->send(200, "number/plane", String(Kd, 2));
    });
    server.on("/getMm", HTTP_GET, [](AsyncWebServerRequest * request) {
      motorMinVoltage = request->getParam("testo")->value().toInt();
      if (motorMinVoltage > 255) {
        eepromVal = 255;
      } else {
        eepromVal = byte(motorMinVoltage);
      }
      EEPROM.write(mml, eepromVal);
      if (motorMinVoltage > 255) {
        eepromVal = motorMinVoltage - 255;
      } else {
        eepromVal = 0;
      }
      EEPROM.write(mmh, eepromVal);
      request->send(200, "number/plane", String(motorMinVoltage));
    });
    server.on("/getMM", HTTP_GET, [](AsyncWebServerRequest * request) {
      motorMaxVoltage = request->getParam("testo")->value().toInt() / 2;
      if (motorMaxVoltage > 255) {
        eepromVal = 255;
      } else {
        eepromVal = byte(motorMaxVoltage);
      }
      EEPROM.write(mMl, eepromVal);
      if (motorMaxVoltage > 255) {
        eepromVal = motorMaxVoltage - 255;
      } else {
        eepromVal = 0;
      }
      EEPROM.write(mMh, eepromVal);
      motorMaxVoltage = motorMaxVoltage * 2;
      request->send(200, "number/plane", String(motorMaxVoltage));
    });
    server.on("/getZA", HTTP_GET, [](AsyncWebServerRequest * request) {
      double tune = request->getParam("testo")->value().toFloat();
      zeroAngle = tune;
      eepromVal = byte(tune);
      EEPROM.write(za, eepromVal);
      tune = tune - float(eepromVal);
      eepromVal = byte(tune * 100.00);
      EEPROM.write(zadec, eepromVal);
      request->send(200, "number/plane", String(zeroAngle, 2));
    });
    server.on("/getAD", HTTP_GET, [](AsyncWebServerRequest * request) {
      adjustAngle = (pitch - adjustAngle) * -1.00;
      double tune;
      byte sign;
      if (adjustAngle < 0) {
        tune = 0.00 - adjustAngle;
        sign = 0;
      } else {
        tune = adjustAngle;
        sign = 1;
      }
      eepromVal = byte(tune);
      EEPROM.write(ad, eepromVal);
      tune = tune - float(eepromVal);
      eepromVal = byte(tune * 100.00);
      EEPROM.write(add, eepromVal);
      EEPROM.write(ads, sign);
    });
    server.on("/getOffset", HTTP_GET, [](AsyncWebServerRequest * request) {
      offset = request->getParam("testo")->value().toInt() + offset;
      if (offset >= 0) {
        offsetMotorA = offset;
        offsetMotorB = 0;
        eepromVal = byte(offsetMotorA);
        EEPROM.write(mao, eepromVal);
        eepromVal = byte(offsetMotorB);
        EEPROM.write(mbo, eepromVal);
      } else {
        offsetMotorB = abs(offset);
        offsetMotorA = 0;
        eepromVal = byte(offsetMotorB);
        EEPROM.write(mbo, eepromVal);
        eepromVal = byte(offsetMotorA);
        EEPROM.write(mao, eepromVal);
      }
      request->send(200, "number/plane", String(offset));
    });
    server.on("/getReset", HTTP_GET, [](AsyncWebServerRequest * request) {
      ESP.restart();
    });
    server.on("/getSaveAndReset", HTTP_GET, [](AsyncWebServerRequest * request) {
      EEPROM.commit();
      ESP.restart();
    });
    server.on("/getDistance", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plane", String(distance));
    });
    server.on("/getTurnSpeed", HTTP_GET, [](AsyncWebServerRequest * request) {
      turnSpeed = request->getParam("testo")->value().toInt();
      if (turnSpeed > 255) {
        eepromVal = 255;
      } else {
        eepromVal = byte(turnSpeed);
      }
      EEPROM.write(tsl, eepromVal);
      if (turnSpeed > 255) {
        eepromVal = turnSpeed - 255;
      } else {
        eepromVal = 0;
      }
      EEPROM.write(tsh, eepromVal);
      request->send(200, "number/plane", String(turnSpeed));
    });
    server.on("/toggleOTA", HTTP_GET, [](AsyncWebServerRequest * request) {
      String OTA = request->getParam("testo")->value();
      if (OTA.equals("true")) {
        remote = false;
      } else {
        remote = true;
      }

    });

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // initialize device
    mpu.initialize();
    // Initializing DMP
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      // Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable ESP8266 interrupt detection
      pinMode(OTAbutton, INPUT);

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      //pid servo and sr04 configuration
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.SetSampleTime(sampleTime);
      myPID.SetOutputLimits(-1023, 1023);
      //turn the PID on
      myPID.SetMode(AUTOMATIC);
      pTemp = Kp;
      iTemp = Ki;
      dTemp = Kd;
      analogWriteFreq(2000);
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      pinMode(m1, OUTPUT);
      pinMode(m2, OUTPUT);
      pinMode(m3, OUTPUT);
      pinMode(m4, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(echoPin), readPin, CHANGE);
      digitalWrite(trigPin, LOW);  // Added this line
      pinMode(servo, OUTPUT);
      myservo.attach(servo);
      Input = 0.00;
      pinMode(led, OUTPUT);
      server.onNotFound(notFound);
      server.begin();
    }
  }
  /*
    else
    {
    Serial.println("Failed!");
    }
  */
}
///////////////////////////////////////////////////////////////////


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  otaManager();
  if (!OTAstate) {
    measure();
    RCmanager();
    dmpData();
    calculatePID();
  } else {
    ArduinoOTA.handle();
  }
  Serial.println(pitch);
}
////////////////////////////////////////////////////////////////////


//==============================================================
//  sr04 ISR routine attached to the echo pin used as interrupt
//==============================================================


void readPin() { // echo pin interrupt
  if (OTAstate) {
    return;
  }
  if (digitalRead(echoPin) == HIGH) {
    startEndReading = true;
    start = micros();
  } else if (startEndReading && digitalRead(echoPin) == LOW) {
    finish = micros();
    startEndReading = false;
    distanceDataReady = true;
  }
}

//==============================================================
//                  Remote controls function manager
//==============================================================

void RCmanager() {
  checkTurn();
  speedManager();
}

void checkTurn() {
  if (desiredAngle != 0.00) {
    if (!turnStarted) {
      turnStarted = true;
      turnTimer = millis();
    } else if (millis() - turnTimer >= turnTime) {
      desiredAngle = 0.00;
      turnStarted = false;
    }
  }
}

void speedManager() {
  if (speedRobot != 0.00) {
    if (speedRobot >= 0.00) {
      robotDirection = false;
    } else {
      robotDirection = true;
    }
    if (!started) {
      speedTimer = millis();
      started = true;
    } else {
      if (millis() - speedTimer >= speedTime) {
        speedRobot = 0.00;
        started = false;
      }
    }
  }
}

//==============================================================
//OTA, you can enter in this modality remotely or by pressing the button the robot start in this state
//==============================================================

void otaManager() {
  bool temp = digitalRead(OTAbutton) == LOW;
  if (!OTAstate && (!remote || temp)) {
    if (temp) {
      remote = false;
      temp = false;
    }
    stopMotors();
    digitalWrite(led, HIGH);
    OTAstate = true;
    delay(500);
  }
  if (OTAstate && (remote || temp)) {
    if (temp) {
      remote = true;
    }
    digitalWrite(led, LOW);
    OTAstate = false;
    delay(500);
  }
}

//==============================================================
//This function calculate the output from the DMP pitch and send it to the motor controller routine
//==============================================================

void calculatePID() {
  //if (sensorDataSync) {
  // sensorDataSync = false;
  k = fabs(pitch);

  if (k <= 60.00) {
    measureDAngle();
    if (k <= zeroAngle) {
      Input = 0.00;
    } else {
      Input = pitch;
    }

    if (Kp != pTemp || Ki != iTemp || Kd != dTemp) {
      myPID.SetTunings(Kp, Ki, Kd);
      pTemp = Kp;
      iTemp = Ki;
      dTemp = Kd;
    }

    myPID.Compute();
    if (output != Output) {
      output = Output;
      value = fabs(Output);
      motorController();
    }

    if (stopRobot) {
      stopRobot = false;
    }

  } else {
    if (!stopRobot) {
      stopMotors();
      stopRobot = true;
    }
  }
  //}
}

//==============================================================
//This function check for minumum motor voltage, max motor voltage and turn routine
//==============================================================

void motorController() {
  if (value < motorMinVoltage && value != 0.00) {
    value = motorMinVoltage;
  }
  val = int (value);
  val2 = val;
  val = val + offsetMotorA;
  val2 = val2 + offsetMotorB;
  if (desiredAngle < 0.00) { //turning right

    if (robotDirection) { //going backward

      val2 = val2 + turnSpeed;
      checkMotorMaxVoltage();
      goBackwardM34();

      if (Output < 0.00) {
        goBackwardM12();
      } else if (Output > 0.00) {
        goForwardM12();
      } else {
        stopMotors();
      }

    } else { // going forward

      val = val + turnSpeed;
      checkMotorMaxVoltage();
      goForwardM12();

      if (Output < 0.00) {
        goBackwardM34();
      } else if (Output > 0.00) {
        goForwardM34();
      } else {
        stopMotors();
      }

    }

  } else if (desiredAngle > 0.00) { //turning left

    if (robotDirection) { // going backward

      val = val + turnSpeed;
      checkMotorMaxVoltage();
      goBackwardM12();
      if (Output < 0.00) {
        goBackwardM34();
      } else if (Output > 0.00) {
        goForwardM34();
      } else {
        stopMotors();
      }
    } else { // going forward

      val2 = val2 + turnSpeed;
      checkMotorMaxVoltage();
      goForwardM34();
      if (Output < 0.00) {
        goBackwardM12();
      } else if (Output > 0.00) {
        goForwardM12();
      } else {
        stopMotors();
      }
    }

  } else {
    checkMotorMaxVoltage();
    if (Output < 0.00) {
      goBackwardM12();
      goBackwardM34();
    } else if (Output > 0.00) {
      goForwardM12();
      goForwardM34();
    } else {
      stopMotors();
    }
  }
}

//==============================================================
//This function check that the analog write doesn't get over the maximum motor voltage
//==============================================================

void checkMotorMaxVoltage() {
  if (val > motorMaxVoltage) {
    val = motorMaxVoltage;
  }
  if (val2 > motorMaxVoltage) {
    val2 = motorMaxVoltage;
  }
}

//==============================================================
//                  basic analog write funtions
//==============================================================

void goBackwardM12() {
  analogWrite(m1, 1);
  analogWrite(m2, val);
}

void goForwardM34() {
  analogWrite(m3, 1);
  analogWrite(m4, val2);
}

void goForwardM12() {
  analogWrite(m1, val);
  analogWrite(m2, 1);
}

void goBackwardM34() {
  analogWrite(m3, val2);
  analogWrite(m4, 1);
}

void stopM12() {
  analogWrite(m1, 1);
  analogWrite(m2, 1);
}

void stopM34() {
  analogWrite(m3, 1);
  analogWrite(m4, 1);
}

void stopMotors() {
  analogWrite(m1, 1);
  analogWrite(m2, 1);
  analogWrite(m3, 1);
  analogWrite(m4, 1);
}

//==============================================================
//Servo routine, it moves every 3 x times the measure routine run
//==============================================================

void servoMove() {
  if (servoTrigger == 3) {
    servoTrigger = 0;
    obstacleAvoidance();
    if (servoPos == 0) {
      servoDirection = true;
    }
    if (servoPos == 180) {
      servoDirection = false;
    }
    if (servoDirection) {
      servoPos = servoPos + 45;
    } else {
      servoPos = servoPos - 45;
    }
    myservo.write(servoPos);
  }
}

//==============================================================
//This function activate the SR04 every 60ms like in the datasheet, and changes the global distance variable, also it moves the servo if needed
//==============================================================

void measure() {
  unsigned long now = millis();
  if (now - sr04Timer >= 60) {
    if (distanceDataReady) {
      distanceDataReady = false;
      double tempDistance = (finish - start) / 58.00;
      if (distance != tempDistance && tempDistance < 400.00 && tempDistance > 2.00) {
        distance = tempDistance;
      }
    }
    sr04Timer = now;
    servoTrigger++;
    servoMove();
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }
}

//==============================================================
//This function let the robot turn by measuring the actual distance and in 90 degree case, let the robot go backward
//==============================================================

void obstacleAvoidance() {
  if (distance < 15.00) {
    if (servoPos == 90) {
      speedRobot = -2.00;
      speedTime = 250;
    } else {
      speedRobot = -0.50;
      if (servoPos < 90)
        desiredAngle = 5.00;
      if (servoPos > 90)
        desiredAngle = -5.00;
      turnTime = 80;
      speedTime = 80;
    }
  }
}

//==============================================================
//This is my backup function that I created for give more stability to the robot
//==============================================================

void measureDAngle() {

  if (i < 2) {
    i++;
    double j = fabs(pitch);
    if (pitch <= 0.00) {
      minimum = j;
    } else if (pitch >= 0.00) {
      maximum = j;
    }
  }
  if (i > 1) {
    i = 0;
    Setpoint = (minimum - maximum) / 2.00;
  }
}

//==============================================================
//This function changes the pitch global variable, here the adjust angle and speed variable are added ripectively as offset and for kicking the robot, because it doesn't have any meaning of the speed
//You can use the mpu6050 or 9250 using only four pin without connecting the INT pin to the microcontroller
//==============================================================

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

void dmpData() {
  mpuIntStatus = mpu.getIntStatus();
  if (mpuIntStatus & 0x02 || fifoCount >= packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    //dmpReady = false;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      float ypr[3];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      ypr[1] = ypr[1] * 180 / M_PI;
      pitch = adjustAngle + ypr[1] + speedRobot;
      //Serial.println(ypr[1]);
    }
  }
}
