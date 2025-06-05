//Libraries
#include <Motoron.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

//Wifi init
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
IPAddress local_ip(192, 168, 0, 56);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
const unsigned int localPort = 55500;
WiFiUDP udp;
char incomingPacket[64];

//Servo init
Servo digitalServo1;  
Servo digitalServo2;  
const int servoPin1 = 40;
const int servoPin2 = 42;
int currentCommand = -1;
int currentCommand2 = 4;

//QTRRC init
const uint8_t RFLpins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39}; // GPIOs connected to the 9 sensor outputs
const uint8_t RFLCount = 18;
uint16_t RFLValues[RFLCount];
int RFLstate = 0;
int split = 0;

//Button init
const int buttonPin1 = 41;
bool buttonPressed1 = false;
bool buttonState = true;
int button_count1 = 0;
const int buttonPin2 = 43;
int buttonPressed2 = false;
int button_count2 = 1;

//LDR init
int LDRValue;

//Distance init
const int DistanceSensorPins[3] = {A0, A1};
float distances[2];
int distance_count = 1;

//MotoronI2C init
MotoronI2C mc;
int dash_count = 1;

//QTRRC read values
void readQTR_RC(uint16_t* readings) {
  // Charge capacitors
  for (uint8_t i = 0; i < RFLCount; i++) {
    pinMode(RFLpins[i], OUTPUT);
    digitalWrite(RFLpins[i], HIGH);
  }
  delayMicroseconds(10);

  // Set to input (start discharge)
  for (uint8_t i = 0; i < RFLCount; i++) {
    pinMode(RFLpins[i], INPUT);
  }

  // Measure time to LOW
  unsigned long startTime = micros();
  unsigned long timeout = 3000; // Max wait time in µs
  bool done[RFLCount] = {false};

  while ((micros() - startTime) < timeout) {
    for (uint8_t i = 0; i < RFLCount; i++) {
      if (!done[i] && digitalRead(RFLpins[i]) == LOW) {
        readings[i] = micros() - startTime;
        done[i] = true;
      }
    }
  }

  // Default to timeout if not discharged
  for (uint8_t i = 0; i < RFLCount; i++) {
    if (!done[i]) {
      readings[i] = timeout;
    }
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(9600); 

  //Motoron setup
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);
  mc.setMaxAcceleration(3, 80);
  mc.setMaxDeceleration(3, 300);

  //Button setup
  pinMode(buttonPin1, INPUT_PULLUP); 
  pinMode(buttonPin2, INPUT_PULLUP); 

  //Servo setup
  digitalServo1.attach(servoPin1);
  digitalServo2.attach(servoPin2);

  // Wi-Fi setup
  WiFi.config(local_ip, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localPort);
}

void button_press_detection()
{
  if (digitalRead(buttonPin1) == LOW) {
    if (!buttonPressed1) {
      buttonPressed1 = true;
      button_count1 += 1;
      if (button_count1 % 2 == 0){
        buttonState = true;
      }
      else {
        buttonState = false;
      }
    }
  } else {
    buttonPressed1 = false;
  }

  if (digitalRead(buttonPin2) == LOW) {
    if (!buttonPressed2) {
      buttonPressed2 = true;
      button_count2 += 1;
      if (button_count2 % 2 == 0){
        currentCommand2 = 4;
      }
      else{
        currentCommand2 = 3;
      }
    }
  } else {
    buttonPressed2 = false;
  }
}

void print_sensorvalues()
{
  // Print button values
  Serial.println(button_count1);
  Serial.println(button_count2);

  readQTR_RC(RFLValues); 

  // Print RFL values
  for (int i = 0; i < RFLCount; i++) {
    Serial.print(RFLValues[i]);
    Serial.print(" ");
  }
  Serial.print(" ");

  // Print LDR values
  LDRValue = analogRead(A2); 
  Serial.print(LDRValue);   
  Serial.print("   ");

  // Print Distance Sensor values
  for (int i = 0; i < 2; i++) {
    int raw = analogRead(DistanceSensorPins[i]);
    float voltage = raw * (5.0 / 1023.0);
    float distance = 12.08 * pow(voltage, -1.058);
    if (distance > 30) distance = 30;

    distances[i] = distance;
    Serial.print(distance, 1);
    if (i < 2) Serial.print(" ");
  }
  Serial.print("  ");

  Serial.println(); 
}

void udp_message_detection()
{
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    memset(incomingPacket, 0, sizeof(incomingPacket));
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;

      if (String(incomingPacket) == "Stop") {
        buttonState = false;
        button_count1 += 1;
      }
    }
  }
}

void input_commands()
{
  if (Serial.available()) {
    char input = Serial.read();

    if (input == '1') {
      currentCommand2 = 1;
    } else if (input == '2') {
      currentCommand2 = 2;
    } else if (input == '3') {
      currentCommand2 = 3;
    } else if (input == '4') {
      currentCommand2 = 4;
    } else if (input == '0') {
      currentCommand = 0;
    } else if (input == '5') {
      currentCommand = 5;
    } else if (input == '6') {
      currentCommand = 6;
    } else if (input == '7') {
      currentCommand = 7;
    } else if (input == '8') {
      currentCommand = 8;
    }
  }
}

void line_following()
{
  if ((RFLValues[0] < 230 && RFLValues[1] < 230 && RFLValues[2] < 230 && RFLValues[3] < 230 && RFLValues[14] < 230 && RFLValues[15] < 230 && RFLValues[16] < 230 && RFLValues[17] < 230) && ((RFLValues[8] > 180 && RFLValues[9] > 180) || (RFLValues[7] > 180 && RFLValues[8] > 180) || (RFLValues[9] > 180 && RFLValues[10] > 180) || (RFLValues[8] > 180) || (RFLValues[9] > 180) || (RFLValues[7] > 180 && RFLValues[8] > 180 && RFLValues[9] > 180 && RFLValues[10] > 180))) {
    dash_count = 1;
    mc.setSpeed(2, 100);
    mc.setSpeed(3, 100); //forward
  } else if (RFLValues[0] > 800 && RFLValues[3] > 800 && RFLValues[6] > 800 && RFLValues[9] > 800 && RFLValues[10] > 800 && RFLValues[11] > 800 && RFLValues[14] > 800 && RFLValues[17] > 800) {
    mc.setSpeed(2, 0);
    mc.setSpeed(3, 0); // stop
    buttonPressed2 = false;
  } else if (RFLValues[0] > 250 && RFLValues[8] < 350 && RFLValues[9] < 350 && RFLValues[17] > 250) {
    if (split == 0) {
      dash_count = 1;
      mc.setSpeed(2, 150);
      mc.setSpeed(3, -175);
      delay(1000); //split
      split = 1;
    }
    else if (split == 2) {
      mc.setSpeed(2, -100);
      mc.setSpeed(3, -100);
      if (RFLValues[0] > 250 && RFLValues[8] < 350 && RFLValues[9] < 350 && RFLValues[17] > 250) {
        mc.setSpeed(2, -175);
        mc.setSpeed(3, 150);
        split = -1;
      }
    }
  } else if (RFLValues[0] > 230 || RFLValues[1] > 230 || RFLValues[2] > 230) {
    dash_count = 1;
    mc.setSpeed(2, -200);
    mc.setSpeed(3, 400); // large turn
  } else if (RFLValues[3] > 230 || RFLValues[4] > 230 ||RFLValues[5] > 230 || RFLValues[6] > 230 || RFLValues[7] > 230) {
    dash_count = 1;
    mc.setSpeed(2, -100);
    mc.setSpeed(3, 250); // small turn
  } else if (RFLValues[15] > 230 || RFLValues[16] > 230 || RFLValues[17] > 230) {
    dash_count = 1;
    mc.setSpeed(2, 400);
    mc.setSpeed(3, -200); // large turn
  } else if (RFLValues[10] > 230 || RFLValues[11] > 230 || RFLValues[12] > 230 || RFLValues[13] > 230 || RFLValues[14] > 230) {
    dash_count = 1;
    mc.setSpeed(2, 250);
    mc.setSpeed(3, -100); // small turn
  } else if (RFLValues[0] < 220 && RFLValues[3] < 220 && RFLValues[6] < 220 && RFLValues[9] < 220 && RFLValues[10] < 220 && RFLValues[11] < 220 && RFLValues[14] < 220 && RFLValues[17] < 220) {
    if (dash_count == 1) {
      mc.setSpeed(2, 100);
      mc.setSpeed(3, 100);
      delay(350);
      dash_count = 0;
    } 
    else {
      mc.setSpeed(2, -100);
      mc.setSpeed(3, -100); // error
      if (split == 1) {
        split = 2;
      }
    }
  } else {
    mc.setSpeed(2, 200);
    mc.setSpeed(3, 200);
  }
}

void wall_following()
{
  if (distances[0] >= 18) {
    distance_count = 1;
    mc.setSpeed(2, 200);
    mc.setSpeed(3, 200); // forward
  }
  else if (distances[0] < 18 && distances[1] < 18) {
    if (distance_count = 1) {
      mc.setSpeed(2, 400);
      mc.setSpeed(3, -400);
      delay(800);
      distance_count = 0; // turn
    }
    else {
      mc.setSpeed(2, 200);
      mc.setSpeed(3, 200);
    }
  }
  else {
    distance_count = 1;
    mc.setSpeed(2, -150);
    mc.setSpeed(3, -150); // error
  }
}

void lava_pit()
{
  int star = millis();
  int end = millis();
  while ((end - star) < 6000) {
    mc.setSpeed(2, 400);
    mc.setSpeed(3, 400);
    end = millis();
  }

  int star = millis();
  int end = millis();
  while ((end - star) < 1000) {
    mc.setSpeed(2, 100);
    mc.setSpeed(3, 100);
    end = millis();
  }
  digitalServo1.write(55);
  digitalServo2.write(85);
  star = millis();
  end = millis();
  while ((end - star) < 5000) {
    mc.setSpeed(2, 400);
    mc.setSpeed(3, 400);
    end = millis();
  }
  digitalServo2.write(0);
  digitalServo1.write(160);
}

void loop()
{
  mc.clearResetFlag();
  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_BRAKE);
  
  button_press_detection();

  udp_message_detection();

  input_commands();

  if (buttonState == true){
    Serial.print("Start  ");

    if (currentCommand == 0) {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      mc.setSpeed(3, 0);
    } else if (currentCommand == 5) {
      mc.setSpeed(2, 800);
      mc.setSpeed(3, 800);
    } else if (currentCommand == 6) {
      mc.setSpeed(2, -800);
      mc.setSpeed(3, -800);
    } else if (currentCommand == 7) {
      mc.setSpeed(2, 500);
      mc.setSpeed(3, -200);
    } else if (currentCommand == 8) {
      mc.setSpeed(2, -200);
      mc.setSpeed(3, 500);
    }

    if (currentCommand2 == 1) {
      digitalServo2.write(160);
    } else if (currentCommand2 == 2) {
      digitalServo2.write(85);
    } else if (currentCommand2 == 3) {
      digitalServo1.write(45);
      RFLstate = 1;
    } else if (currentCommand2 == 4) {
      digitalServo1.write(0);
      RFLstate = 0;
    } 

    //line following
    if (RFLstate == 0) {
      line_following();
    }

    //wall following
    else if (RFLstate == 1) {
      wall_following();
    }

    //lava pit
    else if (RFLstate == 2) {
      lava_pit();
    }
  }
  else if (buttonState == false)
  {
    Serial.print("Stop  ");
    mc.setSpeed(1, 0);
    mc.setSpeed(2, 0);
    mc.setSpeed(3, 0);
  }

  delay(100); 
}
