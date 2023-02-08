#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Radio Setup
RF24 radio(10, 9);  // CE, CSN
byte sAddresses[][6] = {"BasT","BasR"};

int buzzerPin = 6;

//Radio Pair
typedef struct{
  int id = 11; // Each student will receive a id number
  bool paired = false;
}
pair;
pair pairData;

// Motor Driver Ports
int leftPWM = 3;
int leftForward = A4;
int leftBackward = A5;
int rightForward = A1;
int rightBackward = A2;
int rightPWM = 5;

int leftSpeed = 60;
int rightSpeed = 60;
bool boosting = 0;
unsigned long boostTime = 5000;
unsigned long boostRecharge = 5000;
unsigned long previousTime = 0;

int ledBoostRecharge = 1;

int weapon1 = 4;
int weapon2 = 2;

//Controller Data
typedef struct{
  bool forward = 0;   // debug value
  bool backward = 0;
  bool left = 0;
  bool right = 0;
  bool btn1 = 0;
  bool btn2 = 0;
  bool btn3 = 0;
  bool btn4 = 0;
}
ctrl;
ctrl ctrlData;

//Fighter data
typedef struct{
  int id = pairData.id;
  float battery = 11; 
}
fighter;
fighter fighterData;

void pairNow(){
  radio.begin();
  radio.setChannel(1);
  radio.openWritingPipe(sAddresses[0]);  
  radio.stopListening();

  radio.setRetries(3,5); 

  bool rslt;
  rslt = radio.write( &pairData, sizeof(pairData) );
  if (rslt) {
        Serial.print("Data Sent Id = ");
        Serial.print(pairData.id);
        Serial.println("  Acknowledge received");
        pairData.paired = true;
        digitalWrite(buzzerPin, HIGH);
        delay(200);
        digitalWrite(buzzerPin, LOW);
        radio.setChannel(pairData.id);
        radio.stopListening();
        radio.openReadingPipe(1, sAddresses[0]);
        radio.startListening();
    }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Fighter Init");

  pinMode(leftForward, OUTPUT); 
  pinMode(leftBackward, OUTPUT); 
  pinMode(rightForward, OUTPUT); 
  pinMode(rightBackward, OUTPUT); 

  pinMode(weapon1, OUTPUT); 
  pinMode(weapon2, OUTPUT); 

  pinMode(leftPWM, OUTPUT); 
  pinMode(rightPWM, OUTPUT); 

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);
  delay(500);
  digitalWrite(buzzerPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);



  // digitalWrite(leftForward, HIGH);
}
float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void move(){
  if (ctrlData.forward){
    Serial.println("forward");    
    digitalWrite(rightForward, HIGH);
    digitalWrite(leftForward, HIGH);
  } else if (ctrlData.backward){
    Serial.println("backward");    
    digitalWrite(rightForward, HIGH);
    digitalWrite(leftForward, HIGH);
  } else if (ctrlData.left){
    Serial.println("left");    
    digitalWrite(rightBackward, HIGH);
    digitalWrite(leftForward, HIGH);
  }  else if (ctrlData.right){
    Serial.println("right");    
    digitalWrite(rightForward, HIGH);
    digitalWrite(leftBackward, HIGH);
  } else{
    digitalWrite(rightForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightBackward, LOW);
    digitalWrite(leftForward, LOW);
  }

  if (ctrlData.btn4)
      {
      // Serial.println("Weapon");

        digitalWrite(weapon1, HIGH);
        digitalWrite(weapon2, HIGH);
      } else{
        digitalWrite(weapon1, LOW);
        digitalWrite(weapon2, LOW);

      }

  analogWrite(leftPWM, map(leftSpeed, 0, 100, 0, 255));
  analogWrite(rightPWM, map(rightSpeed, 0, 100, 0, 255));

}

void loop() {

  unsigned long currentTime = millis();

  float voltage = analogRead(A3);  // It reads the input pin  
  // voltage = voltage * (3.3 / 1023);
  fighterData.battery = mapFloat(voltage, 0, 650, 0, 12);  
  // Serial.print("Voltage: ");    
  // Serial.println(voltage);    
 
  if (!pairData.paired)
  {
    pairNow();
  } else {
    radio.openReadingPipe(1, sAddresses[0]);
    radio.startListening();
    if (radio.available()){
      radio.read(&ctrlData, sizeof(ctrlData));
      radio.stopListening();
      radio.openWritingPipe(sAddresses[1]);  
      radio.setRetries(1,0);
      radio.write( &fighterData, sizeof(fighterData) );
      move();
    } else{
      // Serial.println("Rx failed");
    }
    
  }
  previousTime = currentTime;
}
