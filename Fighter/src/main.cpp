#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>

//Radio Setup
RF24 radio(10, 9);  // CE, CSN
byte sAddresses[][6] = {"BasT","BasR"};

int buzzerPin = 6;

int rxError =0;
int errorMax =100;

//Radio Pair
typedef struct{
  int id = EEPROM.read(0); // Each student will receive a id number
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

int leftSpeed = 100;
int rightSpeed = 100;
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
  int boost = 0;
}
fighter;
fighter fighterData;

void pairNow(){
  radio.begin();
  radio.setChannel(1);
  radio.openWritingPipe(sAddresses[0]);  
  radio.enableAckPayload();
  radio.stopListening();

  radio.setRetries(3,5); 

  bool rslt;
  rslt = radio.write( &pairData, sizeof(pairData) );
  if (rslt) {
        Serial.println("  Acknowledge received");

      if (radio.isAckPayloadAvailable()) {
        radio.read(&pairData, sizeof(pairData));
        Serial.print("Data Sent Id = ");
        Serial.print(pairData.id);
        Serial.println("  Acknowledge received");
        Serial.println(pairData.paired);

        if (pairData.paired)
        {
        Serial.println("Paired");

          digitalWrite(buzzerPin, HIGH);
          delay(200);
          digitalWrite(buzzerPin, LOW);
          radio.setChannel(pairData.id);
          radio.stopListening();
          radio.openReadingPipe(1, sAddresses[0]);
          radio.startListening();
        } else{
        Serial.println("Not Paired");

        }
        
        
      }
    }
}

int resetPin = 28;

void setup() {
  //Reset ID EEPROM
  // EEPROM.update(0, 255);

  Serial.begin(9600);
  Serial.println("Fighter Init");
  Serial.print("Id: ");
  Serial.println(pairData.id);

  digitalWrite(resetPin, HIGH);   
  pinMode(resetPin, OUTPUT); 
  

  pinMode(leftForward, OUTPUT); 
  pinMode(leftBackward, OUTPUT); 
  pinMode(rightForward, OUTPUT); 
  pinMode(rightBackward, OUTPUT); 

  pinMode(weapon1, OUTPUT); 
  pinMode(weapon2, OUTPUT); 
  digitalWrite(weapon1, HIGH);
  digitalWrite(weapon2, HIGH);

  pinMode(leftPWM, OUTPUT); 
  pinMode(rightPWM, OUTPUT); 

  pinMode(buzzerPin, OUTPUT);
  // digitalWrite(buzzerPin, HIGH);
  // delay(200);
  // digitalWrite(buzzerPin, LOW);
  // delay(500);
  // digitalWrite(buzzerPin, HIGH);
  // delay(500);
  // digitalWrite(buzzerPin, LOW);


  Serial.println("Fighter Ready!");
  Serial.println("Commands available:");
  Serial.println("| i - ID | m - Motors | w - Weapon");


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
    digitalWrite(rightBackward, HIGH);
    digitalWrite(leftBackward, HIGH);
  } else if (ctrlData.left){
    Serial.println("left");    
    digitalWrite(rightForward, HIGH);
    digitalWrite(leftBackward, HIGH);
  }  else if (ctrlData.right){
    Serial.println("right");    
    digitalWrite(rightBackward, HIGH);
    digitalWrite(leftForward, HIGH);
  } else{
    digitalWrite(rightForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightBackward, LOW);
    digitalWrite(leftForward, LOW);
  }

  if (ctrlData.btn4)
      {
      // Serial.println("Weapon");

        digitalWrite(weapon1, LOW);
        digitalWrite(weapon2, LOW);
      } else{
        digitalWrite(weapon1, HIGH);
        digitalWrite(weapon2, HIGH);

      }

  analogWrite(leftPWM, map(leftSpeed, 0, 100, 0, 255));
  analogWrite(rightPWM, map(rightSpeed, 0, 100, 0, 255));

}

void motor(){
      analogWrite(leftPWM, map(80, 0, 100, 0, 255));
      analogWrite(rightPWM, map(80, 0, 100, 0, 255));
      Serial.println("rightForward");
      digitalWrite(rightForward, HIGH);
      delay(2000);
      digitalWrite(rightForward, LOW);
      Serial.println("rightBackward");
      digitalWrite(rightBackward, HIGH);
      delay(2000);
      digitalWrite(rightBackward, LOW);
      Serial.println("leftForward");
      digitalWrite(leftForward, HIGH);
      delay(2000);
      digitalWrite(leftForward, LOW);
      Serial.println("leftBackward");
      digitalWrite(leftBackward, HIGH);
      delay(2000);
      digitalWrite(leftBackward, LOW);
      delay(2000);
      Serial.println("All Forward");
      digitalWrite(leftForward, HIGH);
      digitalWrite(rightForward, HIGH);
      delay(2000);
      Serial.println("Max Speed!");
      analogWrite(leftPWM, map(100, 0, 100, 0, 255));
      analogWrite(rightPWM, map(100, 0, 100, 0, 255));
      delay(2000);
      digitalWrite(leftForward, LOW);
      digitalWrite(rightForward, LOW);
      analogWrite(leftPWM, map(80, 0, 100, 0, 255));
      analogWrite(rightPWM, map(80, 0, 100, 0, 255));
}

void weapon(){
  Serial.println("Get Ready");
  delay(2000);
  Serial.println("Weapon 1");
  digitalWrite(weapon1, LOW);
  delay(2000);
  digitalWrite(weapon1, HIGH);
  Serial.println("Weapon 2");
  digitalWrite(weapon2, LOW);
  delay(2000);
  digitalWrite(weapon2, HIGH);
}

int newID;
String idString = "";
void loop() {
  // Serial.println(rxError);
  digitalWrite(buzzerPin, LOW);

  unsigned long currentTime = millis();

  float voltage = analogRead(A7);  // It reads the input pin  
  // voltage = voltage * (3.3 / 1023);
  fighterData.battery = mapFloat(voltage, 0, 650, 0, 12);  
  // Serial.print("Voltage: ");    
  // Serial.println(voltage);    
  // Serial.println(fighterData.battery);  
  
  
  if (Serial.available()) {
    String inByte = Serial.readString();
    Serial.println(inByte);

    if (inByte == "i"){
      Serial.println("Reseting id:");
      pairData.id = 255;
    }
    if (inByte == "m"){
      Serial.println("Running motors");
      motor();
      Serial.println("Done!");

    }
    if (inByte == "w"){
      Serial.println("Running weapons");
      weapon();
      Serial.println("Done!");
    }

  }

if (pairData.id == 255){
    Serial.println("Please set Fighter ID: ");
    idString = "";
  }
  while (pairData.id == 255){
      if (Serial.available()) {
      String inByte = Serial.readString();
      if (inByte == "e"){
        Serial.println("");
        Serial.println("Saving id:");
        newID = idString.toInt();
        Serial.println(newID);
        EEPROM.update(0, newID);
        pairData.id = newID;
        digitalWrite(resetPin, LOW); 
        setup();
      }
      idString = idString + inByte;
      Serial.println(idString);
      
    }
  }
  
  if (!pairData.paired)
  {
    pairNow();
  } else {
    radio.openReadingPipe(1, sAddresses[0]);
    radio.startListening();
    if (radio.available()){
      rxError =0;
      radio.read(&ctrlData, sizeof(ctrlData));
      radio.stopListening();
      radio.openWritingPipe(sAddresses[1]);  
      radio.setRetries(1,0);
      radio.write( &fighterData, sizeof(fighterData) );
      move();
    } else{
      rxError = rxError + 1;
      if (rxError > errorMax)
      {
        Serial.println("Rx failed");
        digitalWrite(buzzerPin, HIGH);
        delay(100);

        // Serial.println(rxError);
        digitalWrite(weapon1, HIGH);
        digitalWrite(weapon2, HIGH);
        digitalWrite(rightForward, LOW);
        digitalWrite(leftBackward, LOW);
        digitalWrite(rightBackward, LOW);
        digitalWrite(leftForward, LOW);

      }
      
      
    }
    
  }
  previousTime = currentTime;
}
