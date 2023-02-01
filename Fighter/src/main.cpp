#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//Radio Setup
RF24 radio(7, 8);  // CE, CSN
byte sAddresses[][6] = {"BasT","BasR"};

const bool autoPair = true;

//Radio Pair
typedef struct{
  int id = 105; // Each student will receive a id number
  bool paired = false;
}
pair;
pair pairData;


// Motor Driver Ports
int leftPWM = 10;
int leftForward = A0;
int leftBackward = A1;
int rightForward = A2;
int rightBackward = A3;
int rightPWM = 9;

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
  int leftSpeed = 0;   // debug value
  int rightSpeed = 0;
  bool leftTrigger = 0;
  bool rightTrigger = 0;
}
ctrl;
ctrl ctrlData;

void pairNow(){
  radio.begin();
  radio.setChannel(1);
  // radio.openReadingPipe(1, sAddresses[0]);
  radio.openWritingPipe(sAddresses[0]);  
  radio.stopListening();
  radio.setPALevel(RF24_PA_LOW); //Default Max Power
  // radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5); // delay, count
  // radio.startListening();

  bool rslt;
  rslt = radio.write( &pairData, sizeof(pairData) );

  Serial.print("Data Sent Id = ");
  Serial.print(pairData.id);

  if (rslt) {
        Serial.println("  Acknowledge received");
        pairData.paired = true;
        radio.setChannel(pairData.id);
        radio.stopListening();
        radio.openReadingPipe(1, sAddresses[0]);
        radio.startListening();
    }
    else {
        Serial.println("  Tx failed");
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

  // digitalWrite(leftForward, HIGH);

  if (!autoPair)
  {
    radio.begin();
    radio.setChannel(pairData.id);
    radio.openReadingPipe(1, sAddresses[0]);
    radio.startListening();
    Serial.println("Manual Pair");
    pairData.paired = true;
  } else{
    Serial.println("Auto Pair");
  } 


}

void loop() {
  unsigned long currentTime = millis();
  if (!pairData.paired)
  {
    pairNow();
  } else {
    if (radio.available())
    {
      // Serial.println("Received");
      radio.read(&ctrlData, sizeof(ctrlData));
      // Serial.println(ctrlData.leftSpeed);
      // Serial.println(ctrlData.rightSpeed);
      // Serial.println(ctrlData.leftTrigger);
      // Serial.println(ctrlData.rightTrigger);

      if (ctrlData.leftSpeed == 90)
      {
        digitalWrite(leftBackward, LOW);
        digitalWrite(leftForward, HIGH);
      } else if (ctrlData.leftSpeed == 50)
      {
        digitalWrite(leftBackward, HIGH);
        digitalWrite(leftForward, LOW);
      }else{
        digitalWrite(leftBackward, LOW);
        digitalWrite(leftForward, LOW);
      }

      if (ctrlData.rightSpeed == 90)
      {
        digitalWrite(rightBackward, LOW);
        digitalWrite(rightForward, HIGH);
      } else if (ctrlData.rightSpeed == 50)
      {
        digitalWrite(rightBackward, HIGH);
        digitalWrite(rightForward, LOW);
      }else{
        digitalWrite(rightBackward, LOW);
        digitalWrite(rightForward, LOW);
      }
      
      if (ctrlData.leftTrigger)
      {
      Serial.println("Weapon");

        digitalWrite(weapon1, HIGH);
        digitalWrite(weapon2, HIGH);
      } else{
        digitalWrite(weapon1, LOW);
        digitalWrite(weapon2, LOW);

      }
      if (ctrlData.rightTrigger)
      {
        if (currentTime <= previousTime ) {
          /* Event code */
          Serial.println("Recharging!");

        } else
        {
        
          Serial.println("Boost");
          boosting = 1;
          previousTime = currentTime;
          }
      } 

      if (boosting){
        
        
        // Serial.println("check boost");
        Serial.println(currentTime);
        Serial.println(previousTime);
        Serial.println(currentTime-previousTime);

        /* This is the event */
        if (currentTime - previousTime <= boostTime) {
          /* Event code */
          Serial.println("Boosting!");
          
        /* Update the timing for the next time around */
          // previousTime = currentTime;
          leftSpeed = 100;
          rightSpeed = 100;
        } else
        {
          Serial.println("Boosting done");
          leftSpeed = 60;
          rightSpeed = 60;
          boosting = 0;
          previousTime = currentTime + boostRecharge;
        }
        
      }

      analogWrite(leftPWM, map(leftSpeed, 0, 100, 0, 255));
      analogWrite(rightPWM, map(rightSpeed, 0, 100, 0, 255));

    }

  }

}
