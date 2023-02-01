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
  int id = 0; // Each student will receive a id number
  bool paired = false;
}
pair;
pair pairData;


// Controller Ports
int leftBtn = A2;
bool leftValue = 0;
int rightBtn = A3;
bool rightValue = 0;
int upBtn = A0;
bool upValue = 0;
int downBtn = A1;
bool downValue = 0;

int actionBtn = A4;
bool actionValue = 0;
int boostBtn = A5;
bool boostValue = 0;

int leftSpeed = 0;
int rightSpeed = 0;

int red_light_pin= 6;
int green_light_pin = 10;
int blue_light_pin = 9;

int pairButton= 2;
boolean pairbuttonState;

//Controller Data
typedef struct{
  int leftSpeed = 90;   /*These values will be used as the initial values*/
  int rightSpeed = 90;  
  bool leftTrigger = 0;
  bool rightTrigger = 0;
}
ctrl;
ctrl ctrlData;

void RGB_color(int red_light_value, int green_light_value, int blue_light_value){
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

void pairNow(){
  delay(500);
  pairData.id = 0;
  pairData.paired = false;
  radio.begin();
  radio.setChannel(1);
  radio.stopListening();
  radio.openReadingPipe(1, sAddresses[0]);
  radio.startListening();
  radio.write( &pairData, sizeof(pairData) );  

    while (!radio.available())
    {
      Serial.println("No signal");    
    }

    if (radio.available())
    {
      pairbuttonState = false;
      Serial.println("Received");
      radio.read(&pairData, sizeof(pairData));
      Serial.println(pairData.id);
      pairData.paired = true;
      radio.setChannel(pairData.id);
      radio.stopListening();
      radio.openWritingPipe(sAddresses[0]);      
    }
  
}

void getController(){
  //Serial.println("Get controls");
   pairbuttonState = !digitalRead(pairButton);
 

  if (analogRead(leftBtn) <= 100)
  {
    leftValue = true;
  }else{
    leftValue = false;
  }
  if (analogRead(rightBtn)<= 100)
  {
    rightValue = true;
  }else{
    rightValue = false;
  }
  if (analogRead(upBtn) <= 100)
  {
    upValue = true;
  }else{
    upValue = false;
  }
  if (analogRead(downBtn) <= 100)
  {
    downValue = true;
  }else{
    downValue = false;
  }

  if (analogRead(actionBtn) <= 100)
  {
    actionValue = true;
  }else{
    actionValue = false;
  }
    if (analogRead(boostBtn) <= 100)
  {
    boostValue = true;
  }else{
    boostValue = false;
  }
  // Serial.println(analogRead(rightBtn));

  // Serial.println(analogRead(boostBtn));
  // Serial.println(analogRead(upBtn));
  // Serial.println(upValue);
  // Serial.println(downValue);
   //Serial.println(leftValue);

  leftSpeed = 0;
  rightSpeed = 0;

  

  if (upValue)
  {
    leftSpeed = 90;
    rightSpeed = 90;
  }
  if (downValue)
  {
    leftSpeed = 50;
    rightSpeed = 50;
  }

  if (leftValue)
  {
    leftSpeed = 50;
    rightSpeed = 90;
  }

  if (rightValue)
  {
    leftSpeed = 90;
    rightSpeed = 50;
  }

  //Serial.println(leftSpeed);
  // Serial.println(rightSpeed);  
}

void sendData(){
  ctrlData.leftTrigger = actionValue;
  ctrlData.rightTrigger = boostValue;  
  ctrlData.leftSpeed = leftSpeed;
  ctrlData.rightSpeed = rightSpeed;
  // Serial.println(ctrlData.rightTrigger);
  radio.write( &ctrlData, sizeof(ctrlData) );
}

void setup() {
  Serial.begin(9600);
  Serial.println("Remote Init");

  if (!autoPair)
  {
    radio.begin();
    radio.setChannel(pairData.id);
    radio.openWritingPipe(sAddresses[0]); 
    radio.stopListening();
    Serial.println("Manual Pair");
    pairData.paired = true;
  } else{
    Serial.println("Auto Pair");
  } 

  pinMode(actionBtn,INPUT_PULLUP);
  pinMode(boostBtn,INPUT_PULLUP);
  pinMode(pairButton,INPUT_PULLUP);

  pinMode(leftBtn,INPUT_PULLUP);
  pinMode(rightBtn,INPUT_PULLUP);
  pinMode(upBtn,INPUT_PULLUP);
  pinMode(downBtn,INPUT_PULLUP);

  pinMode(red_light_pin,OUTPUT);
  pinMode(green_light_pin,OUTPUT);
  pinMode(blue_light_pin,OUTPUT);


  //pairbuttonState = true;

    RGB_color(255, 0, 0); // Red
  //radio.setPALevel(RF24_PA_LOW); //Default Max Power

}

void loop() {  
  getController();
  if (pairbuttonState)
  {
    pairData.paired = false;
    Serial.println("Pair");
    RGB_color(0, 0, 255); // Blue
    pairNow();
  }
  
  if (!pairData.paired)
  { 
    Serial.println("Not Paired");

    RGB_color(255, 0, 0); // Red
  }else {
    // Serial.println("Paired");
    RGB_color(255, 255, 255); // White
    sendData();  
  }
  
}