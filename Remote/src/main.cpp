#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN_NEO_PIXEL  3   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     24  // The number of LEDs (pixels) on NeoPixel

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);


//Radio Setup
RF24 radio(10, 9);  // CE, CSN
byte sAddresses[][6] = {"BasT","BasR"};

int buzzerPin = 6;

//Radio Pair
typedef struct{
  int id = 0; // Each student will receive a id number
  bool paired = false;
}
pair;
pair pairData;
int pairId= 0;

// Controller Ports
int leftBtn = A3;
int rightBtn = A2;
int upBtn = A0;
int downBtn = A1;

int btn1 = A4;
int btn2 = A5;
int btn3 = A6;
int btn4 = A7;


int leftSpeed = 0;
int rightSpeed = 0;

int errorCount = 0;

int pairButton= 2;
boolean pairbuttonState;

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
  int id = 0;
  float battery = 0; 
  int boost = 0;
}
fighter;
fighter fighterData;

void pairNow(){
  delay(500);
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // for each pixel
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 10, 10)); // it only takes effect if pixels.show() is called
  }
  NeoPixel.show();
  pairData.id = 0;
  pairData.paired = false;
  radio.begin();
  radio.setChannel(1);
  radio.enableAckPayload();
  radio.stopListening();
  radio.openReadingPipe(1, sAddresses[0]);
  radio.startListening();
  radio.write( &pairData, sizeof(pairData) );  
  while (!radio.available())
  {
    // Serial.println("No signal");    
  }
  
  if (radio.available())
  {
    pairbuttonState = false;
    // Serial.println("Received");
    radio.read(&pairData, sizeof(pairData));
    pairData.paired = true;
    pairId = pairData.id;
    radio.writeAckPayload(1, &pairData, sizeof(pairData));
    // Serial.println(pairData.id);
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    pairData.paired = true;
    radio.setChannel(pairData.id);
    radio.stopListening();
    radio.openWritingPipe(sAddresses[0]);      
  }
  
}

void getController(){
  pairbuttonState = !digitalRead(pairButton);

  if (analogRead(leftBtn) <= 100)
  {
    ctrlData.left = 1;
  }else{
    ctrlData.left = 0;
  }
  if (analogRead(rightBtn)<= 100)
  {
    ctrlData.right = 1;
  }else{
    ctrlData.right = 0;
  }
  if (analogRead(upBtn) <= 100)
  {
    ctrlData.forward = 1;
  }else{
    ctrlData.forward = 0;
  }
  if (analogRead(downBtn) <= 100)
  {
    ctrlData.backward = 1;
  }else{
    ctrlData.backward = 0;
  }
  // Serial.println(analogRead(btn4));
  if (analogRead(btn1) <= 100)
  {
    // Serial.println("Btn1");

    ctrlData.btn1 = true;
  }else{
    ctrlData.btn1 = false;
  }
    if (analogRead(btn2) <= 100)
  {
    // Serial.println("Btn2");

    ctrlData.btn2 = true;
  }else{
    ctrlData.btn2 = false;
  }
  if (analogRead(btn3) <= 100)
  {
    // Serial.println("Btn3");

    ctrlData.btn3 = true;
  }else{
    ctrlData.btn3 = false;
  }
    if (analogRead(btn4) <= 100)
  {
    // Serial.println("Btn4");

    ctrlData.btn4 = true;
  }else{
    ctrlData.btn4 = false;
  }

}

void sendData(){

  bool rslt;
  radio.stopListening();
  radio.setRetries(3,5);
  radio.openWritingPipe(sAddresses[0]);  
  rslt = radio.write( &ctrlData, sizeof(ctrlData) );
  radio.openReadingPipe(1, sAddresses[1]);
  radio.startListening();
  if (rslt) {
            // Serial.println("  Acknowledge received");
            while (radio.available()){
              for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // for each pixel
                NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 10, 0)); // it only takes effect if pixels.show() is called
              }
              
              // Serial.println(" Fighter Acknowledge received");
              radio.read(&fighterData, sizeof(fighterData));
              // Serial.println(fighterData.id);
              int batteryStat = map(fighterData.battery, 8.4 ,12 , 1, 24);
              // Serial.print("Volts: ");
              // for (int pixel = 0; pixel < NUM_PIXELS-batteryStat; pixel++) { // for each pixel
              //   NeoPixel.setPixelColor(pixel, NeoPixel.Color(30, 10, 0)); // it only takes effect if pixels.show() is called
              // }

              for (int pixel = 0; pixel < NUM_PIXELS-batteryStat; pixel++) { // for each pixel
                NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 10, 80)); // it only takes effect if pixels.show() is called
              }

              NeoPixel.show();

              // Serial.println(fighterData.battery);
              // Serial.println(batteryStat);

              errorCount = 0;
            }
        }
        else {
            Serial.println("  Tx failed");
            errorCount = errorCount + 1; 
            if (errorCount >= 20)
            {
              digitalWrite(buzzerPin, HIGH);
              delay(100);
              for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // for each pixel
                NeoPixel.setPixelColor(pixel, NeoPixel.Color(10, 0, 0)); // it only takes effect if pixels.show() is called
              }
              NeoPixel.show();

            radio.setChannel(1);
            radio.stopListening();
            radio.openReadingPipe(1, sAddresses[0]);
            radio.startListening();
            radio.write( &pairData, sizeof(pairData) );  
            
            
            if (radio.available())
            {
              pairbuttonState = false;
              // Serial.println("Received");
              radio.read(&pairData, sizeof(pairData));
              if (pairData.id == pairId){
                pairData.paired = true;
                radio.writeAckPayload(1, &pairData, sizeof(pairData));
                // Serial.println(pairData.id);
                digitalWrite(buzzerPin, HIGH);
                delay(100);
                digitalWrite(buzzerPin, LOW);
                pairData.paired = true;
                 
              }
              
                   
            }
            radio.setChannel(pairData.id);
            radio.stopListening();
            radio.openWritingPipe(sAddresses[0]);
            }
        }
}


void setup() {
  Serial.begin(9600);
  Serial.println("Remote Init");
  NeoPixel.begin();
  // turn off all pixels for two seconds
  NeoPixel.clear();
  // turn on all pixels to red at the same time for two seconds
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // for each pixel
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 0, 10)); // it only takes effect if pixels.show() is called
  }
  NeoPixel.show();
  pinMode(btn1,INPUT_PULLUP);
  pinMode(btn2,INPUT_PULLUP);
  pinMode(btn3,INPUT_PULLUP);
  pinMode(btn4,INPUT_PULLUP);
  pinMode(pairButton,INPUT_PULLUP);

  pinMode(leftBtn,INPUT_PULLUP);
  pinMode(rightBtn,INPUT_PULLUP);
  pinMode(upBtn,INPUT_PULLUP);
  pinMode(downBtn,INPUT_PULLUP);
  //pairbuttonState = true;
  //radio.setPALevel(RF24_PA_LOW); //Default Max Power
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);
  delay(200);
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);

  Serial.println("Commands available:");
  Serial.println("| p - Pair | ");
}

void loop() {  

  if (Serial.available()) {
    String inByte = Serial.readString();
    Serial.println(inByte);

    if (inByte == "p"){
      Serial.println("Pairing");
      pairData.paired = false;
      pairNow();
    }
  }

  getController();
  if (pairbuttonState)
  {
    pairData.paired = false;
    Serial.println("Pair");
    pairNow();
  }
  
  if (!pairData.paired)
  { 
    // Serial.println("Not Paired");
  }else {
    // Serial.println("Paired");
    sendData();  
  }
  digitalWrite(buzzerPin, LOW);
  
}