//include ros packages
#include <Wire.h>
#include <Arduino.h>

//Setup Roboclaw
#include <SoftwareSerial.h>
#include "RoboClaw.h"
SoftwareSerial serial(10,11);
RoboClaw roboclaw(&serial,10000);
#define address0 0x80
#define address1 0x81
#define address2 0x82
#define address3 0x83

//define arduino pins
int relay1 = 53; int relay2 = 51; int relay3 = 49; int relay4 = 47;
int relay5 = 45; int relay6 = 43; int relay7 = 41; int relay8 = 39;
int sensor1 = 24; int sensor2 = 25; int sensor3 = 26; int sensor4 = 27;
int sensor5 = 28; int sensor6 = 29; int sensor7 = 31; int sensor8 = 30;

//input master information
bool eStop = false;
bool wetOn = true;
bool wetAllOn = false;

//input wetting information
bool breaker = false;
bool doser = false;
bool agitator = false;
bool water = false;
bool mixer = false;
bool reservoir = false;

//pub wetting information
bool levelBreaker;
bool levelReservoir;


void updateSensor(){
  levelBreaker = digitalRead(sensor1);
  levelReservoir = digitalRead(sensor4);
  digitalWrite(relay8, LOW);
}

//start the breaker if doser, water, mixer and reservoir are ON and reservoir is not FULL
void breakerOn(){
  if (wetAllOn == true && breaker == true && doser == true && 
  agitator == true && mixer == true && levelReservoir == false){
    roboclaw.BackwardM1(address0,100);
  }
  else if(wetOn == true && breaker == true){
    roboclaw.BackwardM1(address0,100);
  }
  else{
    roboclaw.BackwardM1(address0,0);
  }
}

//start the doser if water, mixer and reservoir are ON and reservoir is not FULL 
void doserOn(){
  if (wetAllOn == true && doser == true && water == true &&
  mixer == true && levelReservoir == false){
    roboclaw.SpeedM1(address1,-100000);
  }
  else if (wetOn == true && doser == true){
    roboclaw.SpeedM1(address1,-100000);
  }
  else{
    roboclaw.SpeedM1(address1,0);
  }
  
}

//start the agitator if mixer is ON 
void agitatorOn(){
  if(wetAllOn == true && agitator == true){
    roboclaw.ForwardM2(address0,100);
  }
  else if (wetOn == true && agitator == true){
    roboclaw.ForwardM2(address0,100);
  }
  else{
    roboclaw.ForwardM2(address0,0);
  }
}

//start the water if mixer and reservoir are ON and reservoir is not FULL 
void waterOn(){
  if (wetAllOn == true && water == true && agitator == true && mixer == true && levelReservoir == false){
    digitalWrite(relay3, LOW);
  }
  else if (wetOn == true && water == true){
    digitalWrite(relay3, LOW);
  }
  else{
    digitalWrite(relay3, HIGH);
  }
}

//start the mixer
void mixerOn(){
  if (wetAllOn == true && mixer == true && levelReservoir == false){
    digitalWrite(relay2, LOW);
  }
  else if (wetOn == true && mixer == true){
    digitalWrite(relay2, LOW);
  }
  else{
    digitalWrite(relay2, HIGH);
  }
  
}

//start the reservoir if reservoir is not FULL 
void reservoirOn(){
  if (wetAllOn == true && reservoir == true){
    digitalWrite(relay6, LOW);
  }
  else if (wetOn == true && reservoir == true){
    digitalWrite(relay6, LOW);
  }
  else{
    digitalWrite(relay6, HIGH);
  }
}

void allOff(){
  breaker = false;
  doser = false;
  agitator = false;
  water = false;
  mixer = false;
  reservoir = false;
}

void allOn(){
  breaker = true;
  doser = true;
  agitator = true;
  water = true;
  mixer = true;
  reservoir = true;
}

void sysPrint(){ 
  Serial.print(" eStop:");Serial.print(eStop);
  Serial.print(" wetOn:");Serial.print(wetOn); 
  Serial.print(" wetAllOn:");Serial.print(wetAllOn);
  Serial.print(" breaker:");Serial.print(breaker); 
  Serial.print(" doser:");Serial.print(doser);
  Serial.print(" agitator:");Serial.print(agitator); 
  Serial.print(" water:");Serial.print(water); 
  Serial.print(" mixer:");Serial.print(mixer);
  Serial.print(" reservoir:");Serial.print(reservoir); 
  Serial.print(" levelBreaker:");Serial.print(levelBreaker);
  Serial.print(" levelReservoir:");Serial.println(levelReservoir);
}

//system setup
void setup()
{
  pinMode(relay1,OUTPUT);pinMode(relay2,OUTPUT);pinMode(relay3,OUTPUT);pinMode(relay4,OUTPUT);
  pinMode(relay5,OUTPUT);pinMode(relay6,OUTPUT);pinMode(relay7,OUTPUT);pinMode(relay8,OUTPUT);
  pinMode(sensor1,INPUT);pinMode(sensor2,INPUT);pinMode(sensor3,INPUT);pinMode(sensor4,INPUT);
  pinMode(sensor5,INPUT);pinMode(sensor6,INPUT);pinMode(sensor7,INPUT);pinMode(sensor8,INPUT);
  roboclaw.begin(38400);
  roboclaw.SetM1VelocityPID(address1, 0.1, 10, 1, 200000);
  Serial.begin(9600);
}

void loop(){
  updateSensor();
  sysPrint();
  if (wetOn == false && wetAllOn == false){
    allOff();
  }
  else if (wetAllOn == true){
    allOn();
  }
  breakerOn();
  doserOn();
  agitatorOn();
  waterOn();
  mixerOn();
  reservoirOn();
  delay(50);  //refresh every 50 ms
}
