//include ros packages
#include <ros.h>
#include <Wire.h>
#include <Arduino.h>
//subscribe
#include <ros_essentials_cpp/MASTER.h>
#include <ros_essentials_cpp/Wetting_I.h>
#include <ros_essentials_cpp/Wetting_O.h>
#include <ros_essentials_cpp/Delivery_O.h>

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
int relay1 = 38; int relay2 = 40; int relay3 = 42; int relay4 = 44;
int relay5 = 46; int relay6 = 48; int relay7 = 50; int relay8 = 52;
int sensor1 = 28; int sensor2 = 29; int sensor3 = 30; int sensor4 = 31;
int sensor5 = 32; int sensor6 = 33; int sensor7 = 34; int sensor8 = 35;
int servo1 = 24; int servo2 = 25; int servo3 = 26; int servo4 = 27;

//sub master information
bool eStop = true;
bool siloOn = false;
bool siloAllOn = false;
bool wetOn = false;
bool wetAllOn = false;

//sub wetting information
bool breaker = false;
int breakerPWM = 0;
bool doser = false;
int doserSpeed = 0;
int doserP = 0;
int doserI = 0;
int doserD = 0;
int doserQPPS = 200000;
bool agitator = false;
int agitatorPWM = 0;
bool water = false;
bool mixer = false;
bool reservoir = false;
int reservoirSpeed = 0;
//pub wetting information
bool levelBreaker;
int breakerCurrent;
int doserRSpeed;
int doserCurrent;
int agitatorCurrent;
int waterRSpeed;
int mixerRSpeed;
bool levelReservoir;


//sub silo information
bool siloBreaker = false;
bool siloDoser = false;
//pub silo information
bool siloSensor1;
bool siloSensor2;

/*sub delivery information
limitSwitch1 = false;
*/

//initialize ROS
//subscribe to master, wetting_input, 
ros::NodeHandle nh;
ros_essentials_cpp::MASTER master;  //sub
ros_essentials_cpp::SILO_I silo_i;  //sub
ros_essentials_cpp::SILO_O silo_o;  //pub
ros_essentials_cpp::WETTING_I wetting_i;  //sub
ros_essentials_cpp::WETTING_O wetting_o;  //pub
//ros_essentials_cpp::DELIVERY_O delivery_o;  //sub
ros::Publisher wetting_output("Wetting_Output_Topic", &wetting_o);
ros::Publisher silo_output("Silo_Output_Topic", &silo_o);

//arduino subscribe master message input
void messageMASTER( const ros_essentials_cpp::MASTER &master_msg){
  eStop = master_msg.eStop;
  siloOn = master_msg.siloOn;
  siloAllOn = master_msg.siloAllOn;
  wetOn = master_msg.wetOn;
  wetAllOn = master_msg.wetAllOn;
}

//arduino subscribe wetting message input
void messageWETTING_I( const ros_essentials_cpp::Wetting_I &wetting_i_msg){
  breaker = wetting_i_msg.breaker;
  breakerPWM = wetting_i_msg.breakerPWM;
  doser = wetting_i_msg.doser;
  doserSpeed = wetting_i_msg.doserSpeed;
  doserP = wetting_i_msg.doserP;
  doserI = wetting_i_msg.doserI;
  doserD = wetting_i_msg.doserD;
  agitator = wetting_i_msg.agitator;
  agitatorPWM = wetting_i_msg.agitatorPWM;
  water = wetting_i_msg.water;
  waterSpeed = wetting_i_msg.waterSpeed;
  mixer = wetting_i_msg.mixer;
  mixerSpeed = wetting_i_msg.mixerSpeed;
  reservoir = wetting_i_msg.reservoir;
  reservoirSpeed = wetting_i_msg.reservoirSpeed;  
}

//arduino subscribe silo message input
void messageSILO_I( const ros_essentials_cpp::Silo_I &silo_i_msg){
  siloBreaker = silo_i_msg.siloBreaker;
  siloDoser = silo_i_msg.siloDoser; 
}

/*arduino subscribe delivery message input
void messageDELIVERY_O( const ros_essentials_cpp::DELIVERY_O &delivery_o_msg){
  limitSwitch1 = delivery_o_msg.limitSwitch1;
}
*/

//publish silo status
void update_silo(){
  silo_o.siloSensor1 = digitalRead(sensor2);
  silo_o.siloSensor2 = digitalRead(sensor3);
}

//publish wetting module status
void update_wetting(){
  wetting_o.levelBreaker = digitalRead(sensor1);
  wetting_o.breakerCurrent = random(300);
  wetting_o.doserRSpeed = random(300);
  wetting_o.doserCurrent = random(300);
  wetting_o.agitatorCurrent = random(300);
  wetting_o.waterRSpeed = random(300);
  wetting_o.mixerRSpeed = random(300);
  wetting_o.levelReservoir = digitalRead(sensor4);
}

//start the breaker if doser, water, mixer and reservoir are ON and reservoir is not FULL
void breakerOn(){
  if (breaker == true doser == true && water == true && 
  mixer == true && reservoir == true && digitalRead(sensor4) == false){
    roboclaw.ForwardM2(address0,50);
  }
  else{
    roboclaw.ForwardM2(address0,0);
  }
}

//start the doser if water, mixer and reservoir are ON and reservoir is not FULL 
void doserOn(){
  if (doser == true && water == true && mixer == true && 
  reservoir == true && digitalRead(sensor4) == false){
    roboclaw.SpeedM1(address1,doserSpeed);
  }
  else{
    roboclaw.SpeedM1(address1,0);
  }
  
}

//start the agitator if mixer is ON 
void agitatorOn(){
  if(agitator == true && mixer == true){
    roboclaw.ForwardM1(address0,50);
  }
  else{
    roboclaw.ForwardM1(address0,0);
  }
}

//start the water if mixer and reservoir are ON and reservoir is not FULL 
void waterOn(){
  if (wetAllOn == true && digitalRead(sensor4) == false){
    
  }
  else if(water == true && mixer == true && reservoir == true && digitalRead(sensor4) == false){
    digitalWrite(relay3, HIGH);
  }
  else{
    digitalWrite(relay3, LOW);
  }
}

//start the mixer
void mixerOn(){
  if (wetAllOn == true && digitalRead(sensor4) == false){
    digitalWrite(relay2, HIGH);
  }
  else if (wetOn == true && mixer == true){
    digitalWrite(relay2, HIGH);
  }
  else{
    digitalWrite(relay2, LOW);
  }
  
}

//start the reservoir if reservoir is not FULL 
void reservoirOn(){
  if (wetAllOn == true && digitalRead(sensor4) == false){
    //Turn on reservoir
  }
  else if (wetOn == true && reservoir == true && digitalRead(sensor4) == false){
    //Turn on reservoir
  }
  else{
    //Turn off reservoir
  }
}

void allOff(){
  roboclaw.ForwardM1(address0,0);
  roboclaw.ForwardM2(address0,0);
  roboclaw.SpeedM1(address1,0);
  //turn off reservoir motor
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  digitalWrite(relay4, LOW);
  digitalWrite(relay5, LOW);
  digitalWrite(relay8, LOW);
}


//initialize subscriber
ros::Subscriber<ros_essentials_cpp::MASTER> master_input("Master_Topic", &messageMASTER);
ros::Subscriber<ros_essentials_cpp::WETTING_I> wetting_input("Wetting_Input_Topic", &messageWETTING_I);
ros::Subscriber<ros_essentials_cpp::SILO_I> silo_input("Silo_Input_Topic", &messageSILO_I);
//ros::Subscriber<ros_essentials_cpp::DELIVERY_O> delivery_output("Delivery_Output_Topic", &messageDELIVERY_O);

//system setup
void setup()
{
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);
  pinMode(relay5,OUTPUT);
  pinMode(relay8,OUTPUT);
  pinMode(sensor1,INPUT);
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(sensor4,INPUT);
  nh.subscribe(master_input);
  nh.subscribe(wetting_input);
  nh.subscribe(silo_input);
  nh.advertise(wetting_output);
  nh.advertise(silo_output);
  nh.initNode();
  roboclaw.begin(38400);
  roboclaw.SetM1VelocityPID(address1,doserD,doserP,doserI,doserQPPS);
}

void loop()
{
  update_wetting();
  update_silo();
  wetting_output.publish( &wetting_o);
  nh.spinOnce();
  if (eStop == true||(wetOn == false && wetAllOn == false)){
    allOff();    
  }
  else if(wetAllOn == true || wetOn == true){
    breakerOn();
    doserOn();
    agitatorOn();
    waterOn();
    mixerOn();
    reservoirOn();
  }
  
  delay(50);  //refresh every 50 ms
}
