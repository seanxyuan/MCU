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

//sub delivery information
limitSwitch1 = false;

//initialize ROS
//subscribe to master, wetting_input, 
ros::NodeHandle nh;
ros_essentials_cpp::MASTER master;  //sub
ros_essentials_cpp::WETTING_I wetting_i;  //sub
ros_essentials_cpp::DELIVERY_O delivery_o;  //sub
ros_essentials_cpp::WETTING_O wetting_o;  //pub
ros::Publisher wetting_output("Wetting_Output_Topic", &wetting_o);

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

//arduino subscribe delivery message input
void messageDELIVERY_O( const ros_essentials_cpp::DELIVERY_O &delivery_o_msg){
  limitSwitch1 = delivery_o_msg.limitSwitch1;
}

//publish wetting module status
void update_wetting(){
  levelBreaker = digitalRead();
  levelReservoir = digitalRead();
  digitalWrite(,HIGH);
  digitalWrite(,HIGH);
  digitalWrite(,HIGH);
}

void allOff(){
  roboclaw.ForwardM1(address0,0);
  roboclaw.SpeedM1(address,0);
  roboclaw.ForwardM1(address1,50);
  digitalWrite(,LOW);
  digitalWrite(,LOW);
  digitalWrite(,LOW);
}

void allOn(){
  roboclaw.ForwardM1(address0,50);
  roboclaw.SpeedM1(address,doserSpeed);
  roboclaw.ForwardM1(address1,50);
}

void breakerOn(){
  if (breaker == true){
    roboclaw.ForwardM1(address0,50);
  }
  else{
    roboclaw.ForwardM1(address0,0);
  }
}

void doserOn(){
  if (doser == true){
    roboclaw.SpeedM1(address,doserSpeed);
  }
  else{
    roboclaw.SpeedM1(address,0);
  }
  
}

void agitatorOn(){
  if(agitator == true){
    roboclaw.ForwardM1(address1,50);
  }
  else{
    roboclaw.ForwardM1(address1,50);
  }
}

void waterOn(){
  if(water == true){
    digitalWrite(,HIGH);
  }
  else{
    digitalWrite(,LOW);
  }
}

void mixerOn(){
  if(mixer == true){
    digitalWrite(,HIGH);
  }
  else{
    digitalWrite(,LOW);
  }
  
}

void reservoirOn(){
  if (reservoir == true){
    digitalWrite(,HIGH);
  }
  else{
    digitalWrite(,LOW);
  }
}

//initialize subscriber
ros::Subscriber<ros_essentials_cpp::MASTER> master_input("Master_Topic", &messageMASTER);
ros::Subscriber<ros_essentials_cpp::WETTING_I> wetting_input("Wetting_Input_Topic", &messageWETTING_I);
ros::Subscriber<ros_essentials_cpp::DELIVERY_O> delivery_output("Delivery_Output_Topic", &messageDELIVERY_O);

//system setup
void setup()
{
  pinMode(limitS1Pin, INPUT);
  nh.subscribe(master_input);
  nh.subscribe(wetting_input);
  nh.subscribe(delivery_output);
  nh.initNode();
  nh.advertise(wetting_output);
  roboclaw.begin(38400);
  roboclaw.SetM1VelocityPID(address,doserD,doserP,doserI,doserQPPS);
}

void loop()
{
  update_wetting();
  wetting_output.publish( &wetting_o);
  nh.spinOnce();
  if (eStop == true || wetOn == false){
    allOff();    
  }
  else{
    if(wetAllOn == true){
      allOn();
    }
    else if(wetOn == true){
      breakerOn();
      doserOn();
      agitatorOn();
      waterOn();
      mixerOn();
      reservoirOn();
    }
  }
  
  

  delay(50);  //refresh every 50 ms
}
