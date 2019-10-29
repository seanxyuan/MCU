//include ros packages
#include <ros.h>
#include <Wire.h>
#include <Arduino.h>
//subscribe
#include <ros_essentials_cpp/MASTER.h>
#include <ros_essentials_cpp/TOWER_MCU_O.h>

//Setup Roboclaw
#include <SoftwareSerial.h>
#include "RoboClaw.h"
SoftwareSerial serial(10,11);
RoboClaw roboclaw(&serial,10000);
#define address_silo 0x80

//define limitswitch pins
int limitS1Pin = 8;
bool limitS1 = false;

//aquire input data
bool eStop = true;
int RoboClaw6PWM = 0;
bool limitS2 = false;
int ultraSonic2;
bool limitS3 = false;


bool initial = true;
int slowSpeed = 50;
long dumpTimer;
long waitTimer;
int motionMode = 1;

//ultraSonic distance
int ultraSonic1Pin = A0;
int ultraSonic1 = 0;

//initialize ROS
ros::NodeHandle nh;
ros_essentials_cpp::MASTER master;  //sub
ros_essentials_cpp::TOWER_MCU_O silo_mcu_o;  //sub


//arduino subscribe message input
void messageMASTER( const ros_essentials_cpp::MASTER &master_msg){
  eStop = master_msg.eStop;
  RoboClaw6PWM = master_msg.RoboClaw6PWM;
}

//arduino subscribe message input
void messageTOWER_MCU_O( const ros_essentials_cpp::TOWER_MCU_O &tower_MCU_o_msg){
  limitS2 = tower_MCU_o_msg.limitSwitch2;
  limitS3 = tower_MCU_o_msg.limitSwitch3;
  ultraSonic2 = tower_MCU_o_msg.ultraSonic2;
  
}

void update_silo(){
  ultraSonic1 = map(analogRead(ultraSonic1Pin), 0, 1023, 0, 100);
  limitS1 = digitalRead(limitS1Pin);
}

//initialize subscriber
ros::Subscriber<ros_essentials_cpp::MASTER> master_input("Master_Topic", &messageMASTER);
ros::Subscriber<ros_essentials_cpp::TOWER_MCU_O> tower_MCU_input("Tower_Output_Topic", &messageTOWER_MCU_O);

//system setup
void setup()
{
  pinMode(limitS1Pin, INPUT);
  nh.subscribe(master_input);
  nh.subscribe(tower_MCU_input);
  nh.initNode();
  roboclaw.begin(38400);

}

void loop()
{
  update_silo();
  nh.spinOnce();
  
  if (eStop == true){
    roboclaw.ForwardM1(address_silo,0);
    motionMode = 1; 
    initial = true;
  }
  else if (motionMode == 1){
    if (initial == true){
      roboclaw.BackwardM1(address_silo,slowSpeed); 
    }
    else if (initial == false){
      roboclaw.BackwardM1(address_silo,RoboClaw6PWM); 
    }
    if (ultraSonic2 < 90){
      motionMode = 2;
      initial = false;
     }
  }
  else if (motionMode == 2){
    roboclaw.BackwardM1(address_silo,0); 
    if (limitS3 == true){
      motionMode = 3;
    }
     
  }
  else if (motionMode == 3){
    roboclaw.BackwardM1(address_silo,slowSpeed);
    if (limitS2 == true){
      motionMode = 4;
      dumpTimer = millis() + 10000;
    }
     
  }
  else if (motionMode == 4){
    roboclaw.BackwardM1(address_silo,0);
    if (millis() >= dumpTimer){
      motionMode = 5;
    }
     
  }
  else if (motionMode == 5){
    roboclaw.ForwardM1(address_silo,slowSpeed);
    if (ultraSonic2 < 90){
      motionMode = 6;
    }
     
  }
  else if (motionMode == 6){
    roboclaw.ForwardM1(address_silo,60);
    if (limitS1 == true){
      delay(10);
      if (limitS1 == true){
        motionMode = 8;
        dumpTimer = millis() + 15000;
        }   
    }
  }
 
  else if (motionMode == 8){
    roboclaw.ForwardM1(address_silo,0);
    if (millis() > dumpTimer){
      motionMode = 1;
    }
     
  }
  

  delay(50);  //refresh every 50 ms
}
/*if (cart2Tower == true && ultraSonic3 <= 90){
    cart2TowerSlow = true;
  }
  if (cart2TowerSlow == true && limitS3 == true){
    cart2TowerSlow = false;
  }
  
 
  if (eStop == true || (cart2Tower == true && tower2Cart ==true)){
    cur = 0;
    roboclaw.ForwardM2(address_tower,0); 
    delay(1);
    roboclaw.BackwardM2(address_tower,0);
  }
  else if (tower2Cart == true && lS4 == false && uS4 > 90){
    cur = 1;
    roboclaw.BackwardM2(address_tower,RoboClaw6PWM);
    delay(1);
    roboclaw.BackwardM2(address_tower,RoboClaw5PWM);
    
  }
  else if (tower2Cart == true && lS4 == false && uS4 <= 90){
    cur = 2; 
    roboclaw.BackwardM2(address_tower,25); 
    delay(1);
    roboclaw.BackwardM2(address_tower,25); 
  }
  else if (cart2Tower == true && limitS3 == false && ultraSonic3 > 90 && cart2TowerSlow == false){
    cur = 3;
    roboclaw.ForwardM2(address_tower,RoboClaw5PWM); 
    delay(1);
    roboclaw.ForwardM2(address_tower,RoboClaw5PWM);
  }
  else if (cart2Tower == true && limitS3 == false && (ultraSonic3 <= 90 || cart2TowerSlow == true)){
    cur = 4;
    roboclaw.ForwardM2(address_tower,35); 
    delay(1);
    roboclaw.ForwardM2(address_tower,35); 
  }
  else{
    cur = 0;
    roboclaw.ForwardM2(address_tower,0); 
    delay(1);
    roboclaw.ForwardM2(address_tower,0); 
  }*/
//include ros packages
/*#include <ros.h>
#include <Wire.h>
#include <Arduino.h>

//Setup Roboclaw
#include <SoftwareSerial.h>
#include "RoboClaw.h"
SoftwareSerial serial(10,11);
RoboClaw roboclaw(&serial,10000);
#define address1 0x80
#define address2 0x81
#define address3 0x82
#define address4 0x83

//setup relay pins
int relay1 = 53;
int relay2 = 51;
int relay3 = 49;

//system setup
void setup()
{
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  roboclaw.begin(38400);
}

void loop()
{
  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);
  roboclaw.ForwardM1(address1,50); 
  roboclaw.ForwardM2(address1,50); 
  roboclaw.ForwardM1(address2,50); 
  roboclaw.ForwardM2(address2,50); 
  roboclaw.ForwardM1(address3,50); 
  roboclaw.ForwardM2(address3,50); 
  roboclaw.ForwardM1(address4,50); 
  roboclaw.ForwardM2(address4,50); 
  delay(5000);
  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);
  roboclaw.ForwardM1(address1,100); 
  roboclaw.ForwardM2(address1,100); 
  roboclaw.ForwardM1(address2,100); 
  roboclaw.ForwardM2(address2,100); 
  roboclaw.ForwardM1(address3,100); 
  roboclaw.ForwardM2(address3,100); 
  roboclaw.ForwardM1(address4,100); 
  roboclaw.ForwardM2(address4,100); 
  delay(5000);
  digitalWrite(relay1,LOW);
  digitalWrite(relay2,LOW);
  digitalWrite(relay3,LOW);
  roboclaw.ForwardM1(address1,0); 
  roboclaw.ForwardM2(address1,0); 
  roboclaw.ForwardM1(address2,0); 
  roboclaw.ForwardM2(address2,0); 
  roboclaw.ForwardM1(address3,0); 
  roboclaw.ForwardM2(address3,0); 
  roboclaw.ForwardM1(address4,0); 
  roboclaw.ForwardM2(address4,0);
  delay(10000);
}
*/
