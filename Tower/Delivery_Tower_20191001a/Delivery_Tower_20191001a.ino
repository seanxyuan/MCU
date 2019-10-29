//include ros packages
#include <ros.h>
#include <Wire.h>
#include <Arduino.h>
//subscribe
#include <ros_essentials_cpp/MASTER.h>
#include <ros_essentials_cpp/CART_MCU_O.h>
//publish
#include <ros_essentials_cpp/TOWER_MCU_O.h>

//Setup Roboclaw
#include <SoftwareSerial.h>
#include "RoboClaw.h"
SoftwareSerial serial(10,11);
RoboClaw roboclaw(&serial,10000);
#define address_tower 0x84

//define limitswitch pins
int limitS2Pin = 8;
int limitS3Pin = 7;
bool limitS2 = false;
bool limitS3 = false;

//aquire input data
bool eStop = true;
int RoboClaw5PWM = 0;
bool limitS4 = false;
int ultraSonic4 = 0;

//assistant data
bool initial = true;
int slowSpeed = 30;
long dumpTimer;
long waitTimer;
int motionMode = 1;

//ultraSonic distance
int ultraSonic2Pin = A1;
int ultraSonic3Pin = A0;
int ultraSonic2 = 0;
int ultraSonic3 = 0;

//initialize ROS
ros::NodeHandle nh;
ros_essentials_cpp::MASTER master;  //sub
ros_essentials_cpp::CART_MCU_O cart_mcu_o;  //sub
ros_essentials_cpp::TOWER_MCU_O tower_mcu_o;  //pub
ros::Publisher tower_MCU_output("Tower_Output_Topic", &tower_mcu_o);


//arduino subscribe message input
void messageMASTER( const ros_essentials_cpp::MASTER &master_msg){
  eStop = master_msg.eStop;
  //tower2Cart = master_msg.tower2Cart;
  //cart2Tower = master_msg.cart2Tower;
  RoboClaw5PWM = master_msg.RoboClaw5PWM;
}

//arduino subscribe message input
void messageCART_MCU_O( const ros_essentials_cpp::CART_MCU_O &cart_MCU_o_msg){
  limitS4 = cart_MCU_o_msg.limitSwitch4;
  ultraSonic4 = cart_MCU_o_msg.ultraSonic4;
  
}

void update_tower(){
  ultraSonic2 = map(analogRead(ultraSonic2Pin), 0, 1023, 0, 100);
  ultraSonic3 = map(analogRead(ultraSonic3Pin), 0, 1023, 0, 100);
  limitS2 = digitalRead(limitS2Pin);
  limitS3 = digitalRead(limitS3Pin);
  tower_mcu_o.limitSwitch2 = limitS2;
  tower_mcu_o.limitSwitch3 = limitS3;
  tower_mcu_o.ultraSonic2 = ultraSonic2;
  tower_mcu_o.ultraSonic3 = ultraSonic3;
}

//initialize subscriber
ros::Subscriber<ros_essentials_cpp::MASTER> master_input("Master_Topic", &messageMASTER);
ros::Subscriber<ros_essentials_cpp::CART_MCU_O> cart_MCU_input("Cart_Output_Topic", &messageCART_MCU_O);

//system setup
void setup()
{
  pinMode(limitS2Pin, INPUT);
  pinMode(limitS3Pin, INPUT);
  nh.subscribe(master_input);
  nh.subscribe(cart_MCU_input);
  nh.initNode();
  nh.advertise(tower_MCU_output);
  roboclaw.begin(38400);

}

void loop()
{
  update_tower();
  tower_MCU_output.publish( &tower_mcu_o );
  nh.spinOnce();
  
  if (eStop == true){
    roboclaw.ForwardM1(address_tower,0);
    motionMode = 1; 
    initial = true;
  }
  else if (motionMode == 1){
    if (initial == true){
      roboclaw.BackwardM1(address_tower,slowSpeed); 
    }
    else if (initial == false){
      roboclaw.BackwardM1(address_tower,RoboClaw5PWM); 
    }
    if (ultraSonic4 < 90){
      motionMode = 2;
      initial = false;
     }
  }
  else if (motionMode == 2){
    roboclaw.BackwardM1(address_tower,slowSpeed); 
    if (limitS4 == true){
      motionMode = 3;
      dumpTimer = millis() + 500;
    }
     
  }
  else if (motionMode == 3){
    roboclaw.BackwardM1(address_tower,0);
    if (millis() >= dumpTimer){
      motionMode = 4;
    }
     
  }
  else if (motionMode == 4){
    roboclaw.ForwardM1(address_tower,RoboClaw5PWM);
    if (ultraSonic3 < 90){
      motionMode = 5;
    }
     
  }
  else if (motionMode == 5){
    roboclaw.ForwardM1(address_tower,slowSpeed);
    if (limitS3 == true){
      motionMode = 6;
    }
     
  }
  else if (motionMode == 6){
    roboclaw.ForwardM1(address_tower,0);
    if (limitS2 == true){
      waitTimer = millis() + 500;
      motionMode = 7;
    }
  }
  else if (motionMode == 7){
    roboclaw.BackwardM1(address_tower,0);
    if (millis() >= waitTimer){
      motionMode = 8;
    }
     
  }
  else if (motionMode == 8){
    roboclaw.BackwardM1(address_tower,slowSpeed);
    if (ultraSonic3 < 90){
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
    roboclaw.BackwardM2(address_tower,RoboClaw5PWM);
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
