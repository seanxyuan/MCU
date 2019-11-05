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

//initialize ROS
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
}

void allOff(){
  
}

void allOn(){
  
}

void breakerOn(){
  
}

void doserOn(){
  
}

void agitatorOn(){
  
}

void waterOn(){
  
}

void mixerOn(){
  
}

void reservoirOn(){
  
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
      doserOn(doser);
      agitatorOn(agitator);
      waterOn(water);
      mixerOn(mixer);
      reservoirOn(reservoir);
    }
  }
  
  

  delay(50);  //refresh every 50 ms
}
