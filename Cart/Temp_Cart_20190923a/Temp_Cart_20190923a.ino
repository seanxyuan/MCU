//include ros packages
#include <ros.h>
#include <Wire.h>
//publish
#include <ros_essentials_cpp/CART_MCU_O.h>

//define limitswitch pins
int limitS4Pin = 4;
bool limitS4 = false;

//ultraSonic distance
int ultraSonic4Pin = A4;
int ultraSonic4 = 0;

//initialize ROS
ros::NodeHandle nh;
ros_essentials_cpp::CART_MCU_O cart_mcu_o;  //pub
ros::Publisher cart_mcu_output("Cart_Output_Topic", &cart_mcu_o);

void update_cart(){
  ultraSonic4 = analogRead(ultraSonic4Pin);
  limitS4 = digitalRead(limitS4Pin);
  //cart_mcu_o.limitSwitch4 = limitS4;
  //cart_mcu_o.ultraSonic4 = ultraSonic4;
  cart_mcu_o.limitSwitch4 = -1 + (int)random(2) * 2;
  //Serial.println(cart_mcu_o.limitSwitch4);
  cart_mcu_o.ultraSonic4 = random(0,100);
  //Serial.println(cart_mcu_o.ultraSonic4);
}

//system setup
void setup()
{
  pinMode(limitS4Pin, INPUT);
  nh.initNode();
  nh.advertise(cart_mcu_output);
  //Serial.begin(9600);
}

void loop()
{
  update_cart();
  cart_mcu_output.publish( &cart_mcu_o );
  nh.spinOnce();
  delay(50);  //refresh every 50 ms
  
}
