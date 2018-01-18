//per ara execució realitzant: 
//rostopic pub servo1 std_msgs/UInt16  <angle1> & rostopic pub servo2 std_msgs/UInt16  <angle2> & rostopic pub servo3 std_msgs/UInt16  <angle3>
//part del publisher ANGLE comentada

#if defined (ARDUINO) && (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

//std_msgs::UInt16 angle;

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
Servo servo3;

void servo1_cb(const std_msgs::UInt16& cmd_msg)
{
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void servo2_cb(const std_msgs::UInt16& cmd_msg)
{
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void servo3_cb(const std_msgs::UInt16& cmd_msg)
{
  servo3.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

//void updateAngle()
//{
//  angle.data = analogRead();
//}

ros::Subscriber<std_msgs::UInt16> sub1("servo1", &servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", &servo2_cb);
ros::Subscriber<std_msgs::UInt16> sub3("servo3", &servo3_cb);
//ros::Publisher pub_angle("angulos",&angle);

void setup(){
 
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  //nh.advertise(pub_angle);
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10); //attach it to pin 10
  servo3.attach(11); //attach it to pin 11

   pinMode(13, OUTPUT);
}

void loop(){
  //updateAngle();
  //pub_angle.publish(&angle);
  nh.spinOnce();
  delay(20);
}
