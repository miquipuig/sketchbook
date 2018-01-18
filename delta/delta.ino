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
#include <std_msgs/Bool.h>

//std_msgs::UInt16 angle;


ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
Servo servo3;
std_msgs::Bool pushed_msg;

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


const int button_pin = 7;
const int led_pin = 13;
bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;
//void updateAngle()
//{
//  angle.data = analogRead();
//}

ros::Subscriber<std_msgs::UInt16> sub1("servo1", &servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", &servo2_cb);
ros::Subscriber<std_msgs::UInt16> sub3("servo3", &servo3_cb);
ros::Publisher pub_button("pushed", &pushed_msg);

void setup(){
 
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(pub_button);
  
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10); //attach it to pin 10
  servo3.attach(11); //attach it to pin 11

  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);

  digitalWrite(button_pin, HIGH);
  last_reading = ! digitalRead(button_pin);

}

void loop(){
  //updateAngle();
  //pub_angle.publish(&angle);
  nh.spinOnce();
 
  bool reading = ! digitalRead(button_pin);
    if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
    if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
   delay(20);
}
