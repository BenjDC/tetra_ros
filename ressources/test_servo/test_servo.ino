#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const sensor_msgs::Joy& cmd_msg){

  int angle = (int)((cmd_msg.axes[0]+1)*90);
  //int  = map(cmd_msg.axes[0],-1, 1, 0, 180);
  servo.write(angle); //set servo angle, should be from 0-180
  
}


ros::Subscriber<sensor_msgs::Joy> sub("joy", servo_cb);

void setup(){
    nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
