#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;

Servo pan;
Servo pitch;
Servo recoil;

// !!motor servos are TBD!!
const byte pan_motor = 5;
const byte pitch_motor = 6;
const byte recoil_motor = 10;

const bytes motor_a = 12;
const bytes break_a = 9;
const bytes spin_a = 3;

const bytes motor_b = 13;
const bytes break_b = 8;
const bytes spin_a = 11;

const byte pan_limit_1 = 0;
const byte pan_limit_2 = 180;
const byte pan_init = 90;
const byte tilt_limit_1 = 65;
const byte tilt_limit_2 = 180;
const byte tilt_init = 105;
const byte recoil_rest = 180;    // Angle of the servo when at rest
const byte recoil_pushed = 125;  // Angle the servo need to reach to push the dart

//-----Variable related to motor timing and firing
bool is_firing =  false;
bool can_fire =  true;
bool recoiling = false;

unsigned long firing_start_time = 0;
unsigned long firing_current_time = 0;
const long firing_time = 150;

unsigned long recoil_start_time = 0;
unsigned long recoil_current_time = 0;
const long recoil_time = 2 * firing_time;

const byte motor_pin =  12;
boolean motors_ON = false;

ros::Subscriber<sensor_msgs::Joy> sub("joy", joy_cb);


void fire() 
{ //if motor byte on, turn motor on and check for time it has been on

  if (can_fire && !is_firing && motors_ON) 
  {
    
    firing_start_time = millis();
    recoil_start_time = millis();
    is_firing = true;
  }

  firing_current_time = millis();
  recoil_current_time = millis();

  if (is_firing && firing_current_time - firing_start_time < firing_time) 
  {
    recoil_motor.write(recoil_pushed);
  }
  else if (is_firing && recoil_current_time - recoil_start_time < recoil_time) 
  {
    recoil_motor.write(recoil_rest);
  }
  else if (is_firing && recoil_current_time - recoil_start_time > recoil_time) 
  {
    is_firing = false;
  }

}



void joy_cb( const sensor_msgs::Joy& cmd_msg){

  // axes are in & -1 ; 1 scale.
  int pan_angle = (int)((cmd_msg.axes[0]+1)*(pan_limit_2 - pan_limit_1)/2) + pan_limit_1;

  int tilt_angle = (int)((cmd_msg.axes[1]+1)*(tilt_limit_2 - tilt_limit_1) / 2+ tilt_limit_1);
  

  pan.write(pan_angle); //set servo angle, should be from 0-180
  pan.write(pitch_motor); //set servo angle, should be from 0-180

  //right trigger ? 
  if (cmd_msg.buttons[0] == 1)
  {
    //fire !
  }

  //left trigger ? 
  if (cmd_msg.buttons[1])
  {
    digitalWrite(break_a, LOW);   // Disengage the Brake for Channel A
    digitalWrite(break_b, LOW);   // Disengage the Brake for Channel B

    analogWrite(spin_a, 100);    //Spins the motor on Channel A at 40%
    analogWrite(spin_b, 100);    //Spins the motor on Channel A at 40%

    motors_ON = true;

  }
  else
  {
    digitalWrite(break_a, HIGH);   // Engage the Brake for Channel A
    digitalWrite(break_b, HIGH);   // Engage the Brake for Channel B

    motors_ON = false;
  }

  
}


void setup(){
    nh.initNode();
  
  nh.subscribe(sub);

  pan.attach(pan_motor); 
  pitch.attach(pitch_motor);
  recoil.attach(recoil_motor);


  //Setup Motor A
  pinMode(motor_a, OUTPUT); // Initiates Motor Channel A pin
  pinMode(break_a, OUTPUT); // Initiates Brake Channel A pin

  //Setup Motor B
  pinMode(motor_b, OUTPUT);  // Initiates Motor Channel A pin
  pinMode(break_b, OUTPUT);  // Initiates Brake Channel A pin

  // init firing motors
  digitalWrite(motor_a, HIGH); // Establishes forward direction of Channel A
  digitalWrite(motor_b, HIGH); // Establishes forward direction of Channel B

  digitalWrite(break_a, HIGH);   // Engage the Brake for Channel A
  digitalWrite(break_b, HIGH);   // Engage the Brake for Channel B

  // starting sequence
  recoil.write(recoil_rest);
  pan.write(pan_init);
  delay(1000);
  pitch.write(tilt_init);


}

void loop(){
  nh.spinOnce();
  delay(10);
}
