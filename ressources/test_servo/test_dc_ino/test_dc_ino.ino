
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define TILT_INIT 1250
#define TILT_RANGE 200

#define PAN_INIT 1330
#define PAN_RANGE 600

#define FIRE_INIT 1700
#define FIRE_PUSH 1000

#define READY 0
#define RUN 1
#define FIRE 2

#define RUN_TIME 2000
#define RECOIL_TIME 2500


ros::NodeHandle  nh;
int fire_state;
unsigned long fire_time;

void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  char message[100];

  //int motor_power = (int)((cmd_msg.axes[2])*150);
  int motor_power = 0;


  //direction
  int servo_pan = (int)(PAN_INIT + PAN_RANGE * cmd_msg.axes[0]);
  int servo_tilt = (int)(TILT_INIT + TILT_RANGE * cmd_msg.axes[1]);
  

  pwm.writeMicroseconds(0, servo_pan);
  pwm.writeMicroseconds(1, servo_tilt);
    

  //sprintf(message, "pan : %d, tilt : %d\n", servo_pan, servo_tilt);
  //nh.loginfo(message);
    
  if (cmd_msg.buttons[5] == 1)
  {
    //are we firing ? 
    if (fire_state == READY)
    {
        //Motor A forward @ configured speed
      digitalWrite(12, HIGH); //Establishes forward direction of Channel A
      digitalWrite(9, LOW);   //Disengage the Brake for Channel A
      analogWrite(3, motor_power);   //Spins the motor on Channel A at full speed

      //Motor B backward @ half speed
      digitalWrite(13, LOW);  //Establishes backward direction of Channel B
      digitalWrite(8, LOW);   //Disengage the Brake for Channel B
      analogWrite(11, motor_power);    //Spins the motor on Channel B at half speed

      fire_state = RUN;
      fire_time = millis();

      sprintf(message, "running hot !");
      nh.loginfo(message);
  
    }
  }
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", joy_cb);

void setup() {

  pwm.begin();

  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin


  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.writeMicroseconds(0, TILT_INIT);
  pwm.writeMicroseconds(1, PAN_INIT);
  pwm.writeMicroseconds(2, FIRE_INIT);

  fire_state = READY;
  fire_time = millis();

  delay(10);

  nh.initNode();
  nh.subscribe(sub);

}

void fire_management()
{
  if (fire_state == RUN)
  {
    if ((millis() - fire_time) > RUN_TIME)
    {
      pwm.writeMicroseconds(2, FIRE_PUSH);
      fire_state = FIRE;
    }
  }
  else if (fire_state == FIRE)
  {
    if ((millis() - fire_time) > RECOIL_TIME)
    {
      pwm.writeMicroseconds(2, FIRE_INIT);
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
      fire_state = READY;
    }
  }
}

void loop() {
  nh.spinOnce();

  fire_management();
  //delay(10);

}
