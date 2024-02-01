#include <ros.h>
#include <std_msgs/String.h>
// left motor control setting
const byte LEFT1 = 9;
const byte LEFT2 = 8;
const byte LEFT_PWM = 10;

// right motor control setting
const byte RIGHT1 = 7;
const byte RIGHT2 = 6;
const byte RIGHT_PWM = 5;

byte MAX_SPEED = 255;
byte MIN_SPEED = 66;
byte RIGHTSpeed = 90;
byte LEFTSpeed = 70;

void forward() {
  digitalWrite(LEFT1, HIGH);
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM, LEFTSpeed);

  // right wheel
  digitalWrite(RIGHT1, HIGH);
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM, RIGHTSpeed);
}
void rotaleft() {
   
  // left wheel
  digitalWrite(LEFT1, LOW);
  digitalWrite(LEFT2, HIGH);
  analogWrite(LEFT_PWM, LEFTSpeed);

  // right wheel
  digitalWrite(RIGHT1, HIGH);
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM, MIN_SPEED);
}
void rotaright() {
   
  // left wheel
  digitalWrite(LEFT1, HIGH);
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM, MIN_SPEED);

  // right wheel
  digitalWrite(RIGHT1, LOW);
  digitalWrite(RIGHT2, HIGH);
  analogWrite(RIGHT_PWM, RIGHTSpeed);
}
// backward
void backward() {
  // left wheel
  digitalWrite(LEFT1, LOW);
  digitalWrite(LEFT2, HIGH);
  analogWrite(LEFT_PWM, LEFTSpeed);

  // right wheel
  digitalWrite(RIGHT1, LOW);
  digitalWrite(RIGHT2, HIGH);
  analogWrite(RIGHT_PWM, RIGHTSpeed);
}
void turnleft() {
  
  // left wheel
  analogWrite(LEFT_PWM, 0);

  // right wheel
  digitalWrite(RIGHT1, HIGH);
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM, RIGHTSpeed);
  
}
void turnright() {
  
  // left wheel
  digitalWrite(LEFT1, HIGH);
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM, LEFTSpeed);

  // right wheel
  analogWrite(RIGHT_PWM, 0);
}
void stopmotor() {
  
  // left wheel
  analogWrite(LEFT_PWM, 0);

  // right wheel
  analogWrite(RIGHT_PWM, 0);
}


ros::NodeHandle nh;

std_msgs::String arduino_msg;
ros::Publisher feedback("feedback", &arduino_msg);

void pi_sent_cmd(const std_msgs::String &rasberrypi_msg) {
  String data = String(rasberrypi_msg.data);
  if( data=="F"){
    forward();
  }else if( data=="B"){
    backward();
  }else if( data=="L"){
    turnleft();
  }else if( data=="R"){
    turnright();
  }else if( data=="S"){
    stopmotor();
  }
  arduino_msg.data = rasberrypi_msg.data;
  feedback.publish( &arduino_msg );
}
ros::Subscriber<std_msgs::String> sub("picmd", pi_sent_cmd);

void setup() {
  // Set up the motor pins as output
  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  
  // Set up the serial communication
  nh.initNode();  
  nh.subscribe(sub);  
  nh.advertise(feedback);
  Serial.begin(9600);
}

void loop() {  
  
  nh.spinOnce();
}
