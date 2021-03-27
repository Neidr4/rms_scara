#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

/*--------------------------------------*/
/*---------------- Pins ----------------*/
/*--------------------------------------*/

const int driver_RST = 28;

const int joint1_EN = 22;
const int joint1_DIR = 23;
const int joint1_STEP = 2;

const int joint2_EN = 24;
const int joint2_DIR = 25;
const int joint2_STEP = 3;

const int joint3_EN = 26;
const int joint3_DIR = 27;
const int joint3_STEP = 4;

const int gripper_SIGNAL = 5;

const int joint1_STOP = 18;
const int joint2_STOP = 19;
const int joint3_STOP = 20;

/*
const int joint1_STOP = 29;
const int joint2_STOP = 30;
const int joint3_STOP = 31;
*/

const int LED1 = 32;
const int LED2 = 33;

/*--------------------------------------*/
/*---------- Global variables ----------*/
/*--------------------------------------*/

volatile char buffer_chatter[20] = "init done";
char hello[14] = "hello world!";
char interrupt_01[14] = "endstop01 trg";
char interrupt_23[14] = "endstop23 trg";
char interrupt_45[14] = "endstop45 trg";

bool bonsoir = LOW;

/*--------------------------------------*/
/*-------------- Callbacks -------------*/
/*--------------------------------------*/

//Callback for joint1 subscriber
void callback_joint1(const std_msgs::Int32& msg) {
  //Changing speed thanks to the first 3 digits
  byte pwm = msg.data % 1000;
  analogWrite(joint1_STEP, pwm);

  //Changing direction thanks to the 4th digit
  if(msg.data%10000 >= 1000){
    //Negative
    digitalWrite(joint1_DIR, LOW);
  }
  else {
    //Forward
    digitalWrite(joint1_DIR, HIGH);
  }

  //Time to wait in ms starting from 5th digit
  unsigned short wait = floor(msg.data / 10000);
  
  /*
  //Send data "easily"
  char buffer_string[33];
  itoa(wait, buffer_string, 10);
  strcpy(buffer_chatter, buffer_string);
  */
  
  //Activating joint_1
  digitalWrite(joint1_EN, LOW);
  delay(wait);
  digitalWrite(joint1_EN, HIGH);
}

//Callback for joint2 subscriber
void callback_joint2(const std_msgs::Int32& msg) {
  //Changing speed thanks to the first 3 digits
  byte pwm = msg.data % 1000;
  analogWrite(joint2_STEP, pwm);

  //Changing direction thanks to the 4th digit
  if(msg.data%10000 >= 1000){
    //Negative
    digitalWrite(joint2_DIR, LOW);
  }
  else {
    //Forward
    digitalWrite(joint2_DIR, HIGH);
  }
  
  //Time to wait in ms starting from 5th digit
  unsigned short wait = floor(msg.data / 10000);

  //Activating joint_2
  digitalWrite(joint2_EN, LOW);
  delay(wait);
  digitalWrite(joint2_EN, HIGH);
}

//Callback for joint3 subscriber
void callback_joint3(const std_msgs::Int32& msg) {
  //Changing speed thanks to the first 3 digits
  byte pwm = msg.data % 1000;
  analogWrite(joint3_STEP, pwm);

  //Changing direction thanks to the 4th digit
  if(msg.data%10000 >= 1000){
    //Negative
    digitalWrite(joint3_DIR, LOW);
  }
  else {
    //Forward
    digitalWrite(joint3_DIR, HIGH);
  }  

  //Time to wait in ms starting from 5th digit
  unsigned short wait = floor(msg.data / 10000);
  
  //Activating joint_3
  digitalWrite(joint3_EN, LOW);
  delay(wait);
  digitalWrite(joint3_EN, HIGH);
}

/*--------------------------------------*/
/*------------- Interrupts -------------*/
/*--------------------------------------*/

void catch_interrupt_01(){
  digitalWrite(joint1_EN, HIGH);
  digitalWrite(LED2, HIGH);
  strcpy(buffer_chatter, interrupt_01);
}

void catch_interrupt_23(){
  digitalWrite(joint2_EN, HIGH);
  digitalWrite(LED2, HIGH);
  strcpy(buffer_chatter, interrupt_23);
}

void catch_interrupt_45(){
  digitalWrite(joint3_EN, HIGH);
  digitalWrite(LED2, HIGH);
  strcpy(buffer_chatter, interrupt_45);
}

/*--------------------------------------*/
/*----- Publishers and Subscribers -----*/
/*--------------------------------------*/

//Publishers
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

//Subscribers
ros::Subscriber<std_msgs::Int32> sub_joint1("/rms/joint1_arduino", callback_joint1 );
ros::Subscriber<std_msgs::Int32> sub_joint2("/rms/joint2_arduino", callback_joint2 );
ros::Subscriber<std_msgs::Int32> sub_joint3("/rms/joint3_arduino", callback_joint3 );


/*--------------------------------------*/
/*----------- Initialisation -----------*/
/*--------------------------------------*/

void zeros(){
  int init_pwm = 50;
  int basic_wait = 10;

  //Desabling all driver for safety
  digitalWrite(joint1_EN, HIGH);
  digitalWrite(joint2_EN, HIGH);
  digitalWrite(joint3_EN, HIGH);
  
  //joint1 init
  digitalWrite(joint1_DIR, LOW);
  analogWrite(joint1_STEP, init_pwm);
  digitalWrite(joint1_EN, LOW);
  while(digitalRead(joint1_STOP));
  digitalWrite(joint1_DIR, LOW);
  delay(basic_wait);
  digitalWrite(joint1_EN, HIGH);

  //joint2 init
  digitalWrite(joint2_DIR, LOW);
  analogWrite(joint2_STEP, init_pwm);
  digitalWrite(joint2_EN, LOW);
  while(digitalRead(joint2_STOP));
  digitalWrite(joint2_DIR, LOW);
  delay(basic_wait);
  digitalWrite(joint2_EN, HIGH);

  //joint3 init
  digitalWrite(joint3_DIR, LOW);
  analogWrite(joint3_STEP, init_pwm);
  digitalWrite(joint3_EN, LOW);
  while(digitalRead(joint3_STOP));
  digitalWrite(joint3_DIR, LOW);
  delay(basic_wait);
  digitalWrite(joint3_EN, HIGH);
  
}

void simple_test_init(){
  digitalWrite(joint1_DIR, LOW);
  analogWrite(joint1_STEP, 200);
  digitalWrite(joint1_EN, LOW);
}

void simple_test_loop(){
  //str_msg.data = hello;
  digitalWrite(joint1_EN, LOW);
  delay(2000);
  digitalWrite(joint1_DIR, HIGH);
  delay(2000);
  digitalWrite(joint1_DIR, LOW);
}

void setup() {
  Serial.begin(57600);

  //Setting I/O pins
  pinMode(joint1_EN, OUTPUT);
  pinMode(joint1_DIR, OUTPUT);
  pinMode(joint1_STEP, OUTPUT);
  pinMode(joint1_STOP, INPUT);
  pinMode(joint2_EN, OUTPUT);
  pinMode(joint2_DIR, OUTPUT);
  pinMode(joint2_STEP, OUTPUT);
  pinMode(joint2_STOP, INPUT);
  pinMode(joint3_EN, OUTPUT);
  pinMode(joint3_DIR, OUTPUT);
  pinMode(joint3_STEP, OUTPUT);
  pinMode(joint3_STOP, INPUT);
  pinMode(gripper_SIGNAL, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  //Toggling interrupts for endstops
  attachInterrupt(digitalPinToInterrupt(joint1_STOP), catch_interrupt_01, HIGH);
  attachInterrupt(digitalPinToInterrupt(joint2_STOP), catch_interrupt_23, HIGH);
  attachInterrupt(digitalPinToInterrupt(joint3_STOP), catch_interrupt_45, HIGH);
  
  //ROS init
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub_joint1);
  nh.subscribe(sub_joint2);
  nh.subscribe(sub_joint3);

  //Initialising Axis
  //zeros();

  //Publish first msg on chatter topic
  chatter.publish( &str_msg );

}

void loop() {
  
  str_msg.data = buffer_chatter;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(200);
}
