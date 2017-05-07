#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "SR04.h"
#include "DualMC33926MotorShield.h"

//connect SR04
const int TRIG_1=50;
const int ECHO_1=51;
const int TRIG_2=52;
const int ECHO_2=53;
SR04 sr01 = SR04(ECHO_1, TRIG_1);
SR04 sr02 = SR04(ECHO_2, TRIG_2);

//listen to ROS
ros::NodeHandle nh;
DualMC33926MotorShield md;
double lin_vel = 0;
double ang_vel = 0;
int cmd_ctrl = 0;
int abs_speed = 0;
int abs_speed2 = 0;
std_msgs::String str_msg;

void motor_cb(const geometry_msgs::Twist& vel){
  Serial.print("motor_cb\n");
  lin_vel=vel.linear.x;
  ang_vel=vel.angular.z;
  cmd_ctrl=1*lin_vel+3*ang_vel;
}

ros::Publisher pub("pub",&str_msg);
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", motor_cb);

//actions
void wait(){
  abs_speed=0;
  cmd_ctrl=0;
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(200);
}

int lookAround(){
  if(sr01.Distance()<1||sr02.Distance()<1)
  {
     Serial.print("thing!!!!\n");
     wait();
     cmd_ctrl=0;
     return 1;
  }
  return 0;
}

void walk(){
  for(int i=0; i<4; i++){
    //if(lookAround()){
      //break;
    //}
    delay(500);
  }
}

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  md.init();
  nh.advertise(pub);
  nh.subscribe(sub);
  abs_speed=0;
}

void loop()
{  
  nh.spinOnce();
  Serial.print("distance: \t");
  Serial.print(sr01.Distance());
  Serial.print("\t");
  Serial.print(sr02.Distance());
  Serial.print("\n");
  switch(cmd_ctrl){
    case 2:{
      Serial.print("forward\n");
      abs_speed=70;
      md.setM1Speed(abs_speed);
      md.setM2Speed(abs_speed);
      //walk();
      delay(1000);
      cmd_ctrl=0;
      break;
    }
    case -2:{
      Serial.print("backward\n");
      abs_speed=70;
      md.setM1Speed(-abs_speed);
      md.setM2Speed(-abs_speed);
      //walk();
      delay(1000);
      cmd_ctrl=0;
      break;
      //delay(0)
    }
    case 6:{
      Serial.print("left\n");
      abs_speed=180;
      md.setM1Speed(abs_speed);
      md.setM2Speed(-abs_speed);
      delay(1000);
      cmd_ctrl=0;
      break;
    }
    case -6:{
      Serial.print("right\n");
      abs_speed=180;
      abs_speed2=100;
      md.setM1Speed(-abs_speed);
      md.setM2Speed(abs_speed);
      delay(1000);
      cmd_ctrl=0;
      break;
    }
    case 1:{
      Serial.print("stop\n");
      abs_speed=0;
      md.setM1Speed(abs_speed);
      md.setM2Speed(abs_speed);
      cmd_ctrl=0;
      break;
      //delay(0)
    }
    default:{
      Serial.print("confused\n");
      wait();
      break;
    }
  }
  lookAround();
}


