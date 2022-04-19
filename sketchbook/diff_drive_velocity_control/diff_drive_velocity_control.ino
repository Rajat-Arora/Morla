#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <PID_v1.h>

ros::NodeHandle nh;

//-------------Motor1_PID------------

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Output1a;

//Specify the links and initial tuning parameters
double Kp1=3, Ki1=3, Kd1=0.03;
PID PID_motorA(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

//------------Motor2_PID-----------------

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2, Output2a;

//Specify the links and initial tuning parameters
double Kp2=3, Ki2=3, Kd2=0.03;
PID PID_motorB(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

//Left and Right Motors 
float demand1; 
float demand2;

// Sent through /cmd/vel topic in m/s and rad/s
float demand_x;
float demand_z;

// Actual left and right wheel speed.
float actual_speed_l;
float actual_speed_r;

//Variables to take care of Loop Time.
unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;

//Encoder pin definations

#define encoder0PinA 2
#define encoder0PinB 3

#define encoder1PinA 15
#define encoder1PinB 14

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;


// Callback for Twist messages subscriber.

void velocity_callback( const geometry_msgs::Twist& velocity){
  
    demandx = velocity.linear.x;
    demandz = velocity.angular.z;

    demandx = constrain(demandx, -0.25, 0.25);
    demandz = constrain(demandz, -1, 1);
  
  }

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velocity_callback);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);


void setup() {
  
  nh.initNode();

}

void loop() {
  // put your main code here, to run repeatedly:

}
