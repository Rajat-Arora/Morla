#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

ros::NodeHandle nh;

//-------------Motor1_PID------------

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Output1a;

//Specify the links and initial tuning parameters
double Kp1=3, Ki1=3, Kd1=0.03;
PID PID_MotorA(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

//------------Motor2_PID-----------------

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2, Output2a;

//Specify the links and initial tuning parameters
double Kp2=3, Ki2=3, Kd2=0.03;
PID PID_MotorB(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);


// Motors Enable and Input pins definition
const unsigned int enA = 5;
const unsigned int in1 = 8;
const unsigned int in2 = 9;

const unsigned int in3 = 11;
const unsigned int in4 = 10;
const unsigned int enB = 6;


//Left and Right Motors 
float demand1; 
float demand2;

// Sent through /cmd/vel topic in m/s and rad/s
float demandx;
float demandz;

// Actual left and right wheel speed.
//float speed_act_left;
//float speed_act_right;

//Variables to take care of Loop Time.
unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;

//Encoder pin definations
#define baudRate_ROSserial 19200

#define encoder0PinA 2
#define encoder0PinB 4

#define encoder1PinA 3
#define encoder1PinB 7

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

float encoder0Diff;
float encoder1Diff;

// Callback for Twist messages subscriber.

void velocity_callback(const geometry_msgs::Twist& velocity){
  
    demandx = velocity.linear.x;
    demandz = velocity.angular.z;

    demandx = constrain(demandx, -0.25, 0.25);
    demandz = constrain(demandz, -1, 1);
  
  }

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &velocity_callback);
//geometry_msgs::Vector3Stamped speed_msg;
//ros::Publisher speed_pub("speed", &speed_msg);


void setup() {
  nh.loginfo("In setup1");
  //Serial.begin(19200);
  nh.initNode();
  nh.getHardware()->setBaud(baudRate_ROSserial);
  nh.loginfo("In setup2");
  nh.subscribe(sub);
//  nh.advertise(speed_pub);
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Set all the encoder pins
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  

  PID_MotorA.SetOutputLimits(-200,200);  
  PID_MotorA.SetMode(AUTOMATIC); 
  PID_MotorA.SetSampleTime(10);

  PID_MotorB.SetOutputLimits(-200,200);  
  PID_MotorB.SetMode(AUTOMATIC); 
  PID_MotorB.SetSampleTime(10);

  while(!nh.connected()){
    nh.spinOnce();
    }
  
  nh.loginfo("In setup end");
}

//void publishSpeed (double time);

void loop() {

   // make sure we listen for ROS messages and activate the callback if there is one

  currentMillis = millis();
  
  if (currentMillis - previousMillis >= loopTime) { // start timed loop for everything else
    
    previousMillis = currentMillis;

    // work out the two values for differential drive of each wheel
    demand1 = demandx - (demandz*0.145);
    demand2 = demandx + (demandz*0.145);

    encoder0Diff = encoder0Pos - encoder0Prev;
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    encoder0Error = (demand1*129);
    encoder1Error = (demand2*129);
  
    encoder0Prev = encoder0Pos;
    encoder1Prev = encoder1Pos;

    // drive wheel 1 at 129 counts per
    Setpoint1 = demand1*129;
    Input1 = encoder0Diff;
    PID_MotorA.Compute();

    Setpoint2 = demand2*129;
    Input2 = encoder1Diff;
    PID_MotorB.Compute();

    //work out actual motor speed from wheel encoders

 //   speed_act_left = encoder0Diff/129;
  //  speed_act_right = encoder1Diff/129;

    // 1 metre/sec is 129 counts per 10ms loop
    // so we look at the counts per 10ms loop and divide by 129 to get the m/s

    //Running the motors
    if(Output1 > 0){
      Output1a = abs(Output1);
      analogWrite(enA, Output1a);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      }

    else if(Output1 < 0){
      Output1a = abs(Output1);
      analogWrite(enA, Output1a);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }  
    else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }


    if(Output2 > 0) {
      Output2a = abs(Output2);
      analogWrite(enB, Output2a);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
     }
    
    else if(Output2 < 0) {
      Output2a = abs(Output2);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

  //publishSpeed(loopTime); //Publish odometry on ROS Topic.
  nh.spinOnce();
}
}

/*

void publishSpeed (double time) {

  speed_msg.header.stamp = nh.now(); //timestamp
  speed_msg.vector.x = speed_act_left; //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;//right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;//looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

*/
