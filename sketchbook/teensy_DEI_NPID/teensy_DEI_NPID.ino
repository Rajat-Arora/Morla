//--arduino libraries--
#include<ros.h>
#include<Wire.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Vector3Stamped.h> 
#include <geometry_msgs/Twist.h>
#include <string.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Int16.h>
#include <PID_v1.h>
#include <TeensyThreads.h>
#include <Encoder.h>
 

Encoder knobLeft(0, 1);
Encoder knobRight(2, 3);


//=============================IMU GLOBAL VARIABLES=======================================

// global variables for imu
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


#define baudRate 19200
#define baudRate_ROSserial 19200

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
std_msgs::Int16 left_ticks;
std_msgs::Int16 right_ticks;

void velocity_callback(const geometry_msgs::Twist& velocity); //Forward declaration of velocity_callback

ros::Publisher _imu("/rosserial_imu", &imu_msg);
ros::Publisher _left_encoder("/left_icks", &left_ticks);
ros::Publisher _right_encoder("/right_ticks", &right_ticks);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &velocity_callback);


//============================MOTOR GLOBAL VARIABLES========================================

//-------------Motor1_PID------------

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Output1a;

//Specify the links and initial tuning parameters
double Kp1=2.8, Ki1=0.009, Kd1=0.002;
PID PID_MotorA(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

//------------Motor2_PID-----------------

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2, Output2a;

//Specify the links and initial tuning parameters
double Kp2=2.8, Ki2=0.009, Kd2=0.002;
PID PID_MotorB(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);


// Motors Enable and Input pins definition
const unsigned int enA = 4;
const unsigned int in1 = 6;
const unsigned int in2 = 7;

const unsigned int in3 = 9;
const unsigned int in4 = 8;
const unsigned int enB = 5;


//Left and Right Motors 
float demand1; 
float demand2;

// Sent through /cmd/vel topic in m/s and rad/s
float demandx;
float demandz;

//Variables to take care of Loop Time.
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long loopTime = 10;

// Data related to encoders

long encoder0Pos = 0;
long encoder1Pos = 0;

float encoder0Error;
float encoder1Error;


long encoder0Prev = 0;
long encoder1Prev = 0;

float encoder0Diff;
float encoder1Diff;


//Callback to subscribe velocity and fill it in demandx and demandz for sending it to motors.

void velocity_callback(const geometry_msgs::Twist& velocity){
  
    demandx = velocity.linear.x;
    demandz = velocity.angular.z;
    nh.loginfo("in callback");

    demandx = constrain(demandx, -0.5, 0.5);
    demandz = constrain(demandz, -1, 1);
    
}



//==================================Thread to handle motors==================================



//===================================Thread Function to handle IMU Data to ROS===============================================

void imu_to_ros() {
  while(1){
    nh.loginfo("in void loop");

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers  String AX = String(mpu6050.getAccX());
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  imu_msg.angular_velocity.x=float(GyX);
  imu_msg.angular_velocity.y=float(GyY);
  imu_msg.angular_velocity.z=float(GyZ);
  imu_msg.linear_acceleration.x=float(AcX);
  imu_msg.linear_acceleration.y=float(AcY);
  imu_msg.linear_acceleration.z=float(AcZ);
  
  //nh.loginfo(GyX);

  _imu.publish(&imu_msg);
  nh.spinOnce();
  delay(10);
  
  }

}


//===================================Thread Function to handle Encoder Data to ROS===============================================

void encoderLeft_to_ros(){
 while(1){
  
  left_ticks.data = int(encoder0Pos);
  //nh.loginfo(String(encoder0Pos).c_str());
  _left_encoder.publish(&left_ticks);
  
  nh.spinOnce();
  delay(12);
 }
 
}

void encoderRight_to_ros(){
 while(1){
  
  right_ticks.data = int(encoder1Pos);
  //nh.loginfo(String(encoder1Pos).c_str());
  _right_encoder.publish(&right_ticks);
  
  nh.spinOnce();
  delay(13);
 }
 
}


void update_encoder(){

  while(1){
      encoder0Pos = -knobLeft.read();
      encoder1Pos = -knobRight.read();
      nh.spinOnce();
      delay(15);
  }
  
}



void setup()
{
  
//=============================IMU Setup===============================================
 
 threads.delay(5000);
 nh.getHardware()->setBaud(baudRate_ROSserial);
 nh.initNode();

 while(!nh.connected()){
                 nh.spinOnce();
         } 

 
 nh.loginfo("in setup imu");

 nh.advertise(_imu);
 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
 
 //nh.loginfo("setup end");


//=============================Encoder publisher Setup===============================================

 nh.advertise(_left_encoder);
 nh.advertise(_right_encoder);


//===========================Motor Setup=======================================

 nh.loginfo("in setup motor");
 
 nh.subscribe(sub);
 
 // Set all the motor control pins to outputs
 pinMode(enA, OUTPUT);
 pinMode(enB, OUTPUT);
 pinMode(in1, OUTPUT);
 pinMode(in2, OUTPUT);
 pinMode(in3, OUTPUT);
 pinMode(in4, OUTPUT);

  
 // Turn off motors - Initial state
 digitalWrite(in1, LOW);
 digitalWrite(in2, LOW);
 digitalWrite(in3, LOW);
 digitalWrite(in4, LOW);
  

 //Set PID Limits and parameters. Might need to modify
 
 PID_MotorA.SetOutputLimits(-300,300);
 PID_MotorA.SetMode(AUTOMATIC);
 PID_MotorA.SetSampleTime(10);

 PID_MotorB.SetOutputLimits(-300,300);  
 PID_MotorB.SetMode(AUTOMATIC); 
 PID_MotorB.SetSampleTime(10);

 
 nh.spinOnce();
 
 //========================================Teensy Threading========================================

 threads.addThread(imu_to_ros);
 threads.addThread(encoderRight_to_ros);
 threads.addThread(encoderLeft_to_ros);  
 threads.addThread(update_encoder);       
 nh.loginfo("setup end whole");  

}


void loop()
{

  //Publishing PWM and IN1...IN4 readings to the motor driver.
 
  currentMillis = millis();
  
  if (currentMillis - previousMillis >= loopTime) {       // start timed loop for everything else
    
    previousMillis = currentMillis;

    // work out the two values for differential drive of each wheel
    demand1 = demandx + (demandz*0.145); //change acccording to robot and application
    demand2 = demandx - (demandz*0.145);
    
//    nh.loginfo(int(demandx));
//    nh.loginfo(int(demandz));

    encoder0Diff = encoder0Pos - encoder0Prev;
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    encoder0Error = (demand1*129) - encoder0Diff;
    encoder1Error = (demand2*129) - encoder1Diff;
  
    encoder0Prev = encoder0Pos;
    encoder1Prev = encoder1Pos;
//    nh.loginfo(String(demand1).c_str());
//    nh.loginfo(String(demand2).c_str());
    // drive wheel 1 at 129 counts per
    Setpoint1 = demand1*500;
    Input1 = encoder0Diff;
    PID_MotorA.Compute();

    Setpoint2 = demand2*500;
    Input2 = encoder1Diff;
    PID_MotorB.Compute();

    //work out actual motor speed from wheel encoders

 //   speed_act_left = encoder0Diff/129;
  //  speed_act_right = encoder1Diff/129;

    // 1 metre/sec is 129 counts per 10ms loop
    // so we look at the counts per 10ms loop and divide by 129 to get the m/s

    //Running the motors
    if(Output1 < 35 && Output1 > -35) {Output1 = 0;}
    if(Output2 < 35 && Output2 > -35) {Output2 = 0;}
    nh.loginfo(String(Output1).c_str());
    nh.loginfo(String(Output2).c_str());

    
    if(Output1 < 0){
      Output1a = abs(Output1);
      analogWrite(enA, Output1a);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      }

    else if(Output1 > 0){
      Output1a = abs(Output1);
      analogWrite(enA, Output1a);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }  
    else {
      analogWrite(enA, Output1a);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }


    if(Output2 < 0) {
      Output2a = abs(Output2);
      analogWrite(enB, Output2a);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
     }
    
    else if(Output2 > 0) {
      Output2a = abs(Output2);
      analogWrite(enB, Output2a);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
    else {
      analogWrite(enB, Output2a);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }

  

  nh.spinOnce();

}
  
  threads.delay(10);

}
