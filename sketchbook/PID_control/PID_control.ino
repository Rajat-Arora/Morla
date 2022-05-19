//--arduino libraries--
#include<ros.h> //to use ros inbuilt functions
#include<Wire.h> // for I2C communication
#include <std_msgs/String.h> 
#include <geometry_msgs/Vector3Stamped.h> 
#include <geometry_msgs/Twist.h>
#include <string.h>

//--ROS Messages files--
//#include<std_msgs/Int16MultiArray.h>
#include<sensor_msgs/Imu.h>
//#include<std_msgs/Int16.h>
#include <PID_v1.h>


// global variables for imu
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


#define baudRate 19200
#define baudRate_ROSserial 19200

//#define current_jointData_rt "/joint_states"
//#define jointReadings_forActuators_rt "/gait_value_topic"
ros::NodeHandle nh;

//std_msgs::Int16 vari;
//sensor_msgs::Imu imu_msg;
//int8_t c=5;

//--callback function for subscribing to trajectory data from CPU (champ_controller node)

//ros::Subscriber<std_msgs::Int16MultiArray> _trajectory(jointReadings_forActuators_rt, &callback_trajectory);
//ros::Publisher _testing_value("/rosserial_to_ROS", &vari);
//ros::Publisher _imu("/rosserial_to_ROS", &imu_msg);
//-------------Motor1_PID------------

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Output1a;

//Specify the links and initial tuning parameters
double Kp1=3, Ki1=0, Kd1=0.000;
PID PID_MotorA(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

//------------Motor2_PID-----------------

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2, Output2a;

//Specify the links and initial tuning parameters
double Kp2=3, Ki2=0, Kd2=0.000;
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
unsigned long previousMillis = 0;
int loopTime = 10;

//Encoder pin definations
#define baudRate_ROSserial 19200

#define encoder0PinA 2
#define encoder0PinB 3

#define encoder1PinA 18
#define encoder1PinB 19

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

float encoder0Error;
float encoder1Error;

float encoder0Prev = 0;
float encoder1Prev = 0;

float encoder0Diff;
float encoder1Diff;

void velocity_callback(const geometry_msgs::Twist& velocity){
  
    demandx = velocity.linear.x;
    demandz = velocity.angular.z;
    nh.loginfo("in callback");
//    float asd = 12.6;
//    int a = 5;
//    nh.loginfo(trial.c_str());
//    nh.loginfo((int)velocity.linear.x);
//    nh.loginfo("velocity.linear.z");
    demandx = constrain(demandx, -0.25, 0.25);
    demandz = constrain(demandz, -1, 1);
//    nh.loginfo(String(demandx).c_str());
//    nh.loginfo(String(demandz).c_str());
    //String y_str = String(a);
    
//==========code from loop()============
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &velocity_callback);
//geometry_msgs::Vector3Stamped speed_msg;
//ros::Publisher speed_pub("speed", &speed_msg);



void setup()
{
  

  nh.getHardware()->setBaud(baudRate_ROSserial);

  nh.initNode();

 nh.loginfo("in setup");
 //nh.subscribe(_trajectory);
 //nh.advertise(_imu);
 //nh.subscribe(sub);
 nh.subscribe(sub);
 while(!nh.connected()){
                 nh.spinOnce();
         } 
 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
nh.loginfo("in mid setup");
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Set all the encoder pins
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);


  attachInterrupt(0, doEncoderA, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);  
  
// encoder pin on interrupt 2 (pin to be updated)
  attachInterrupt(4, doEncoderC, CHANGE); //try 5 

// encoder pin on interrupt 3 (pin to be updated)
  attachInterrupt(5, doEncoderD, CHANGE); //try 4
  
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

 
 //nh.spinOnce();
 //nh.advertise(_testing_value);
 
// while(!nh.connected()){
//                 nh.spinOnce();
//         } 
          
 nh.loginfo("setup end");  
}

void loop()
{


currentMillis = millis();
  
if (currentMillis - previousMillis >= loopTime) { // start timed loop for everything else
    
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
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      }

    else if(Output1 > 0){
      Output1a = abs(Output1);
      analogWrite(enA, Output1a);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }  
    else {
      analogWrite(enA, Output1a);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }


    if(Output2 < 0) {
      Output2a = abs(Output2);
      analogWrite(enB, Output2a);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
     }
    
    else if(Output2 > 0) {
      Output2a = abs(Output2);
      analogWrite(enB, Output2a);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
    else {
      analogWrite(enB, Output2a);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }

  

  nh.spinOnce();
 // delay(10);
}
}


void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}


// encoder 2

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  } 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  } 
}
