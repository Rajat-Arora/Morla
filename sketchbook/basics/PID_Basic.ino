#include <PID_v1.h>
#include <L298NX2.h>

//-------------Motor1_PID------------

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Output1a;

//Specify the links and initial tuning parameters
double Kp1=2, Ki1=5, Kd1=1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

//------------Motor2_PID-----------------

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2, Output2a;

//Specify the links and initial tuning parameters
double Kp2=2, Ki2=5, Kd2=1;
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

float demand1;
float demand2;

unsigned long currentMillis;
unsigned long previousMillis;

//---------------- Encoder data ----------------------

# define  encoder1PinA  2 
# define  encoder1PinB  3 
# define  encoder2PinA  4   //  yet to be changed 
# define  encoder2PinB  5 

//volatile  long  encoder0Pos =  0 ; 
//volatile  long  encoder1Pos =  0 ; 

float encoder0Diff, encoder1Diff;

float encoder0Error, encoder1Error;

float encoder0Prev, encoder1Prev;

//---------------------- motors pin defination-----------------

const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);



void setup()
{
  Serial.begin(115200);
  
  //initialize the variables we're linked to
  pinMode (encoder1PinA, INPUT_PULLUP);  
  pinMode (encoder1PinB, INPUT_PULLUP);  

  pinMode (encoder2PinA, INPUT_PULLUP);  
  pinMode (encoder2PinB, INPUT_PULLUP);

  //  encoder pin on interrupt 0 (pin 2) 
  attachInterrupt ( 0 , doEncoderA, CHANGE); 
  
//  encoder pin on interrupt 1 (pin 3) 
  attachInterrupt ( 1 , doEncoderB, CHANGE);   
  
//  encoder pin on interrupt 2 (pin to be updated) 
  attachInterrupt ( 4 , doEncoderC, CHANGE);   

//  encoder pin on interrupt 3 (pin to be updated) 
  attachInterrupt ( 5 , doEncoderD, CHANGE); 


  //turn the PID1 on
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-200,200);
  myPID1.SetSampleTime(10);

  //turn the PID2 on
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-200,200);
  myPID2.SetSampleTime(10);
  
}

void loop()
{
  currentMillis =  millis ();

  if  (currentMillis - previousMillis >= 10){
    
    previousMillis = currentMillis;   

    if(Serial.available()>0){
      
      char c = Serial.read();

      if(c == 'w'){

        demand1 = 0.5;  //in m/s
        demand2 = 0.5;  //in m/s
        
        }
        
      else if(c=='e'){

        demand1 = 0.25;  //in m/s
        demand2 = 0.25;  //in m/s
              
        }
        
       else if(c=='q'){

        demand1 = 0.1;  //in m/s
        demand2 = 0.1;  //in m/s
              
        }
        
       else if(c=='d'){

        demand1 = 0.145;  //1 rad/s
        demand2 = -0.145;  //-1 m/s
              
        }

       else if(c=='a'){

        demand1 = -0.145;  //-1 rad/s
        demand2 = 0.145;  //1 rad/s
              
        }

       else if(c=='s'){

        demand1 = 0;  //in m/s
        demand2 = 0;  //in m/s
              
        }

       
      
      }
    }
  
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
