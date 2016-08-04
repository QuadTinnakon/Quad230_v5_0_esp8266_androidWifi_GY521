/*
 Test By tinnakon kheowree  Project ESP8266 Wifi Quadrotor Quad230_v1_0_esp8266_androidWifi.ino
 tinnakon_za@hotmail.com
 tinnakonza@gmail.com
 http://quad3d-tin.lnwshop.com/
 https://www.facebook.com/tinnakonza
 
support: 
• ESP8266
• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• X5C 230 mm
*/
////motor
//int MOTOR_FRONTL_PIN = 3;
//int MOTOR_FRONTR_PIN = 10;
//int MOTOR_REARL_PIN = 11;
//int MOTOR_REARR_PIN = 9;
//
//int motor_FRONTL;
//int motor_FRONTR;
//int motor_REARL;
//int motor_REARR;
//
//#define MINCOMMAND 1000
//#define MIDCOMMAND 1500
//#define MAXCOMMAND 2000
//
//
//
////motor command
//void motor_initialize() 
//{
//  pinMode(MOTOR_FRONTL_PIN,OUTPUT);  
//  pinMode(MOTOR_FRONTR_PIN,OUTPUT); 
//  pinMode(MOTOR_REARL_PIN,OUTPUT); 
//  pinMode(MOTOR_REARR_PIN,OUTPUT); 
//}
//
//void motor_command() 
//{
//  analogWrite(MOTOR_FRONTL_PIN, motor_FRONTL/8);
//  analogWrite(MOTOR_FRONTR_PIN, motor_FRONTR/8);
//  analogWrite(MOTOR_REARL_PIN, motor_REARL/8);
//  analogWrite(MOTOR_REARR_PIN, motor_REARR/8);
//}
//
//void motor_command_all(int Values) 
//{
//  analogWrite(MOTOR_FRONTL_PIN, Values/8);
//  analogWrite(MOTOR_FRONTR_PIN, Values/8);
//  analogWrite(MOTOR_REARL_PIN, Values/8);
//  analogWrite(MOTOR_REARR_PIN, Values/8);
//}
//
//


//motor
int MOTOR_FRONTL_PIN = 12;//>>
int MOTOR_FRONTR_PIN = 14;//<<
int MOTOR_REARL_PIN = 15;//>>
int MOTOR_REARR_PIN = 13;//<<

//Servo mFront_L;
//Servo mFront_R;
//Servo mBack_L;
//Servo mBack_R;

int motorCommand_FRONTL;
int motorCommand_FRONTR;
int motorCommand_REARL;
int motorCommand_REARR;

int motor_FRONTL;
int motor_FRONTR;
int motor_REARL;
int motor_REARR;

#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000

void configureMotors() {
  
  pinMode(MOTOR_FRONTL_PIN,OUTPUT);  
  pinMode(MOTOR_FRONTR_PIN,OUTPUT); 
  pinMode(MOTOR_REARL_PIN,OUTPUT); 
  pinMode(MOTOR_REARR_PIN,OUTPUT);
  analogWriteFreq(500);//pwm_freq = 1000; 2000 , 4000 , Hz

//  mFront_L.attach(MOTOR_FRONTL_PIN);
//  mFront_R.attach(MOTOR_FRONTR_PIN);
//  mBack_L.attach(MOTOR_REARL_PIN);
//  mBack_R.attach(MOTOR_REARR_PIN);
   

}

void commandAllMotors(int motorCommand) {

  analogWrite(MOTOR_FRONTL_PIN, (motorCommand-1000)*1.136);
  analogWrite(MOTOR_FRONTR_PIN, (motorCommand-1000)*1.136);
  analogWrite(MOTOR_REARL_PIN, (motorCommand-1000)*1.136);
  analogWrite(MOTOR_REARR_PIN, (motorCommand-1000)*1.136);

//  motorCommand=map(motorCommand, 1000, 2000, 0, 180);

//  mFront_L.write(motorCommand);
//  mFront_R.write(motorCommand);
//  mBack_L.write(motorCommand);
//  mBack_R.write(motorCommand);

}

void commandMotors() {
  
 //analogWrite(MOTOR_FRONTL_PIN, (int)(motor_FRONTL-1000)*0.816);//PWM 0 - 1023 ,,10 bit
 //analogWrite(MOTOR_FRONTR_PIN, (int)(motor_FRONTR-1000)*0.816);
 //analogWrite(MOTOR_REARL_PIN, (int)(motor_REARL-1000)*1.136);
 //analogWrite(MOTOR_REARR_PIN, (int)(motor_REARR-1000)*1.136);
 analogWrite(MOTOR_FRONTL_PIN, motor_FRONTL);//PWM 0 - 1023 ,,10 bit
 analogWrite(MOTOR_FRONTR_PIN, motor_FRONTR);
 analogWrite(MOTOR_REARL_PIN, motor_REARL);
 analogWrite(MOTOR_REARR_PIN, motor_REARR);
 
//  motor_FRONTL=map(motor_FRONTL, 1000, 2000, 0, 180);
//  motor_FRONTR=map(motor_FRONTR, 1000, 2000, 0, 180);
//  motor_REARL=map(motor_REARL, 1000, 2000, 0, 180);
//  motor_REARR=map(motor_REARR, 1000, 2000, 0, 180);
//  mFront_L.write(motor_FRONTL);
//  mFront_R.write(motor_FRONTR);
//  mBack_L.write(motor_REARL);
 // mBack_R.write(motor_REARR);  
}
