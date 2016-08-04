/*
 Test By tinnakon kheowree  Project ESP8266 Wifi Quadrotor
 tinnakon_za@hotmail.com
 tinnakonza@gmail.com
 http://quad3d-tin.lnwshop.com/
 https://www.facebook.com/tinnakonza

QuadTevada v2.0
by: tevada2010    
date: 25-11-2526(2013)
site: http://www.quadrotorthai.com
e-mail: quadrotorthai@gmail.com

app Drone.apk
Supat Boonyou
https://www.facebook.com/supat.fiat?fref=nf
kanin
https://www.facebook.com/profile.php?id=100001195621380&fref=ufi


 4/7/2559    ,,  Quad230_v3_0_esp8266_androidWifi_GY521  ,,write PID gyro ,set PWM 2 KHz
 4/8/2559    ,,  Quad230_v5_0_esp8266_androidWifi_GY521  ,,app Drone.apk  https://goo.gl/1ZUv9L
                 //
support: 
• ESP8266 Nodemcu
• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• X5C 230 mm
• i2c pin D2=SDA D1=SCL begin(int sda, int scl);

--------Type Quad-X-------------      

         6  12>>         <<14  5
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
         8  15>>        <<13  7

---------motor---------
FontLeft  => 12
FontRight => 14
BackLeft  => 15
BackRight => 13

----------rx-----------           
Wifi ,,ppm  => 0 , Key   ,12345678 ,
           
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "config.h"
#include "ahrs.h"
#include "apprx.h"
#include "mpu6050.h"
#include "motor.h"

float invSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
} 

int32_t isq(int32_t  x) {
  return x * x;
}

void setup()
{
 // initialize the serial link with processing
  Serial.begin(115200);
  Serial.println("Quad230_v4_0_esp8266_androidWifi_GY521.ino ");
  Wire.begin(D2,D1);//begin(int sda, int scl);
  Wire.setClock(400000);
//    pinMode(13, OUTPUT);
  WiFi.mode(WIFI_STA);
  delay(50);
  WiFi.disconnect();
  delay(200);
  pinMode(LED, OUTPUT);
  delay(200);
  sprintf(accessPointName, "BDrone-%lu", ESP.getChipId());
  WiFi.softAP(accessPointName, "12345678");
  delay(50);
  accessPointName[DEFAULT_SSID_LENGTH - 1] = {'\0'};
  WiFi.mode(WIFI_AP_STA);
  delay(50);
  udp.begin(localPort);
  delay(50);
  #ifdef Print_Debug
    Serial.println("");
    Serial.print("RemoteTest Access Point: ");
    Serial.println(accessPointName);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("UDP Port: ");
    Serial.println(localPort);
    Serial.println();
  #endif    
  ticker_DEGUG.attach_ms(1000, blink);  //LED Blink  
    mpu6050_initialize();
//    motor_initialize(); 
    delay(10);
    configureMotors();
//    configureReceiver();
    delay(10);
//    attachInterrupt(Receiver, SPPM_Read, RISING );
//    rx_initialize();
//    initializeReceiver(LASTCHANNEL);   
//    receiverXmitFactor = 1.0;    
    ahrs_initialize();       
    delay(10); 
    mpu6050_Gyro_Calibrate();
    mpu6050_Accel_Calibrate();
    delay(10);    
    commandAllMotors(1000);    
    armed = 0; 
    //digitalWrite(13, LOW);
    previousTime = micros(); 
    sensorPreviousTime = previousTime;
    hundredHZpreviousTime = previousTime;
}

void loop()
{
    int aux;
    float aux_float;  
    
    // Timer
    currentTime = micros();
    computeRC();
    // Read data (not faster then every 1 ms) // 1000 Hz
    if (currentTime - sensorPreviousTime >= 1000) 
    {
        mpu6050_readGyroSum();
        mpu6050_readAccelSum(); 
        sensorPreviousTime = currentTime;
    }
    
    // 100 Hz task loop (10 ms),, ,250 Hz = 4000 us ,,,, 400 Hz = 2500 us
    if (currentTime - hundredHZpreviousTime >= 10000) //10000
    {
        frameCounter++;
        G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
        hundredHZpreviousTime = currentTime; 
        mpu6050_Get_gyro();
        mpu6050_Get_accel();
       // for (int axis = XAXIS; axis <= ZAXIS; axis++) {
       //   filteredAccel[axis] = computeFourthOrder(accel[axis], &fourthOrder[axis]);
       // }    
   ////////////////Moving Average Filters///////////////////////////
      GyroXf = (gyro[XAXIS] + GyroX2)/2.0;
      GyroYf = (gyro[YAXIS] + GyroY2)/2.0;
      GyroZf = (gyro[ZAXIS] + GyroZ2)/2.0;
      GyroX2 = gyro[XAXIS];GyroY2 = gyro[YAXIS];GyroZ2 = gyro[ZAXIS];//gyro Old1
        ////////////////Low pass filter/////////////////////////////////
      //GyroZf = GyroZf + (GyroZ - GyroZf)*49.6*G_Dt;
    AccXf = AccXf + (accel[XAXIS] - AccXf)*0.121;//12.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    AccYf = AccYf + (accel[YAXIS] - AccYf)*0.121;//12.4
    AccZf = AccZf + (accel[ZAXIS] - AccZf)*0.121;//12.4
/////////////////////////////////////////////////////////////////////////
        //ahrs_updateIMU(gyro[XAXIS],gyro[YAXIS],gyro[ZAXIS], filteredAccel[XAXIS],filteredAccel[YAXIS],filteredAccel[ZAXIS],G_Dt);
        ahrs_updateIMU(GyroXf,GyroYf,GyroZf, AccXf,AccYf,AccZf,G_Dt);
//////////////////////////////////////////////////////////////////////////
        // YAW CONTROL
        err_yaw_rate = ((CH_RUD - yaw_mid)*0.0035) - gyro[ZAXIS];//0.0055  
        yaw_I_rate += err_yaw_rate*G_Dt;     
        yaw_I_rate = constrain(yaw_I_rate, -100, 100);        
        yaw_D_rate = (err_yaw_rate-err_yaw_ant_rate)/G_Dt;    
        err_yaw_ant_rate = err_yaw_rate;         
        control_yaw = (Kp_rateYaw*err_yaw_rate) + (Ki_rateYaw*yaw_I_rate) + (Kd_rateYaw*yaw_D_rate);
        control_yaw = constrain(control_yaw,-300,300);         

            // ROLL CONTROL angle
            err_roll_level = (((CH_AIL - roll_mid)*(0.0011)) - ahrs_r);//0.0015
            roll_I_level += err_roll_level*G_Dt; 
            roll_I_level = constrain(roll_I_level, -2, 2); 
            roll_D_level = (err_roll_level-err_roll_ant_level)/G_Dt; 
            err_roll_ant_level = err_roll_level;        
            control_roll_angle = Kp_levelRoll*err_roll_level + Ki_levelRoll*roll_I_level + Kd_levelRoll*roll_D_level; 
            control_roll_angle = constrain(control_roll_angle, -35, 35);
            // PITCH CONTROL angle
            err_pitch_level = (((CH_ELE - pitch_mid)*(-0.0011)) - ahrs_p);//-0.0011   
            pitch_I_level += err_pitch_level*G_Dt;  
            pitch_I_level = constrain(pitch_I_level, -2, 2);
            pitch_D_level = (err_pitch_level-err_pitch_ant_level)/G_Dt;       
            err_pitch_ant_level = err_pitch_level;   
            control_pitch_angle = Kp_levelPitch*err_pitch_level + Ki_levelPitch*pitch_I_level + Kd_levelPitch*pitch_D_level;   
            control_pitch_angle = constrain(control_pitch_angle, -35, 35);
            //ROLL CONTROL Gyro
            err_roll_rate = control_roll_angle - (gyro[XAXIS]);  
            roll_I_rate += err_roll_rate*G_Dt;     
            roll_I_rate = constrain(roll_I_rate, -10, 10);            
            roll_D_rate = (err_roll_rate-err_roll_ant_rate)/G_Dt;
            err_roll_ant_rate = err_roll_rate;             
            control_roll = (Kp_rateRoll*err_roll_rate) + (Ki_rateRoll*roll_I_rate) + (Kd_rateRoll*roll_D_rate);
            control_roll = constrain(control_roll,-300,300); 
            //PITCH CONTROL Gyro
            err_pitch_rate = control_pitch_angle - (gyro[YAXIS]);  
            pitch_I_rate += err_pitch_rate*G_Dt;     
            pitch_I_rate = constrain(pitch_I_rate, -10, 10);
            pitch_D_rate = (err_pitch_rate-err_pitch_ant_rate)/G_Dt;
            err_pitch_ant_rate = err_pitch_rate; 
            control_pitch = (Kp_ratePitch*err_pitch_rate) + (Ki_ratePitch*pitch_I_rate) + (Kd_ratePitch*pitch_D_rate);
            control_pitch = constrain(control_pitch,-300,300);
      
        if (armed == 0) 
        {
            motor_FRONTL = 0;
            motor_FRONTR = 0;
            motor_REARL = 0;
            motor_REARR = 0;  
            
            roll_I_rate=0;  
            pitch_I_rate=0;
            yaw_I_rate=0;
            
            roll_I_level=0;
            pitch_I_level=0;  
        }
        
        if (armed == 1)
        {
            
            if (CH_THR < 10) 
            {
        
                motor_FRONTL = 10;
                motor_FRONTR = 10;
                motor_REARL = 10;
                motor_REARR = 10; 
                
                roll_I_rate=0;  
                pitch_I_rate=0;
                yaw_I_rate=0;
                
                roll_I_level=0;
                pitch_I_level=0;

                r_KiTerm=0;
                p_KiTerm=0;
                y_KiTerm=0;
        
            }else{
                //x type
                motor_FRONTL = constrain((CH_THR + control_pitch + control_roll - control_yaw), 10, 1023);
                motor_FRONTR = constrain((CH_THR + control_pitch - control_roll + control_yaw), 10, 1023);
                motor_REARL = constrain((CH_THR - control_pitch + control_roll + control_yaw), 10, 1023);
                motor_REARR = constrain((CH_THR - control_pitch - control_roll - control_yaw), 10, 1023);           
                //motor_FRONTL = constrain((CH_THR - control_yaw), 10, 1023);
                //motor_FRONTR = constrain((CH_THR + control_yaw), 10, 1023);
                //motor_REARL = constrain((CH_THR + control_yaw), 10, 1023);
                //motor_REARR = constrain((CH_THR - control_yaw), 10, 1023); 
//                //+ type      
//                motor_FRONTL=constrain(CH_THR+control_pitch+control_yaw, 1150, 1900);
//                motor_REARR=constrain(CH_THR-control_pitch+control_yaw, 1150, 1900);
//                motor_REARL=constrain(CH_THR+control_roll-control_yaw, 1150, 1900);       
//                motor_FRONTR=constrain(CH_THR-control_roll-control_yaw, 1150, 1900);
            
            }              
        
        }  
        commandMotors();           

        // 50 Hz tak (20 ms)
        if (frameCounter % TASK_50HZ == 0) 
        {
        
            //ch1=CH_AIL;
            //ch2=CH_ELE;
            //ch3=CH_THR;
            //ch4=CH_RUD;
            //ch_aux1=AUX_1;
            //ch_aux2=AUX_2;
            //ch_aux3=AUX_3;    
         if (CH_THR < 10) 
        {
            if (CH_RUD > 1800 && armed == 0) //armed
            {
                armed = 1;
            }
            
            if (CH_RUD < 1100 && armed == 1) //not armed
            {    
                armed = 0;
    
            }
        }

        if(AUX_1<1300)
        {
          //mode=1;
         
        }
        
        if(AUX_1>1300)
        {
          //mode=2;
        }
         mode=2;      
}// end 50 Hz
        
        // 10 Hz task (100 ms)
        if (frameCounter % TASK_10HZ == 0) 
        {
            //G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
            //lowPriorityTenHZpreviousTime = currentTime;
            
//            //x
            Serial.print(motor_FRONTL);Serial.print("\t");
            Serial.print(motor_FRONTR);Serial.print("\t");
            Serial.print(motor_REARL);Serial.print("\t");    
            Serial.print(motor_REARR);Serial.print("\t");

//            Serial.print(r_KiTerm);Serial.print("\t");    
//            Serial.print(r_dInput);Serial.print("\t");     
//            Serial.print(r_value);Serial.print("\t");     
        
//            Serial.print(control_roll);Serial.print("\t");    
//            Serial.print(control_pitch);Serial.print("\t");     
//            Serial.print(control_yaw);Serial.print("\t");       

//            Serial.print(CH_THR);Serial.print("\t");
//            Serial.print((CH_AIL - roll_mid)*(0.75 * 0.002));Serial.print("\t");  
//            Serial.print((CH_ELE - pitch_mid)*(0.75 * 0.002));Serial.print("\t");
//            Serial.print((CH_RUD - yaw_mid)*(2.5 * 0.002));Serial.print("\t");  
//            Serial.print(AUX_1);Serial.print("\t"); 

            Serial.print(CH_THR);Serial.print("\t");
            Serial.print(CH_AIL);Serial.print("\t");  
            Serial.print(CH_ELE);Serial.print("\t");
            Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 

            //Serial.print(gyro[XAXIS]*180/PI);Serial.print("\t");
            //Serial.print(gyro[YAXIS]*180/PI);Serial.print("\t");
            //Serial.print(gyro[ZAXIS]*180/PI);Serial.print("\t");
        
            Serial.print(accel[XAXIS]);Serial.print("\t");
            Serial.print(accel[YAXIS]);Serial.print("\t");
            Serial.print(accel[ZAXIS]);Serial.print("\t");

            Serial.print(ahrs_r*180/PI);Serial.print("\t");
            Serial.print(ahrs_p*180/PI);Serial.print("\t");
            Serial.print(ahrs_y*180/PI);Serial.print("\t");

//            Serial.print(ahrs_r);Serial.print("\t");
//            Serial.print(ahrs_p);Serial.print("\t");
//            Serial.print(ahrs_y);Serial.print("\t");

//            Serial.print(accel_offset[XAXIS]);Serial.print("\t");
//            Serial.print(accel_offset[YAXIS]);Serial.print("\t");
//            Serial.print(accel_offset[ZAXIS]);Serial.print("\t");
  
            Serial.print(1/G_Dt);Serial.print("\t");
            Serial.println("");            
        }  
        // Reset frameCounter back to 0 after reaching 100 (1s)
        if (frameCounter >= TASK_1HZ) {
            frameCounter = 0;
        }
        
        //previousTime = currentTime;
    } //250 Hz
}


