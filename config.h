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
#define TASK_100HZ 1  //3
#define TASK_50HZ 2  //5
#define TASK_10HZ 10 //25
#define TASK_1HZ 100 //100 250

// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;
//unsigned long frameCounter = 0; // main loop executive frame counter

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
uint32_t itterations = 0;

float G_Dt = 0.002; 

byte mode = 1;

//#define PI 3.141592653589793f
#define D_LPF_HZ 20
#define LOOP_S 0.01f //assuming we are running at 200Hz

static float d_lpf = LOOP_S / (LOOP_S + 1/(2*PI*D_LPF_HZ));

//roll angle
float r_Kp=5.0f;
float r_Ki=0.0f;
float r_Kd=0.0f;
float r_max=0.0f;
float r_imax=0.0f;
float r_value = 0.0f;
float r_lastInput = 0.0f;
float r_lastDInput = 0.0f;
float r_KiTerm = 0.0f;
float r_dInput = 0.0f;

//roll rate
float rate_r_Kp=1.0f;
float rate_r_Ki=0.55;
float rate_r_Kd=0.0f;
float rate_r_max=0.0f;
float rate_r_imax=0.0f;
float rate_r_value = 0.0f;
float rate_r_lastInput = 0.0f;
float rate_r_lastDInput = 0.0f;
float rate_r_KiTerm = 0.0f;
float rate_r_dInput = 0.0f;

//pitch level
float p_Kp=5.0f;
float p_Ki=0.0f;
float p_Kd=0.0f;
float p_max=0.0f;
float p_imax=0.0f;
float p_value = 0.0f;
float p_lastInput = 0.0f;
float p_lastDInput = 0.0f;
float p_KiTerm = 0.0f;
float p_dInput = 0.0f;

//pitch rate
float rate_p_Kp=1.0f;
float rate_p_Ki=0.55f;
float rate_p_Kd=0.0f;
float rate_p_max=0.0f;
float rate_p_imax=0.0f;
float rate_p_value = 0.0f;
float rate_p_lastInput = 0.0f;
float rate_p_lastDInput = 0.0f;
float rate_p_KiTerm = 0.0f;
float rate_p_dInput = 0.0f;

//yaw level
float y_Kp=0.0f;
float y_Ki=0.0f;
float y_Kd=0.0f;
float y_max=0.0f;
float y_imax=0.0f;
float y_value = 0.0f;
float y_lastInput = 0.0f;
float y_lastDInput = 0.0f;
float y_KiTerm = 0.0f;
float y_dInput = 0.0f;

//yaw rate
float rate_y_Kp=3.0f;
float rate_y_Ki=0.0f;
float rate_y_Kd=.0f;
float rate_y_max=0.0f;
float rate_y_imax=0.0f;
float rate_y_value = 0.0f;
float rate_y_lastInput = 0.0f;
float rate_y_lastDInput = 0.0f;
float rate_y_KiTerm = 0.0f;
float rate_y_dInput = 0.0f;

//float r_input=0.0;
//float p_input=0.0;
//float y_input=0.0;// -(ahrs_r*180/PI);
			
float loop_s = 0.01f;    




//PID-------------Rate
float Kp_rateRoll = 225.2;//60.00 80
float Ki_rateRoll = 0.35;//0.25
float Kd_rateRoll = 10.00;//5.00

float Kp_ratePitch = 225.2;//80
float Ki_ratePitch = 0.35;
float Kd_ratePitch = 10.00; //-3

float Kp_rateYaw = 350.0;
float Ki_rateYaw = 2.55;
float Kd_rateYaw = 2.0;

//PID--------------Stable
float Kp_levelRoll= 8.51;//6.0 
float Ki_levelRoll= 0.00;//0.0
float Kd_levelRoll= 0.12;//0.0

float Kp_levelPitch= 8.51;
float Ki_levelPitch= 0.00;
float Kd_levelPitch= 0.12;

float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

float yaw_I_rate;
float yaw_D_rate;
float err_yaw_rate=0;
float err_yaw_ant_rate=0;


float roll_I_level;
float roll_D_level;
float err_roll_level=0;
float err_roll_ant_level=0;

float pitch_I_level;
float pitch_D_level;
float err_pitch_level=0;
float err_pitch_ant_level=0;

float altitude_I;
float altitude_D;
float err_altitude=0;
float err_altitude_ant=0;

float last_input_roll=0;
float last_input_pitch=0;

float control_roll_angle=0.0;          
float control_pitch_angle=0.0;
float control_yaw_angle=0.0;


float control_roll=0.0;          
float control_pitch=0.0;
float control_yaw=0.0;

float holdAltitude=0.0;
float altitude=0.0;
int thr=0;


#define tar 0.001
byte armed;

float velocityCompFilter1 = 1.0 / (1.0 + 0.3);
float velocityCompFilter2 = 1 - velocityCompFilter1;

boolean runtimaZBiasInitialized = false;  
float zVelocity = 0.0;
float estimatedZVelocity = 0.0;
float runtimeZBias = 0.0; 
float zDampeningThrottleCorrection = 0.0;


float filteredAccel[3] = {0.0,0.0,0.0};
