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
#define Kp 0.21			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.00051		// integral gain governs rate of convergence of gyroscope biases

float exInt , eyInt , ezInt ;	// scaled integral error
//unsigned long last,now;

float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
float ahrs_p,ahrs_r,ahrs_y;
unsigned long prevTime;

void ahrs_initialize()
{
  exInt=0;
  eyInt=0;
  ezInt=0;
  q0=1.0;
  q1=q2=q3=0;
//  now=micros();
}
void ahrs_toEuler()
{
  /* STANDARD ZYX
   y=atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1);
   p=-asin(2 * q1 * q3 + 2 * q0 * q2); // theta
   r=atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
   */
  ahrs_y=atan2(2*q1*q2+2*q0*q3,2*q0*q0+2*q1*q1-1);
  ahrs_p=-asin(2 * q1 * q3 - 2 * q0 * q2); // theta
  ahrs_r=atan2(2 * q2 * q3 + 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
}
void ahrs_updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt) {

  float norm,halfT;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  halfT=G_Dt/2.0;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          

  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;         

  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;        

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  // integral error scaled integral gain
  exInt += ex*Ki;
  eyInt += ey*Ki;
  ezInt += ez*Ki;

  // adjusted gyroscope measurements
  gx +=Kp*ex + exInt;
  gy +=Kp*ey + eyInt;
  gz +=Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  ahrs_toEuler();
}

void ahrs_updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float G_Dt) {
  float norm,halfT;
  float vx, vy, vz;
  float ex, ey, ez;         

  halfT=G_Dt/2.0;

  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;      

  // estimated direction of gravity
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // error is sum of cross product between reference direction of field and direction measured by sensor
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);

  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  ahrs_toEuler();
}
