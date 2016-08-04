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
// Axis definitions
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

int                   num_readings = 100;
float                 x_accel = 0;
float                 y_accel = 0;
float                 z_accel = 0;
float                 x_gyro = 0;
float                 y_gyro = 0;
float                 z_gyro = 0;

float gyro[3];
float accel[3];

float AccXf,AccYf,AccZf;
float GyroXf,GyroYf,GyroZf;
float GyroX2,GyroY2,GyroZ2;

float gyroScaleFactor = radians(2000.0 / 32768.0);
//float gyroScaleFactor = (0.0174532 / 16.4);

//float accelScaleFactor = 9.81 / 8192.0;
float accelScaleFactor = 9.81 / 4096.0;

uint16_t sensors_detected = 0x00;

uint8_t gyroSamples = 0;
uint8_t accelSamples = 0; 

int16_t gyroRaw[3];
float gyroSum[3];

int16_t accelRaw[3];
float accelSum[3];

int16_t gyro_offset[3];
float accel_offset[3];  

//sensor MPU6050 -------------------------------------
// MPU 6050 Registers
#define MPU6050_ADDRESS         0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74

// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01

//sensor ---------------------
    
void mpu6050_initialize()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);    // Chip reset DEVICE_RESET 1
    Wire.write(BIT_H_RESET);//DEVICE_RESET
    Wire.endTransmission();  
    delay(100);// Startup delay      
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(MPU_CLK_SEL_PLLGYROZ);//CLKSEL 3 (PLL with Z Gyro reference)
    Wire.endTransmission();    
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_SMPLRT_DIV);// SAMPLE RATE
    Wire.write(0x00);//// Sample rate = 1kHz
    Wire.endTransmission();  
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_CONFIG);
    Wire.write(BITS_DLPF_CFG_42HZ);//98 BITS_DLPF_CFG_188HZ, BITS_DLPF_CFG_42HZ, default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    Wire.endTransmission();  
    delay(5);   
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_CONFIG);
    Wire.write(BITS_FS_2000DPS);//BITS_FS_1000DPS FS_SEL = 3: Full scale set to 2000 deg/sec,  BITS_FS_2000DPS
    Wire.endTransmission(); 
    delay(5);  
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_CONFIG);
    Wire.write(0x10);//0x10 = 1G=4096, AFS_SEL=2 (0x00 = +/-2G)  1G = 16,384 //0x10 = 1G = 4096 ,//0x08 = +-4g
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_INT_PIN_CFG);// enable I2C bypass for AUX I2C
    Wire.write(0x00);//0x00=off  0x02 , I2C_BYPASS_EN=1
    Wire.endTransmission();
    delay(1500);   
}

void mpu6050_Gyro_Values()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_XOUT_H);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    
    gyroRaw[XAXIS] = ((Wire.read() << 8) | Wire.read());
    gyroRaw[YAXIS] = ((Wire.read() << 8) | Wire.read())*-1;
    gyroRaw[ZAXIS] = ((Wire.read() << 8) | Wire.read())*-1;
}	

void mpu6050_Gyro_Calibrate()
{
    static uint8_t retry = 0;
    uint8_t i, count = 28;//128
    int16_t xSum = 0, ySum = 0, zSum = 0;
    
    int16_t axSum = 0, aySum = 0, azSum = 0;
  
    for (i = 0; i < count; i++) {
        mpu6050_Gyro_Values();
        
        xSum += gyroRaw[XAXIS];
        ySum += gyroRaw[YAXIS];
        zSum += gyroRaw[ZAXIS];

//        digitalWrite(13, LOW);
          digitalWrite(LED, LOW);
        delay(20);
//        digitalWrite(13, HIGH);
          digitalWrite(LED, HIGH);
       delay(20);
    }
    
    gyro_offset[XAXIS] = -xSum / count;
    gyro_offset[YAXIS] = -ySum / count;
    gyro_offset[ZAXIS] = -zSum / count; 
}

void mpu6050_readGyroSum() {
    mpu6050_Gyro_Values();
    
    gyroSum[XAXIS] += gyroRaw[XAXIS];
    gyroSum[YAXIS] += gyroRaw[YAXIS];
    gyroSum[ZAXIS] += gyroRaw[ZAXIS];
    
    gyroSamples++;
};

void mpu6050_Get_gyro()
{       
    // Calculate average
    gyro[XAXIS] = gyroSum[XAXIS] / gyroSamples;
    gyro[YAXIS] = gyroSum[YAXIS] / gyroSamples;
    gyro[ZAXIS] = gyroSum[ZAXIS] / gyroSamples;    
    
    // Apply offsets
    gyro[XAXIS] += gyro_offset[XAXIS];
    gyro[YAXIS] += gyro_offset[YAXIS];
    gyro[ZAXIS] += gyro_offset[ZAXIS];         
    
    // Apply correct scaling (at this point gyro is in radians)
    gyro[XAXIS] *= gyroScaleFactor;
    gyro[YAXIS] *= gyroScaleFactor;
    gyro[ZAXIS] *= gyroScaleFactor;
    
    // Reset SUM variables
    gyroSum[XAXIS] = 0;
    gyroSum[YAXIS] = 0;
    gyroSum[ZAXIS] = 0;
    gyroSamples = 0;            
}

void mpu6050_Accel_Values()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    accelRaw[XAXIS] = ((Wire.read() << 8) | Wire.read())*-1;
    accelRaw[YAXIS] = ((Wire.read() << 8) | Wire.read());
    accelRaw[ZAXIS] = ((Wire.read() << 8) | Wire.read());
}


void mpu6050_Accel_Calibrate()
{
    static uint8_t retry = 0;
    uint8_t i, count = 28;//128
    int16_t xSum = 0, ySum = 0, zSum = 0;
    
    int16_t axSum = 0, aySum = 0, azSum = 0;
  
//    for (i = 0; i < count; i++) {
//        mpu6050_Accel_Values();
//        
//
//        axSum += accelRaw[XAXIS];
//        aySum += accelRaw[YAXIS];
//        azSum += accelRaw[ZAXIS];        
//        
//        
//        digitalWrite(13, LOW);
//        delay(20);
//        digitalWrite(13, HIGH);
//        delay(20);
//    }
//    
//    accel_offset[XAXIS] = -axSum / count;
//    accel_offset[YAXIS] = -aySum / count;
//    accel_offset[ZAXIS] = ((-azSum) / count) - 8192; // - 1G;    

    for (i = 0; i < count; i++) {
        mpu6050_Accel_Values();
        
        accelSum[XAXIS] += accelRaw[XAXIS];
        accelSum[YAXIS] += accelRaw[YAXIS];
        accelSum[ZAXIS] += accelRaw[ZAXIS];        
        
        
//        digitalWrite(13, LOW);
          digitalWrite(LED, LOW);
        delay(20);
//        digitalWrite(13, HIGH);
          digitalWrite(LED, HIGH);
       delay(20);
    }
    
    accel_offset[XAXIS] = (accelSum[XAXIS] / count)*accelScaleFactor;
    accel_offset[YAXIS] = (accelSum[YAXIS] / count)*accelScaleFactor;
    accel_offset[ZAXIS] = (accelSum[ZAXIS] / count)*accelScaleFactor;
    Serial.print(accel_offset[XAXIS]);Serial.print("\t");
    Serial.print(accel_offset[YAXIS]);Serial.print("\t");
    Serial.print(accel_offset[ZAXIS]);Serial.print("\t");
    Serial.println("");  
    accel_offset[XAXIS] = 1.35;//1.35
    accel_offset[YAXIS] = 1.11;//0.83
}

void mpu6050_readAccelSum() {
    mpu6050_Accel_Values();
    
    accelSum[XAXIS] += accelRaw[XAXIS];
    accelSum[YAXIS] += accelRaw[YAXIS];
    accelSum[ZAXIS] += accelRaw[ZAXIS];  

    accelSamples++;  
};

void mpu6050_Get_accel()
{
    // Calculate average
    accel[XAXIS] = ((accelSum[XAXIS] / accelSamples)*accelScaleFactor)-accel_offset[XAXIS];
    accel[YAXIS] = ((accelSum[YAXIS] / accelSamples)*accelScaleFactor)-accel_offset[YAXIS];
    accel[ZAXIS] = ((accelSum[ZAXIS] / accelSamples)*accelScaleFactor)/*-accel_offset[ZAXIS]*/;  

    // Apply offsets
//    accel[XAXIS] += accel_offset[XAXIS];
//    accel[YAXIS] += accel_offset[YAXIS];
//    accel[ZAXIS] += accel_offset[ZAXIS];
               
    // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
//    accel[XAXIS] *= accelScaleFactor;
//    accel[YAXIS] *= accelScaleFactor;
//    accel[ZAXIS] *= accelScaleFactor;
    
    // Reset SUM variables
    accelSum[XAXIS] = 0;
    accelSum[YAXIS] = 0;
    accelSum[ZAXIS] = 0;
    accelSamples = 0;         
}
