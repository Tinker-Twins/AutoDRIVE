//======================================================================================================================================================================
// LIBRARIES
//======================================================================================================================================================================

#include <Wire.h>

//======================================================================================================================================================================
// REGISTER MAP
//======================================================================================================================================================================
// Refer "MPU-9250 Register Map and Descriptions, Rev. 1.4 (9/9/2013)" for details pertaining to the register map.

//-----------------------------------------------------------------------------
// Register Map for Gyroscope and Accelerometer (MPU6050)
//-----------------------------------------------------------------------------

#define MPU6050_ADDRESS     0x68 // Address of gyroscope and accelerometer

#define SELF_TEST_X_GYRO    0x00                  
#define SELF_TEST_Y_GYRO    0x01                                                                          
#define SELF_TEST_Z_GYRO    0x02

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E    
#define SELF_TEST_Z_ACCEL   0x0F

#define XG_OFFSET_H         0x13
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E   
#define WOM_THR             0x1F

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24   
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38

#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I_MPU6050    0x75 // Device ID (should return 0x71)
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

//-----------------------------------------------------------------------------
// Register Map for Magnetometer (AK8963)
//-----------------------------------------------------------------------------

#define AK8963_ADDRESS      0x0C // Address of magnetometer

#define WHO_AM_I_AK8963     0x00 // Device ID (should return 0x48)
#define INFO                0x01 // Information
#define AK8963_ST1          0x02 // Status 1
#define AK8963_XOUT_L       0x03 // Measurement data
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09 // Status 2
#define AK8963_CNTL         0x0A // Control
#define AK8963_ASTC         0x0C // Self-test
#define AK8963_I2CDIS       0x0F // I2C disable
#define AK8963_ASAX         0x10 // X-axis sensitivity adjustment value
#define AK8963_ASAY         0x11 // Y-axis sensitivity adjustment value
#define AK8963_ASAZ         0x12 // Z-axis sensitivity adjustment value

//======================================================================================================================================================================
// MODES
//======================================================================================================================================================================

#define dataFiltering   true // FALSE: raw data read (acc, gyro, mag) | TRUE: processed data read (roll, pitch, yaw)
#define serialDebug     true // FALSE: no serial output | TRUE: get serial output for debugging
#define magCalibration  true // FALSE: do not calibrate magnetometer | TRUE: calibrate magnetometer

//======================================================================================================================================================================
// VARIABLE DECLARATIONS
//======================================================================================================================================================================

//-----------------------------------------------------------------------------
// Sensor Parameters
//-----------------------------------------------------------------------------

enum Ascale {
  AFS_2G = 0, // 2 g
  AFS_4G,     // 4 g
  AFS_8G,     // 8 g
  AFS_16G     // 16 g
};

enum Gscale {
  GFS_250DPS = 0, // 250 deg/s
  GFS_500DPS,     // 500 deg/s
  GFS_1000DPS,    // 1000 deg/s
  GFS_2000DPS     // 2000 deg/s
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;

uint8_t Mmode = 0x02;     // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
float   aRes, gRes, mRes; // Scale resolutions per LSB for the sensors

//-----------------------------------------------------------------------------
// Variable Declarations
//-----------------------------------------------------------------------------

int16_t accelCount[3];            // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];             // Stores the 16-bit signed gyroscope sensor output
int32_t magCount[3];              // Stores the 16-bit signed magnetometer sensor output
int16_t tempCount;                // Stores the raw temperature count

float   magScale[3] = {0, 0, 0};  // Stores the magnetometer sensitivity adjustment values
float   magBias[3] = {29707.0566, 29817.3750, 28528.2285};   // Stores the magnetometer bias corrections (may be initialized using values from previous calibration)
float   gyroBias[3] = {0, 0, 0};  // Stores the gyroscope bias corrections
float   accelBias[3] = {0, 0, 0}; // Stores the accelerometer bias corrections

float   SelfTest[6];              // Stores the results of gyroscope and accelerometer self-test

float temperature;                        // Stores the actual internal chip temperature in °C
float ax, ay, az, gx, gy, gz, mx, my, mz; // Variables to store raw sensor data
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // Stores the quaternion-based estimate of absolute device orientation
float roll, pitch, yaw;                   // Variables to store roll, pitch and yaw values

// float magDeclination = 1.2167; /* Declination at Potheri, TN 603203, India
//                                 Tool:         https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
//                                 Model Used:   IGRF2020
//                                 Latitude:     12° 49' 23" N
//                                 Longitude:    80° 2' 34" E
//                                 Date:         2023-11-18
//                                 Declination:  1° 13' (1.2167°) W changing by  0° 1' E per year
//                                 Convention:   +ve for west
//                              */

float magDeclination = 6.9167; /* Declination at Greenville, SC 29607, USA
                                Tool:         https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
                                Model Used:   IGRF2020
                                Latitude:     34° 50' 39" N
                                Longitude:    82° 21' 55" W
                                Date:         2023-11-18
                                Declination:  6° 55' (6.9167°) W changing by  0° 3' W per year
                                Convention:   +ve for west
                             */

// Free parameters in the Madgwick filter and fusion scheme
/*
 There is a tradeoff in the beta parameter between accuracy and response speed. This is essentially like the
 I-coefficient in a PID control; the bigger the feedback coefficient, the faster the solution converges, usually
 at the expense of accuracy.
 In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to
 give optimal accuracy. However, with this value, the response time was ~10 sec to a stable initial quaternion.
 Increasing beta (GyroMeasError) by a factor of about fifteen reduced the response time constant to ~2 sec
 without noticable reduction in the solution accuracy.
*/
#define GyroMeasError PI * (40.0f / 180.0f);    // Gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift PI * (0.0f  / 180.0f);    // Gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // Compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // Compute zeta (usually set to a small or zero value)

// Free parameters in the Mahony filter and fusion scheme
#define Kp 2.0f * 5.0f // Proportional feedback gain
#define Ki 0.0f        // Integral feedback gain

float eInt[3] = {0.0f, 0.0f, 0.0f}; // Vector to hold integral error for Mahony filter scheme

// Parameters to compute integration interval for both filter schemes
float deltat = 0.0f;
uint32_t Now = 0, lastUpdate = 0;

//======================================================================================================================================================================
// SETUP
//======================================================================================================================================================================

void setup()
{
  //-----------------------------------------------------------------------------
  // Start Communication
  //-----------------------------------------------------------------------------
  
  Wire.begin();
  Serial.begin(115200);

  //-----------------------------------------------------------------------------
  // Initialization
  //-----------------------------------------------------------------------------

  if(serialDebug){Serial.println("Initializing...\n");} delay(500);
  
  byte c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050); delay(500); // Read the WHO_AM_I register of MPU6050, this is a good test of communication

  if(c == 0x71) {
    if(serialDebug){Serial.println("MPU6050 detected!\n");}

    //-----------------------------------------------------------------------------
    // MPU6050 Self-Test
    //-----------------------------------------------------------------------------

    if(serialDebug){Serial.println("Performing MPU6050 self-test...");}
    MPU6050SelfTest(SelfTest); // Start by performing self-test and reporting values
    if(serialDebug) {
      Serial.println("Acceleromete trim:");
      Serial.print("X: "); Serial.print(SelfTest[0],2); Serial.print("%"); Serial.print("\tY: "); Serial.print(SelfTest[1],2); Serial.print("%"); Serial.print("\tZ: "); Serial.print(SelfTest[2],2); Serial.print("%"); Serial.println("\tof factory value");
      Serial.println("Gyroscope trim:");
      Serial.print("X: "); Serial.print(SelfTest[3],2); Serial.print("%"); Serial.print("\tY: "); Serial.print(SelfTest[4],2); Serial.print("%"); Serial.print("\tZ: "); Serial.print(SelfTest[5],2); Serial.print("%"); Serial.println("\tof factory value\n");
    }
    delay(500);

    //-----------------------------------------------------------------------------
    // MPU6050 Calibration
    //-----------------------------------------------------------------------------
    
    if(serialDebug){Serial.println("Keep the device steady on a flat surface...");}
    delay(5000); // Give some time to place the device on a flat surface
    if(serialDebug){Serial.println("Initializing MPU6050 calibration...\n");}
    calibrateMPU6050(gyroBias, accelBias); // Calibrate gyroscope and accelerometer and load bias values into the bias registers
    if(serialDebug) {
      Serial.println("MPU6050 calibrated!");
      Serial.println("Accelerometer bias:");
      Serial.print("X: "); Serial.print(accelBias[0], 4); Serial.print("\tY: "); Serial.print(accelBias[1], 4); Serial.print("\tZ: "); Serial.print(accelBias[2], 4); Serial.println("\tg");
      Serial.println("Gyroscope bias:");
      Serial.print("X: "); Serial.print(gyroBias[0], 4); Serial.print("\tY: "); Serial.print(gyroBias[1], 4); Serial.print("\tZ: "); Serial.print(gyroBias[2], 4); Serial.println("\tdeg/s\n");
    }
    initMPU6050(); // Initialize the device for active mode read of acclerometer, gyroscope, and thermometer
    delay(500);
  }

  byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); delay(500); // Read the WHO_AM_I register of the AK8963, this is a good test of communication

  if(d == 0x48) {
    if(serialDebug){Serial.println("AK8963 detected!\n");}

    //-----------------------------------------------------------------------------
    // AK8963 Calibration
    //-----------------------------------------------------------------------------

    initAK8963(magScale); // Get magnetometer calibration from AK8963 ROM and initialize the device for active mode read of magnetometer
    if(magCalibration) {
      if(serialDebug){Serial.println("Wave the device in 8 figure until done...");}
      delay(5000); // Give some time to pick up the device and start waving it
      if(serialDebug){Serial.println("Initializing AK8963 calibration...\n");}
      getMres();
      calibrateAK8963(magBias); // Calibrate magnetometer and update bias values
    }
    if(serialDebug) {
      Serial.println("AK8963 calibrated!");
      Serial.println("Magnetometer sensitivity:");
      Serial.print("X: "); Serial.print(magScale[0], 4); Serial.print("\tY: "); Serial.print(magScale[1], 4); Serial.print("\tZ: "); Serial.println(magScale[2], 4);
      Serial.println("Magnetometer bias:");
      Serial.print("X: "); Serial.print(magBias[0], 4); Serial.print("\tY: "); Serial.print(magBias[1], 4); Serial.print("\tZ: "); Serial.print(magBias[2], 4); Serial.println("\tmG\n");
    }
    delay(500);
  }
  
  if(c != 0x71 | d != 0x48) {
    Serial.println("MPU9250 not detected!");
    Serial.print("MPU6050: "); Serial.print("I am "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
    Serial.print("AK8963: "); Serial.print("I am "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX); Serial.println("");
    delay(500);
    setup() ; // Loop within `setup()` function until MPU9250 is detected
  }
}

//======================================================================================================================================================================
// LOOP
//======================================================================================================================================================================

void loop()
{
  //-----------------------------------------------------------------------------
  // Read Raw Sensor Data
  //-----------------------------------------------------------------------------
  
  // Check if data ready interrupt
  //if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
    
    // Accelerometer
    readAccelData(accelCount); // Read the X,Y,Z ADC values
    getAres();
    // Convert the accelerometer reading into g's
    ax = (float)accelCount[0]*aRes;
    ay = (float)accelCount[1]*aRes;
    az = (float)accelCount[2]*aRes;
    
    // Gyroscope
    readGyroData(gyroCount); // Read the X,Y,Z ADC values
    getGres();
    // Convert the gyroscope reading into deg/s
    gx = (float)gyroCount[0]*gRes;
    gy = (float)gyroCount[1]*gRes;
    gz = (float)gyroCount[2]*gRes;

    //Magnetometer
    readMagData(magCount); // Read the X,Y,Z ADC values
    getMres();
    // Convert the magnetometer reading into mG
    // Include factory calibration (as per data sheet) and hard iron corrections
    mx = (float)magCount[0]*mRes*magScale[0] - magBias[0];
    my = (float)magCount[1]*mRes*magScale[1] - magBias[1];
    mz = (float)magCount[2]*mRes*magScale[2] - magBias[2];

    // Thermometer
    tempCount = readTempData();  // Read the adc values
    // Convert the thermometer reading into °C
    temperature = ((float) tempCount) / 333.87 + 21.0;
  //}

  if(!dataFiltering & serialDebug) {
    Serial.print("Accelerometer:"); Serial.print("\tX: "); Serial.print(ax, 4); Serial.print("\tY: "); Serial.print(ay, 4); Serial.print("\tZ: "); Serial.print(az, 4); Serial.println("\tg");
    Serial.print("Gyroscope:"); Serial.print("\tX: "); Serial.print(gx, 4); Serial.print("\tY: "); Serial.print(gy, 4); Serial.print("\tZ: "); Serial.print(gz, 4); Serial.println("\tdeg/s");
    Serial.print("Magnetometer:"); Serial.print("\tX: "); Serial.print(mx, 4); Serial.print("\tY: "); Serial.print(my, 4); Serial.print("\tZ: "); Serial.print(mz, 4); Serial.println("\tmG");
    Serial.print("Thermometer:\t"); Serial.print(temperature, 2);  Serial.println("\t°C\n");
  }

  //-----------------------------------------------------------------------------
  // Filter Raw Sensor Data
  //-----------------------------------------------------------------------------
  /*
  The raw sensor data is filtered using either of the following Bayesian estimation filters:
    - Madgwick filter (more accurate, but slower)
    - Mahony filter (less accurate but faster)
  */
  if(dataFiltering) {
    
    // Set integration time
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // Time elapsed since last filter update
    lastUpdate = Now;
    
    /*
    NOTE 1:
    X, Y-axis of the magnetometer (AK8963) is aligned with the Y, X-axis of the gyroscope and accelerometer (MPU6050), respectively and
    the Z-axis (+ down) of the magnetometer (AK8963) is opposite to Z-axis (+ up) of the gyroscope and accelerometer (MPU6050). Thus, we
    have to correct this orientation mismatch while feeding the data to the quaternion filter. We have chosen a magnetic rotation that
    keeps the sensor forward along the X-axis. This rotation can be modified to allow any convenient orientation convention; this is okay
    by aircraft orientation standards!
    
    NOTE 2:
    The gyro rates need to be passed in rad/s
    */
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    /*
    Get orientation in the form of Tait-Bryan angles (i.e. roll, pitch, yaw) from the updated quaternion.
    
    COORDINATE SYSTEM:
    The following conventions arise from the definition of homogeneous rotation matrix constructed from quaternions.
      - Positive Z-axis is down toward Earth. 
      - Yaw is the angle between sensor X-axis and Earth magnetic North (or true North if corrected for local declination). Looking down on the sensor positive yaw is counterclockwise.
      - Pitch is the angle between sensor X-axis and Earth ground plane. Toward the Earth is positive, up toward the sky is negative.
      - Roll is the angle between sensor Y-axis and Earth ground plane. Y-axis up is positive roll.
    
    NOTE:
    Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation, the
    rotations must be applied in the correct order, which for this configuration is yaw, pitch, and then roll.
    For more information, see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
    */
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   += magDeclination;
    roll  *= 180.0f / PI;
    Serial.print("Roll:"); Serial.print(roll, 2); Serial.print(","); Serial.print("Pitch:"); Serial.print(pitch, 2); Serial.print(","); Serial.print("Yaw:"); Serial.println(yaw+180, 2);
  }
}

//======================================================================================================================================================================
// HELPER FUNCTIONS
//======================================================================================================================================================================

//-----------------------------------------------------------------------------
// Data Read Functions
//-----------------------------------------------------------------------------

void getAres()
{
  switch (Ascale) {
    /*
    Possible accelerometer scales (and their register bit settings) are:
      - 2 g (00)
      - 4 g (01)
      - 8 g (10)
      - 16 g  (11)
    The resolution is returned in g
    */
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void getGres()
{
  switch (Gscale) {
    /*
    Possible gyroscope scales (and their register bit settings) are:
      - 250 DPS (00)
      - 500 DPS (01)
      - 1000 DPS (10)
      - 2000 DPS (11)
    The resolution is returned in deg/s
    */
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getMres()
{
  switch (Mscale) {
     /*
    Possible magnetometer scales (and their register bit settings) are:
      - 14 bit resolution (0)
      - 16 bit resolution (1)
    The resolution is returned in mG
    */
    case MFS_14BITS:
          mRes = 10.*4912./8190.;
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];
  // Read the six raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];
  // Read the six raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int32_t * destination)
{
  uint8_t rawData[7];
  // Wait for magnetometer data ready bit to be set
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
    // Read the six raw data registers sequentially into data array (must read ST2 at the end of data acquisition)
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
    // End data read by reading the ST2 register
    uint8_t c = rawData[6];
    // Check if magnetic sensor overflow set, if not then report data
    if(!(c & 0x08)) {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];
  // Read the two raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1] ;
}

//-----------------------------------------------------------------------------
// Initialization and Calibration Functions
//-----------------------------------------------------------------------------

void initMPU6050()
{
  /*
  This function initializes the MPU6050 by configuring accelerometer, gyroscope and thermometer.
  */
  
  // Wake up device
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset
  
  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready, else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);
  
  /*
  Set SAMPLE_RATE = Internal_Sample_Rate/(1 + SMPLRT_DIV)
  
  NOTE: The accelerometer, gyroscope, and thermometer are set to 1 kHz sample rates, but all
  these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  */
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate determined inset in CONFIG

  /*
  Configure Gyroscope and Thermometer:
    - Disable FSYNC
    - Set thermometer and gyroscope bandwidth to 41 and 42 Hz, respectively
    - Limit the gyroscope sample rate to 1 kHz
    - Set gyroscope full scale range configuration
  */

  writeByte(MPU6050_ADDRESS, CONFIG, 0x03); // Configure DLPF_CFG; disable FSYNC, set thermometer and gyroscope bandwidth to 41 and 42 Hz, respectively, and limit the gyroscope sample rate to 1 kHz

  uint8_t c = readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x02);       // Clear Fchoice_b bits [1:0]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);       // Clear FS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyroscope

  /*
  Configure Accelerometer:
    - Set accelerometer full scale range configuration
    - Set accelerometer bandwidth to 41 Hz
    - Limit accelerometer sample rate to 1 kHz
  */

  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);        // Clear FS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);  // Set full scale range for the accelerometer 

  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG2, c | 0x03);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  /*
  Configure Interrupts and Bypass Enable:
    - Set interrupt pin active high, push-pull; hold interrupt pin level HIGH until interrupt cleared; clear on read of INT_STATUS
    - Eenable I2C_BYPASS_EN so additional chips can join the I2C bus and all can be controlled by the Arduino as master
    - Enable data ready interrupt
  */
   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22); // Set interrupt pin active high and enable bypass mode
   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready interrupt (bit 0)
   delay(100);
}

void MPU6050SelfTest(float * destination)
{
  /*
  This function performs MPU6050 self-test in order to check calibration w.r.t. factory settings.
  It should return percent deviation from factory trim values. +/- 14 or less deviation is a pass.
  */
  
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;
  
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyroscope sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, CONFIG, 0x02);        // Set gyroscope sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyroscope to 250 dps
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  // Get average current values of gyroscope and acclerometer
  for (int ii = 0; ii < 200; ii++) {
    // Read the six accelerometer raw data registers sequentially into data array
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six gyroscope raw data registers sequentially into data array
    readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++) {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
  // Configure the accelerometer for self-test
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self-test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self-test on all three axes and set gyroscope range to +/- 250 deg/s
  delay(25); // Delay a while to let the device stabilize

  // Get average self-test values of gyroscope and acclerometer
  for (int ii = 0; ii < 200; ii++) {
  // Read the six accelerometer raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  // Read the six gyroscope raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++) {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }
  
  // Configure the gyroscope and accelerometer for normal operation
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);  
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0x00);  
  delay(25); // Delay a while to let the device stabilize
   
  // Retrieve accelerometer and gyroscope factory self-test results from USR_Reg
  selfTest[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accelerometer self-test results
  selfTest[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accelerometer self-test results
  selfTest[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accelerometer self-test results
  selfTest[3] = readByte(MPU6050_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyroscope self-test results
  selfTest[4] = readByte(MPU6050_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyroscope self-test results
  selfTest[5] = readByte(MPU6050_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyroscope self-test results

  // Retrieve factory trim values from self-test reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }
}

void calibrateMPU6050(float * dest1, float * dest2)
{
  /*
  This function accumulates gyroscope and accelerometer data after device initialization. It calculates the average
  of the at-rest readings and then loads the resulting offsets into accelerometer and gyroscope bias registers.
  */
  
  uint8_t data[12]; // Array to hold accelerometer and gyroscope X, Y, Z data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  // Reset device
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a 1 to reset bit (bit 7); toggle reset device
  delay(100);
  
  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready, else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);
  
  // Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
  // Configure gyroscope and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyroscope full-scale to 250 deg/s (maximum sensitivity)
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g (maximum sensitivity)
  
  uint16_t  gyrosensitivity  = 131;   // 131 LSB/deg/s
  uint16_t  accelsensitivity = 16384; // 16384 LSB/g
  
  // Configure FIFO to capture accelerometer and gyroscope data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);    // Enable gyroscope and accelerometer sensors for FIFO (max size 512 bytes)
  delay(40);                                    // Accumulate 40 samples in 40 milliseconds = 480 bytes
  
  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);            // Disable gyroscope and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // Read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];      // FIFO count
  packet_count = fifo_count/12;                         // Packet count for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the Z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
  /*
  Construct the gyroscope biases for push to the hardware gyroscope bias registers:
  
  These registers are reset to zero upon device startup. So we can directly update them with the gyroscope biases.
  */
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyroscope biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  // Push gyroscope biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU6050_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFSET_L, data[5]);
  
  // Store scaled gyroscope biases to display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  /*
  Construct the accelerometer biases for push to the hardware accelerometer bias registers:
  
  These registers contain factory trim values which must be added to the calculated accelerometer biases; on boot
  up these registers will hold non-zero values. In addition, bit 0 of the lower byte must be preserved since it is
  used for temperature compensation calculations.
  */
  
  int32_t accel_bias_reg[3] = {0, 0, 0}; // Array to hold the factory accelerometer trim biases
  // Read factory accelerometer trim values
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  /*
  Construct total accelerometer bias, including calculated average accelerometer bias from above:
  
  Subtract calculated average accelerometer bias scaled to 2048 LSB/g (16 g full scale) followed by a logical AND
  operation with 0xFFFE in order to leave the temperature compensation (last bit) unchanged by the computation.
  */
  accel_bias_reg[0] -= ((accel_bias[0]/8) & 0xFFFE);
  accel_bias_reg[1] -= ((accel_bias[1]/8) & 0xFFFE); 
  accel_bias_reg[2] -= ((accel_bias[2]/8) & 0xFFFE);
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  
  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L, data[5]);
  
  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
  dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
  dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void initAK8963(float * destination)
{
  /*
  This function initializes the magnetometer (AK8963) by configuring it.
  */
  
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]); // Read the X, Y and Z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;  // Return X-axis sensitivity adjustment values
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  // Return Y-axis sensitivity adjustment values
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;  // Return Z-axis sensitivity adjustment values
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  /*
  Configure the magnetometer for continuous read and highest resolution:
  Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register, and enable continuous
  mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates.
  */
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void calibrateAK8963(float * destination)
{
  /*
  This function performs hard-iron correction on the magnetometer in order to calibrate it. This process is tedious
  and takes time and may not be used everytime the device is initialized (set magCalibration to false). Instead, the
  bias values reported previously may be noted down and pre-configured into the magBias array.
  */
  
  int32_t mag_bias[3] = {0, 0, 0};
  int32_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
  int sample_count = 128;
  
  for (int ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the magnetometer data
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135); // At 8 Hz ODR, new magnetometer data is available every 125 ms
  }
  
  // Get average magnetometer bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;
  
  // Save magnetometer biases in mG (for main routine)
  destination[0] = (float) mag_bias[0]*mRes*magScale[0];
  destination[1] = (float) mag_bias[1]*mRes*magScale[1];
  destination[2] = (float) mag_bias[2]*mRes*magScale[2];
}

//-----------------------------------------------------------------------------
// Sensor Fusion Algorithms
//-----------------------------------------------------------------------------

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  /*
  The Madgwick filter fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
  device orientation, which can be converted to roll, pitch and yaw. The performance of this orientation filter is at least as
  good as conventional Kalman-based filtering algorithms but is computationally much less intensive; it can be performed even on
  a 3.3 V Pro Mini operating at 8 MHz!
  
  CITATION:
  Sebastian O.H. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays," Technical
  Report; Report x-io and University of Bristol: Bristol, UK, 30 April 2010.
  */
  
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // Short-name local variables for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  
  // Auxiliary variables to avoid repeated arithmetic operations
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  
  // Normalize accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if(norm == 0.0f) return; // Handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;
  
  // Normalize magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if(norm == 0.0f) return; // Handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;
  
  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;
  
  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  
  // Normalize step magnitude
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;
  
  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
  
  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  
  // Normalize quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  /*
  The Mahony filter is similar to Madgwick filter but uses proportional and integral filtering on the error between estimated
  reference vectors and measured ones.
  
  CITATION:
  R. Mahony, T. Hamel and J. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group," in IEEE Transactions
  on Automatic Control, vol. 53, no. 5, pp. 1203-1218, June 2008, doi: 10.1109/TAC.2008.923738.
  */
  
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // Short-name local variables for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;
  
  // Auxiliary variables to avoid repeated arithmetic operations
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;   
  
  // Normalize accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if(norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;
  
  // Normalize magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if(norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;
  
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
  
  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  
  
  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if(Ki > 0.0f) {
    // Accumulate integral error
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else {
    // Prevent integral wind-up
    eInt[0] = 0.0f;
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }
  
  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];
  
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);
  
  // Normalize quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

//-----------------------------------------------------------------------------
// Wire.h Read and Write Protocols
//-----------------------------------------------------------------------------
        
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data;                             // Stores the register data
	Wire.beginTransmission(address);          // Initialize the Tx buffer
	Wire.write(subAddress);	                  // Put slave register address in Tx buffer
	Wire.endTransmission(false);              // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);   // Read one byte from slave register address
	data = Wire.read();                       // Fill Rx buffer with result
	return data;                              // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.endTransmission(false);      // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
  Wire.requestFrom(address, count); // Read bytes from slave register address 
	while (Wire.available()) {
    dest[i++] = Wire.read();        // Put read results in the Rx buffer
  }
}
