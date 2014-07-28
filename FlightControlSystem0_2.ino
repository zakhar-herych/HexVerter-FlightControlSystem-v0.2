// We'll use Servo to control our ESCs
#include <Servo.h> 
// Libraries for interaction with IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// PWM pins
#define ESCPin1_TT 3
#define ESCPin2_LT 5
#define ESCPin3_TB 6
#define ESCPin4_RB 9
#define ESCPin5_RT 10
#define ESCPin6_LB 11

// Here we declare the ESCs,... we treat them as servos
Servo ESC_RT;
Servo ESC_LT;
Servo ESC_TT;
Servo ESC_RB;
Servo ESC_LB;
Servo ESC_TB;

int offset_RT = 0;
int offset_LT = 0;
int offset_TT = 0;
int offset_RB = 0;
int offset_LB = 0;
int offset_TB = 0;

int PWMValue;      // current PWM value
int PWMMax = 141;  // maximum PWM value
int PWMMin = 55;   // minimum PWM value that causes the motors to spin
int PWMZero = 0;   // PWM zeroing value used to calibrate the ESC
int increment = 5; // PWM increment value

// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Orientation that we get from sensors
sensors_vec_t   orientation;

void displayAccelerometerDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayGyroDetails(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayMagnetometerDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


// Init all the sensors of our IMU
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
//  displayGyroDetails();
//  displayAccelerometerDetails();
//  displayMagnetometerDetails();
}

boolean showAccelerometer = true;
boolean showGyroscope = true;
boolean showMagnetometer = true;

void getSensorData(){
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_event_t gyro_event;
  
  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  gyro.getEvent(&gyro_event);

  if (showAccelerometer || showGyroscope || showMagnetometer) {
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print(" ");
  }
  
  if (showAccelerometer) {
    Serial.print("Accelerometer: ");
    Serial.print("X: "); Serial.print(accel_event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(accel_event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(accel_event.acceleration.z); Serial.print("  ");
    Serial.println("m/s^2 ");
  }

  if (showGyroscope) {
    Serial.print("Gyroscope: ");    
    Serial.print("X: "); Serial.print(gyro_event.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print("  ");
    Serial.println("rad/s ");
  }

  if (showMagnetometer) { 
    Serial.print("Magnetometer: ");
    Serial.print("X: "); Serial.print(mag_event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(mag_event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(mag_event.magnetic.z); Serial.print("  ");
    Serial.println("uT");
  }
}

void setup(){
  delay(10000);
  
  // Setup the connection to the XBee
  Serial.begin(9600);

  // Connect to the ESCs as if they were servos
  ESC_RT.attach(ESCPin5_RT);
  ESC_LT.attach(ESCPin2_LT);
  ESC_TT.attach(ESCPin1_TT);
  ESC_RB.attach(ESCPin4_RB);
  ESC_LB.attach(ESCPin6_LB);
  ESC_TB.attach(ESCPin3_TB);

  Serial.print( "Initialize ESC... " );
  ESC_RT.write(PWMZero); 
  ESC_LT.write(PWMZero); 
  ESC_TT.write(PWMZero); 
  ESC_RB.write(PWMZero); 
  ESC_LB.write(PWMZero); 
  ESC_TB.write(PWMZero); 

  // Here we zero the ESCs and wait for 20 seconds. You must zero the ESCs to 
  // initialize them. You only need to wait 5 seconds for initialization, but
  // I wait longer, just to get out of the way of the Y6.
  // Also, change the switch in UART position
  for (int count = 20; count >= 1; count--){
    Serial.println(count);
    delay(1000);
  }
  
  // Initializing sensors
  initSensors();
  getSensorData();
  
  // Now we set the minimum PWM value before starting the test.
  
  Serial.println( "initialization complete... " );
  PWMValue = PWMMin;
  Serial.println( "Starting test... " );
}

boolean ESC_ON = false;

void ascend(){
//  increment = 5;
if (PWMValue < PWMMax) {
  PWMValue ++;
}
Serial.println("PWMValue = " + PWMValue);
}

void descend(){
//  increment = -5; 
if (PWMValue > PWMMin){
  PWMValue --;
}
Serial.println("PWMValue = " + PWMValue);
}

void level(){
// increment = 0; 
}

void startMission(){
  Serial.println("Mission started");
  ESC_ON = true;
  PWMValue = PWMMin;
//  increment = 0;
}

void abortMission(){
  Serial.println("Mission aborted");
  ESC_ON = false;
  PWMValue = PWMMin;
}

int count = 0;

void loop() {
  // We are going to use 2 char commands
  if (Serial.available()) {
    char command = Serial.read();
    switch (command){
      case 'u':
      case 'U': ascend();       break;
      case 'd': 
      case 'D': descend();      break;
      case 'a': 
      case 'A': abortMission(); break;
      case 's': 
      case 'S': startMission(); break;
      case 'z': 
      case 'Z': showAccelerometer = !showAccelerometer; break;
      case 'x': 
      case 'X': showGyroscope = !showGyroscope;         break;
      case 'c': 
      case 'C': showMagnetometer = !showMagnetometer;   break;
      default:                  break;
    }
  }
 if (ESC_ON) {
    ESC_RT.write(PWMValue + offset_RT);
    ESC_LT.write(PWMValue + offset_LT);
    ESC_TT.write(PWMValue + offset_TT);
    ESC_RB.write(PWMValue + offset_RB);
    ESC_LB.write(PWMValue + offset_LB);
    ESC_TB.write(PWMValue + offset_TB);
  }
  else {
    ESC_RT.write(PWMZero);
    ESC_LT.write(PWMZero);
    ESC_TT.write(PWMZero);
    ESC_RB.write(PWMZero);
    ESC_LB.write(PWMZero);
    ESC_TB.write(PWMZero);
  }
  getSensorData();
}
