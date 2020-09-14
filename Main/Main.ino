#include <SparkFun_ADXL345.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>

ADXL345 adxl = ADXL345();
int L3G4200D_Address = 105; //I2C address of the L3G4200D

// Setup state machine
enum STATE {INIT, PRE_OPERATION, OPERATE};

uint8_t state = INIT;
bool NextState = true;

// Setting up timers for setting the timing of the program
unsigned long previousMillis = 0;
const long interval = 50;

//Setting up parameters for PID controller
double Setpoint, Input, PIDOutput;
PID PIDKeepAngle(&Input, &PIDOutput, &Setpoint, 2, 1, 0, DIRECT); 

//Declaring motor controllers
Servo motorController;

// Declaring global angles (should they be global?)
double xAngle, yAngle;

//Declaring global raw data for accelerometer, (should they be global?)
double xGyro, yGyro, zGyro;


void setup() {
  Wire.begin();         //Start communication on I2C protocol
  Serial.begin(9600);   // Start communication to termial

  //Setting up IMU
  SetupIMU();     // Configure L3G4200  - 250, 500 or 2000 deg/sec
  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(8);

  //Setting up PID controller
  Input = 0;
  Setpoint = 0;
  PIDKeepAngle.SetMode(MANUAL);
  PIDKeepAngle.SetOutputLimits(-100, 100);
  PIDKeepAngle.SetSampleTime(50);

  //Setting up motor controller
  motorController.attach(9);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    CalculateAngles();
    RUN_STATE_MACHINE();
  }
  
}

void RUN_STATE_MACHINE(){
  switch(state)
  {
    case INIT:
      Serial.print("INIT");
      if (NextState == true){
        state = PRE_OPERATION;
      }
      break;
    
    case PRE_OPERATION:
      Serial.print("PRE_OPERATIONS");
      
      PIDKeepAngle.SetMode(AUTOMATIC);
      if (NextState == true){
        state = OPERATE;
      }
      break;
      
    case OPERATE:
         
      Input = xAngle;
      PIDKeepAngle.Compute();
      Serial.print(PIDOutput);
      setMotorSpeed();      
  }
}


//Function for getting the setpoint set by the 
void GetSetpoint(){
  Setpoint = 10;
}
  

void setMotorSpeed(){
  // This block contains all necessary code to control the speed of the motor, the speed is in %
  int motorSpeedServo;
  
  motorSpeedServo = map(PIDOutput, 0, 100, 0, 180);         //mapping to correspond to servo
  //motorController.write(motorSpeedServo);
  Serial.write(motorSpeedServo);
}





void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}
