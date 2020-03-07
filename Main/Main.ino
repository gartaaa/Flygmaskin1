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
  setupL3G4200D();     // Configure L3G4200  - 250, 500 or 2000 deg/sec
  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(8);

  //Setting up PID controller
  Input = 0;
  Setpoint = 0;
  PIDKeepAngle.SetMode(AUTOMATIC);
  PIDKeepAngle.SetOutputLimits(-100, 100);
  PIDKeepAngle.SetSampleTime(50);

  //Setting up motor controller
  motorController.attach(9);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval){
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
         
      Setpoint = 10; // move to a separate function, setpint should be recieved from rc controller
      Input = xAngle;
      PIDKeepAngle.Compute();
      Serial.print(PIDOutput);
      setMotorSpeed();      
  }
}

void setMotorSpeed(){
  // This block contains all necessary code to control the speed of the motor, the speed is in %
  int motorSpeedServo;
  
  motorSpeedServo = map(PIDOutput, 0, 100, 0, 180);         //mapping to correspond to servo
  motorController.write(motorSpeedServo);
}

void CalculateAngles(){
  
  // Declare local variables
  double Roll, Pitch;
  double x, y; //temp variables
  int xAccelerometer, yAccelerometer, zAccelerometer;
  
  getGyroValues();  // This will update xGyro, yGyro, and zGyro with new values
  adxl.readAccel(&xAccelerometer, &yAccelerometer, &zAccelerometer);

  Roll = (atan2(-yAccelerometer, zAccelerometer)*180.0)/M_PI;
  Pitch = (atan2(xAccelerometer, sqrt(yAccelerometer*yAccelerometer + zAccelerometer*zAccelerometer))*180.0)/M_PI;
  x = xGyro/14.286;
  y = yGyro/14.286;
  xAngle = 0.95*(xAngle + x * 0.05) - 0.05 * Roll;
  yAngle = 0.95*(yAngle + y * 0.05) - 0.05 * Pitch;
  
}


void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  xGyro = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  yGyro = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  zGyro = ((zMSB << 8) | zLSB);
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}  

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

void setupL3G4200D(){

  #define CTRL_REG1 0x20
  #define CTRL_REG2 0x21
  #define CTRL_REG3 0x22
  #define CTRL_REG4 0x23
  #define CTRL_REG5 0x24
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}
