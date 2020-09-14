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
