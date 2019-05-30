#include "MPU9250.h"

const int brkPin = 3;
const int statePin = 2;
const int MPU9250_ADDRESS = MPU9250_ADDRESS_AD0;

MPU9250 myIMU(MPU9250_ADDRESS, Wire, 400000);

void setup()
{
  // Startup serial and I2C
  Wire.begin();
  Serial.begin(9600);

  // Setup pins.
  pinMode(brkPin, OUTPUT);
  digitalWrite(brkPin, HIGH);

  pinMode(statePin, INPUT);

  // Wait until serial is ready
  while(!Serial){};

  // Read the WHO_AM_I register
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c != 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("ERR: BAD MPU9250 COMMS"));
    Serial.flush();
    abort();
  }
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();

  byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  if (d != 0x48)
  {
    Serial.println(F("ERR: BAD MAG COMMS"));
    Serial.flush();
    abort();
  }

  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.factoryMagCalibration);

  // Get sensor resolutions, only need to do this once
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();

}

void loop()
{
  // Check if data ready. If so, read new data.
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values


    // Now we'll calculate the accleration value into actual g's
    // Use current scale to convert.
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // Use currently set resolution.
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

    // Read the current temperature
    myIMU.tempCount = myIMU.readTempData();

    // Temperature in degrees Celsius
    myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
  }
  myIMU.updateTime();

  myIMU.delt_t = millis() - myIMU.count;
  
  if ((myIMU.delt_t > 200) && digitalRead(statePin) == HIGH)
  { 
    // Print acceleration values in milli-gs!
    Serial.print("ACCEL:");
    Serial.print(9.8 * myIMU.ax, 3); Serial.print(',');
    Serial.print(9.8 * myIMU.ay, 3); Serial.print(',');
    Serial.println(9.8 * myIMU.az, 3);

    // Print gyro values in degree/sec
    Serial.print("GYRO:");
    Serial.print(myIMU.gx, 3); Serial.print(',');
    Serial.print(myIMU.gy, 3); Serial.print(',');
    Serial.println(myIMU.gz, 3);

    // Print mag values in degree/sec
    Serial.print("MAG:");
    Serial.print(myIMU.mx); Serial.print(',');
    Serial.print(myIMU.my); Serial.print(',');
    Serial.println(myIMU.mz);

    // Print temperature in degrees Centigrade
    Serial.print("TEMP:");  Serial.println(myIMU.temperature, 1);

    myIMU.count = millis();
  }
}
