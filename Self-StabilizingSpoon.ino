#include <Wire.h>
#include <Kalman.h>
#include <PID_v1.h>
#include <Servo.h>

#define IMUAddress 0x68
#define I2C_TIMEOUT 1000
#define ROLL_PITCH

Kalman kalmanX, kalmanY;
Servo servo1, servo2;

double accX, accY, accZ, gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;
double SetpointX = 90, InputX, OutputX;
double SetpointY = 90, InputY, OutputY;
double Kp = 1, Ki = 0, Kd = 0;

PID PIDX(&InputX, &OutputX, &SetpointX, Kp, Ki, Kd, DIRECT);
PID PIDY(&InputY, &OutputY, &SetpointY, Kp, Ki, Kd, DIRECT);

uint32_t timer;
uint8_t i2cData[14];

void setup() {
  initializeSerial();
  initializeWire();
  initializeServos();
  initializeSensors();
  initializeKalmanFilter();
  initializePIDControllers();
}

void loop() {
  updateSensorReadings();
  computePIDControllers();
  updateServos();
  printSerialData();
}

void initializeSerial() {
  Serial.begin(115200);
}

void initializeWire() {
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL);
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2;
  #endif
}

void initializeServos() {
  servo1.attach(2);
  servo2.attach(3);
}

void initializeSensors() {
  // Initialization code for sensors, e.g., MPU6050 setup
  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;  // Set Gyro Full Scale Range to 250deg/s
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to 2g
  while (i2cWrite(0x19, i2cData, 4, false))
    ;  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ;  // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100);  // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  #ifdef ROLL_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

}

void initializeKalmanFilter() {
  // Set initial angle for Kalman filter using sensor data
  kalmanX.setAngle(roll); 
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();

  servo1.write(0);
  servo2.write(0);
}

void initializePIDControllers() {
  SetpointX = SetpointY = 90;
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetSampleTime(10);
  PIDY.SetSampleTime(10);
}

void updateSensorReadings() {
  mpuKalman();

  // Read sensor data and update angles using Kalman filter
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  InputX = kalAngleX;
  InputY = kalAngleY;

  if (InputX < -90.0) {
    InputX = -90.0;
  } else if (InputX > 90.0) {
    InputX = 90.0;
  }

  if (InputY < -90.0) {
    InputY = -90.0;
  } else if (InputY > 90.0) {
    InputY = 90.0;
  }

  //Compute the Output
  PIDX.Compute();
  PIDY.Compute();

  servo1.write(OutputY);
  servo2.write(OutputX);

  Serial.print(InputX);
  Serial.print("\t");
  Serial.print(InputY);
  Serial.print("\t");
  Serial.print(SetpointY);
  Serial.print("\t");
  Serial.print(OutputX);
  Serial.print("\t");
  Serial.println(OutputY);
  Serial.print("\t");
}

void computePIDControllers() {
  constrainInputAngles();
  PIDX.Compute();
  PIDY.Compute();
}

void updateServos() {
  servo1.write(OutputY);
  servo2.write(OutputX);
}

void printSerialData() {
  Serial.print(InputX);
  Serial.print("\t");
  Serial.print(InputY);
  Serial.print("\t");
  Serial.print(SetpointY);
  Serial.print("\t");
  Serial.print(OutputX);
  Serial.print("\t");
  Serial.println(OutputY);
}

void constrainInputAngles() {
  InputX = constrain(kalAngleX, -90.0, 90.0);
  InputY = constrain(kalAngleY, -90.0, 90.0);
}

void mpuKalman() { 
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  #ifdef ROLL_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90) {
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

}

// Read and Write To I2C Module
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop);  // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t) true);
    for (uint8_t i = 0; i < nbytes; i++) {
      if (Wire.available())
        data[i] = Wire.read();
      else {
        timeOutTimer = micros();
        while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available())
          ;
        if (Wire.available())
          data[i] = Wire.read();
        else {
          Serial.println(F("i2cRead timeout"));
          return 5;
        }
      }
      return 0;  // Success
    }