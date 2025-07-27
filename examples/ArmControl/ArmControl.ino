#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Servo.h>

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // yaw/pitch/roll container

Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

const int pinYaw = 23;    // adjust to your ESP32 pinout
const int pinPitch = 22;
const int pinRoll = 21;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // initialize device
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();

  // supply your own offsets here. these values are just examples
  mpu.setXAccelOffset(-1680);
  mpu.setYAccelOffset(-824);
  mpu.setZAccelOffset(1798);
  mpu.setXGyroOffset(39);
  mpu.setYGyroOffset(-19);
  mpu.setZGyroOffset(2);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // get expected packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  servoYaw.attach(pinYaw);
  servoPitch.attach(pinPitch);
  servoRoll.attach(pinRoll);
}

void loop() {
  if (!dmpReady) return;

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) return;  // not enough data yet

  if (fifoCount >= 1024) {
    // overflow, so reset
    mpu.resetFIFO();
    return;
  }

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180/M_PI;
  float pitch = ypr[1] * 180/M_PI;
  float roll  = ypr[2] * 180/M_PI;

  // map motion to servo ranges (0-180 degrees)
  servoYaw.write(map(yaw, -90, 90, 0, 180));
  servoPitch.write(map(pitch, -90, 90, 0, 180));
  servoRoll.write(map(roll, -90, 90, 0, 180));

  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.println(roll);

  delay(10);
}
