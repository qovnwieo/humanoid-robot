# ArmControl Example

This sketch demonstrates how to read orientation from an MPU6050 using its
Digital Motion Processor (DMP) and map the resulting angles to servo motors.
The ESP32 reads yaw, pitch and roll of the upper arm so a humanoid robot's
shoulder joints can follow the motion.

## Orientation math

The DMP outputs a quaternion representing the sensor orientation. The library
function `dmpGetYawPitchRoll()` converts this quaternion to Euler angles using
standard relations:

```
roll  = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
pitch = asin(2*(q.w*q.y - q.z*q.x))
yaw   = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
```

Angles are reported in radians; the sketch converts them to degrees before
sending them to the servo objects. Mapping from sensor range to servo range is
done with `map()` so that ±90° motion corresponds to the full servo travel.

Calibration offsets must be adapted to your specific sensor for best accuracy.
This example assumes the MPU6050 is mounted on the upper arm and the servos
control the robot shoulder's yaw, pitch and roll axes.
