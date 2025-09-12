#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// --- IMU ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- Madgwick variables ---
float beta = 0.1;       // 2 * proportional gain
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion state
float sampleFreq = 100.0f;  // 100 Hz

unsigned long microsPerReading, microsPrevious;

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("No BNO055 detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  microsPerReading = 1000000 / sampleFreq;
  microsPrevious = micros();

  Serial.println("Roll,Pitch,Yaw");
}

// --- Safe normalization function ---
bool safeNormalize(float &x, float &y, float &z) {
  float norm = sqrtf(x * x + y * y + z * z);
  if (norm < 1e-6f) return false;  // avoid division by zero
  x /= norm; y /= norm; z /= norm;
  return true;
}

// --- Madgwick algorithm update ---
void MadgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx;
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;

  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  // --- Normalize accelerometer ---
  if (!safeNormalize(ax, ay, az)) return;

  // --- Normalize magnetometer ---
  bool useMag = safeNormalize(mx, my, mz);

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (useMag) {
    // Reference direction of Earth’s magnetic field
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 +
         mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 -
         mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 +
         _2q1mx * q2 - my * q1q1 + my * q2q2 +
         _2q2 * mz * q3 - my * q3q3;
    float _2bx = sqrtf(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q2 + _2q0my * q1 +
                 mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
                 _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0f * _2bx;
    float _4bz = 2.0f * _2bz;

    // Gradient descent algorithm corrective step
    s0 = -_2q2 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q1 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) +
         _2bz * (q1 * q3 - q0 * q2) - mx) +
         (-_2bx * q3 + _2bz * q1) *
         (_2bx * (q1 * q2 - q0 * q3) +
         _2bz * (q0 * q1 + q2 * q3) - my) +
         _2bx * q2 *
         (_2bx * (q0 * q2 + q1 * q3) +
         _2bz * (0.5f - q1q1 - q2q2) - mz);

    s1 = _2q3 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q0 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) +
         _2bz * (q1 * q3 - q0 * q2) - mx) +
         (_2bx * q2 + _2bz * q0) *
         (_2bx * (q1 * q2 - q0 * q3) +
         _2bz * (q0 * q1 + q2 * q3) - my) +
         (_2bx * q3 - _4bz * q1) *
         (_2bx * (q0 * q2 + q1 * q3) +
         _2bz * (0.5f - q1q1 - q2q2) - mz);

    s2 = -_2q0 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q3 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) *
         (_2bx * (0.5f - q2q2 - q3q3) +
         _2bz * (q1 * q3 - q0 * q2) - mx) +
         (_2bx * q1 + _2bz * q3) *
         (_2bx * (q1 * q2 - q0 * q3) +
         _2bz * (q0 * q1 + q2 * q3) - my) +
         (_2bx * q0 - _4bz * q2) *
         (_2bx * (q0 * q2 + q1 * q3) +
         _2bz * (0.5f - q1q1 - q2q2) - mz);

    s3 = _2q1 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q2 * (2.0f * (q0 * q1 + q2 * q3) - ay) +
         (-_4bx * q3 + _2bz * q1) *
         (_2bx * (0.5f - q2q2 - q3q3) +
         _2bz * (q1 * q3 - q0 * q2) - mx) +
         (-_2bx * q0 + _2bz * q2) *
         (_2bx * (q1 * q2 - q0 * q3) +
         _2bz * (q0 * q1 + q2 * q3) - my) +
         _2bx * q1 *
         (_2bx * (q0 * q2 + q1 * q3) +
         _2bz * (0.5f - q1q1 - q2q2) - mz);
  } else {
    // IMU-only fallback (gyro + accel)
    s0 = -_2q2 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q1 * (2.0f * (q0 * q1 + q2 * q3) - ay);
    s1 = _2q3 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q0 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s2 = -_2q0 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q3 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s3 = _2q1 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
         _2q2 * (2.0f * (q0 * q1 + q2 * q3) - ay);
  }

  // Normalise step magnitude
  recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

  // Apply feedback step
  qDot1 -= beta * s0;
  qDot2 -= beta * s1;
  qDot3 -= beta * s2;
  qDot4 -= beta * s3;

  // Integrate rate of change of quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void loop() {
  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Gyro in radians/sec
    float gx = gyro.x() * (M_PI / 180.0f);
    float gy = gyro.y() * (M_PI / 180.0f);
    float gz = gyro.z() * (M_PI / 180.0f);

    // Call Madgwick update
    MadgwickUpdate(gx, gy, gz,
                   accel.x(), accel.y(), accel.z(),
                   mag.x(), mag.y(), mag.z());

    // Convert quaternion to roll/pitch/yaw (degrees)
    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

    Serial.print(roll); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.println(yaw);

    microsPrevious = microsNow;
  }
}
