#include "drivers/mpu9250.hpp"

void MPU9250::init() {
  // Reset device.
  txbuf[0] = mpu9250::PWR_MGMT_1;
  txbuf[1] = 0x80;
  exchange(2);

  chThdSleepMilliseconds(100);   // TODO(yoos): Check whether or not datasheet specifies this holdoff.

  // Set sample rate to 1 kHz.
  txbuf[0] = mpu9250::SMPLRT_DIV;
  txbuf[1] = 0x00;
  exchange(2);

  // Set DLPF to 4 (20 Hz gyro bandwidth1 Hz accelerometer bandwidth)
  txbuf[0] = mpu9250::CONFIG;
  txbuf[1] = 0x04;
  exchange(2);

  // Wake up device and set clock source to Gyro Z.
  txbuf[0] = mpu9250::PWR_MGMT_1;
  txbuf[1] = 0x03;
  exchange(2);

  // Set gyro full range to 2000 dps. We first read in the current register
  // value so we can change what we need and leave everything else alone.
  // See DS p. 12.
  txbuf[0] = mpu9250::GYRO_CONFIG | (1<<7);
  exchange(2);
  chThdSleepMicroseconds(0);   // TODO(yoos): Without this, the GYRO_CONFIG register does not get set. This was not the case in the old C firmware. Why?
  txbuf[0] = mpu9250::GYRO_CONFIG;
  txbuf[1] = (rxbuf[1] & ~0x18) | 0x18;
  exchange(2);

  // Set accelerometer full range to 4 g. See DS p. 13.
  txbuf[0] = mpu9250::ACCEL_CONFIG | (1<<7);
  exchange(2);
  txbuf[0] = mpu9250::ACCEL_CONFIG;
  txbuf[1] = (rxbuf[1] & ~0x18) | 0x08;
  exchange(2);

  // Read once to clear out bad data?
  readGyro();
  readAccel();
}

GyroscopeReading MPU9250::readGyro() {
  GyroscopeReading reading;

  // Poll gyro
  txbuf[0] = mpu9250::GYRO_XOUT_H | (1<<7);
  exchange(7);

  // TODO(yoos): correct for thermal bias.
  reading.axes[Gyroscope::axes[0]] = Gyroscope::signs[0] * ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 16.384 * 3.1415926535 / 180.0 - Gyroscope::offsets[0];
  reading.axes[Gyroscope::axes[1]] = Gyroscope::signs[1] * ((int16_t) ((rxbuf[3]<<8) | rxbuf[4])) / 16.384 * 3.1415926535 / 180.0 - Gyroscope::offsets[1];
  reading.axes[Gyroscope::axes[2]] = Gyroscope::signs[2] * ((int16_t) ((rxbuf[5]<<8) | rxbuf[6])) / 16.384 * 3.1415926535 / 180.0 - Gyroscope::offsets[2];

  // Poll temp
  txbuf[0] = mpu9250::TEMP_OUT_H | (1<<7);
  exchange(3);

  float temp = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 340 + 36.53;

  // DEBUG
  //char dbg_buf[16];
  //for (int i=0; i<16; i++) {
  //  dbg_buf[i] = 0;
  //}
  //chsnprintf(dbg_buf, 12, "%0.6f\r\n", reading.axes[2]);
  //chnWriteTimeout((BaseChannel*)&SD3, (uint8_t*)dbg_buf, 12, MS2ST(20));

  return reading;
}

AccelerometerReading MPU9250::readAccel() {
  // Get data
  txbuf[0] = mpu9250::ACCEL_XOUT_H | (1<<7);
  exchange(7);

  AccelerometerReading meas;
  meas.axes[Accelerometer::axes[0]] = Accelerometer::signs[0] * ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 8192.0 - Accelerometer::offsets[0];
  meas.axes[Accelerometer::axes[1]] = Accelerometer::signs[1] * ((int16_t) ((rxbuf[3]<<8) | rxbuf[4])) / 8192.0 - Accelerometer::offsets[1];
  meas.axes[Accelerometer::axes[2]] = Accelerometer::signs[2] * ((int16_t) ((rxbuf[5]<<8) | rxbuf[6])) / 8192.0 - Accelerometer::offsets[2];

  // Low-pass filter
  const float ALPHA = 0.5;   // TODO(yoos): make configurable
  static AccelerometerReading reading {
    .axes = {0,0,0}
  };
  for (int i=0; i<3; i++) {
    reading.axes[Accelerometer::axes[i]] = ALPHA * meas.axes[Accelerometer::axes[i]] + (1-ALPHA) * reading.axes[Accelerometer::axes[i]];
  }

  return reading;
}

MagnetometerReading MPU9250::readMag() {
  MagnetometerReading reading;

  // Get data
  // TODO(yoos)

  return reading;
}

bool MPU9250::healthy() {
  // Check WHO_AM_I
  txbuf[0] = mpu9250::WHO_AM_I | (1<<7);
  txbuf[1] = 0x00;
  exchange(2);

  if (rxbuf[1] == 0x71) {
    return true;
  }

  // TODO(yoos): check gyr/acc self-test registers

  return false;
}
