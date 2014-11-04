#include <estimator/dcm_attitude_estimator.hpp>

#include <hal_config.hpp>

DCMAttitudeEstimator::DCMAttitudeEstimator() {
  dcm.setIdentity();
}

attitude_estimate_t DCMAttitudeEstimator::update(accelerometer_reading_t& accel_reading, gyroscope_reading_t& gyro_reading) {
  Eigen::Vector3f accel(accel_reading.axes);
  Eigen::Vector3f gyro(gyro_reading.axes);

  accel.normalize();

  Eigen::Vector3f corr(0.0f, 0.0f, 0.0f);
  corr += gyro * DT;
  corr -= dcm.col(2).cross(accel);

  Eigen::Matrix3f dcmStep;
  dcmStep <<      1.0f, -corr.z(), -corr.y(),
             -corr.z(),      1.0f,  corr.x(),
              corr.y(), -corr.x(),      1.0f;

  dcm *= dcmStep;

  orthonormalize();

  attitude_estimate_t estimate = {
    // TODO: Are these trig functions safe at extreme angles?
    .pitch = -asinf(dcm(2, 0)),
    .roll = atan2f(dcm(2, 1), dcm(2, 2)),
    .yaw = atan2f(dcm(1, 0), dcm(0, 0)),
    .pitch_vel = gyro.y(),
    .roll_vel = gyro.x(),
    .yaw_vel = gyro.z()
  };

  return estimate;
}

// TODO(yoos): HACK. This is a copy of the above, only that it accepts imu_reading_t.
attitude_estimate_t DCMAttitudeEstimator::update(imu_reading_t& imu_reading) {
  Eigen::Vector3f accel(imu_reading.acc);
  Eigen::Vector3f gyro(imu_reading.gyr);

  accel.normalize();

  Eigen::Vector3f corr(0.0f, 0.0f, 0.0f);
  corr += gyro * DT;
  corr -= dcm.col(2).cross(accel);

  Eigen::Matrix3f dcmStep;
  dcmStep <<      1.0f, -corr.z(), -corr.y(),
             -corr.z(),      1.0f,  corr.x(),
              corr.y(), -corr.x(),      1.0f;

  dcm *= dcmStep;

  orthonormalize();

  attitude_estimate_t estimate = {
    // TODO: Are these trig functions safe at extreme angles?
    .pitch = -asinf(dcm(2, 0)),
    .roll = atan2f(dcm(2, 1), dcm(2, 2)),
    .yaw = atan2f(dcm(1, 0), dcm(0, 0)),
    .pitch_vel = gyro.y(),
    .roll_vel = gyro.x(),
    .yaw_vel = gyro.z()
  };

  return estimate;
}

void DCMAttitudeEstimator::orthonormalize() {
  // Make the i and j vectors orthogonal
  float error = dcm.row(0).dot(dcm.row(1));

  Eigen::Matrix3f corr;
  corr.row(0) = dcm.row(1) * -error / 2;
  corr.row(1) = dcm.row(0) * -error / 2;

  dcm += corr;

  // Estimate k vector from corrected i and j vectors
  dcm.row(2) = dcm.row(0).cross(dcm.row(1));

  // Normalize all vectors
  dcm.rowwise().normalize();
}
