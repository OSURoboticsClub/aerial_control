#include <estimator/dcm_attitude_estimator.hpp>

#include <config.hpp>

DCMAttitudeEstimator::DCMAttitudeEstimator() {
  dcm.setIdentity();
}

struct attitude_estimate_t DCMAttitudeEstimator::update(struct accelerometer_reading_t& accel_reading, struct gyroscope_reading_t& gyro_reading) {
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

  struct attitude_estimate_t estimate = {
    // TODO: Are these trig functions safe at extreme angles?
    .roll = atan2f(dcm(2, 1), dcm(2, 2)) * 180 / M_PI,
    .pitch = -asinf(dcm(2, 0)) * 180 / M_PI,
    .yaw = atan2f(dcm(1, 0), dcm(0, 0)) * 180 / M_PI,
    .roll_vel = gyro.x(),
    .pitch_vel = gyro.y(),
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
