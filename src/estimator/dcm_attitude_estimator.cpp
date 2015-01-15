#include "estimator/dcm_attitude_estimator.hpp"

#include "protocol/messages.hpp"

#include "unit_config.hpp"

DCMAttitudeEstimator::DCMAttitudeEstimator(Communicator& communicator)
  : attitudeMessageStream(communicator, 100) {
  dcm.setIdentity();
}

attitude_estimate_t DCMAttitudeEstimator::update(gyroscope_reading_t& gyroReading, accelerometer_reading_t& accelReading) {
  Eigen::Vector3f gyro(gyroReading.axes.data());
  Eigen::Vector3f accel(accelReading.axes.data());

  accel.normalize();

  Eigen::Vector3f corr(0.0f, 0.0f, 0.0f);
  corr -= gyro * unit_config::DT;
  corr -= dcm.col(2).cross(accel);

  Eigen::Matrix3f dcmStep;
  dcmStep <<      1.0f, -corr.z(), -corr.y(),
             -corr.z(),      1.0f,  corr.x(),
              corr.y(), -corr.x(),      1.0f;

  dcm *= dcmStep;

  orthonormalize();

  attitude_estimate_t estimate {
    // TODO: Are these trig functions safe at extreme angles?
    .roll = atan2f(dcm(2, 1), dcm(2, 2)),
    .pitch = -asinf(dcm(2, 0)),
    .yaw = atan2f(dcm(1, 0), dcm(0, 0)),
    .roll_vel = gyro.x(),
    .pitch_vel = gyro.y(),
    .yaw_vel = gyro.z()
  };

  if(attitudeMessageStream.ready()) {
    protocol::message::attitude_message_t m {
      .roll = estimate.roll,
      .pitch = estimate.pitch,
      .yaw = estimate.yaw
    };

    attitudeMessageStream.publish(m);
  }

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
