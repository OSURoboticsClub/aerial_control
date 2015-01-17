#include "estimator/dcm_attitude_estimator.hpp"

#include "hal.h"
#include "protocol/messages.hpp"

#include "unit_config.hpp"

DCMAttitudeEstimator::DCMAttitudeEstimator(Communicator& communicator)
  : dcm(Eigen::Matrix3f::Identity()),
    attitudeMessageStream(communicator, 100) {
}

attitude_estimate_t DCMAttitudeEstimator::update(gyroscope_reading_t& gyroReading, accelerometer_reading_t& accelReading) {
  Eigen::Vector3f gyro(gyroReading.axes.data());
  Eigen::Vector3f accel(accelReading.axes.data());

  float accWeight = 0.02f;
  palClearPad(GPIOE, GPIOE_LED4_BLUE);
  if(accel.norm() > 1.05 * 9.8 || accel.norm() < 0.95 * 9.8) {
    palSetPad(GPIOE, GPIOE_LED4_BLUE);
    accWeight = 0.0f;
  }

  accel.normalize();

  Eigen::Vector3f corr = Eigen::Vector3f::Zero();
  corr += gyro * unit_config::DT * (1.0f - accWeight);
  corr += dcm.col(2).cross(accel) * accWeight;

  Eigen::Matrix3f dcmStep;
  dcmStep <<      1.0f,  corr.z(), -corr.y(),
             -corr.z(),      1.0f,  corr.x(),
              corr.y(), -corr.x(),      1.0f;

  dcm *= dcmStep;

  orthonormalize();

  attitude_estimate_t estimate {
    // TODO: Are these trig functions safe at extreme angles?
    .roll = -atan2f(dcm(2, 1), dcm(2, 2)) * dcm(0, 0) + atan2f(dcm(2, 0), dcm(2, 2)) * dcm(0, 1),
    .pitch = atan2f(dcm(2, 0), dcm(2, 2)) * dcm(1, 1) - atan2f(dcm(2, 1), dcm(2, 2)) * dcm(1, 0),
    .yaw = 0.0f, // atan2f(dcm(1, 1), dcm(0, 1)),
    .roll_vel = gyro.x(),
    .pitch_vel = gyro.y(),
    .yaw_vel = gyro.z()
  };

  if(attitudeMessageStream.ready()) {
    protocol::message::attitude_message_t m {
      .dcm = {
        // estimate.roll, estimate.pitch, estimate.yaw,
        // accel(0), accel(1), accel(2),
        // gyro(0), gyro(1), gyro(2),
        dcm(0, 0), dcm(0, 1), dcm(0, 2),
        dcm(1, 0), dcm(1, 1), dcm(1, 2),
        dcm(2, 0), dcm(2, 1), dcm(2, 2)
      }
    };

    attitudeMessageStream.publish(m);
  }

  return estimate;
}

void DCMAttitudeEstimator::orthonormalize() {
  // Make the i and j vectors orthogonal
  float error = dcm.row(0).dot(dcm.row(1));

  Eigen::Matrix3f corr = Eigen::Matrix3f::Zero();
  corr.row(0) = dcm.row(1) * (-error) / 2;
  corr.row(1) = dcm.row(0) * (-error) / 2;

  dcm.row(0) += corr.row(0);
  dcm.row(1) += corr.row(1);

  // Estimate k vector from corrected i and j vectors
  dcm.row(2) = dcm.row(0).cross(dcm.row(1));

  // Normalize all vectors
  dcm.rowwise().normalize();
}
