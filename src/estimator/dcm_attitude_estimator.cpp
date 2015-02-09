#include "estimator/dcm_attitude_estimator.hpp"

#include "protocol/messages.hpp"

#include "unit_config.hpp"

DCMAttitudeEstimator::DCMAttitudeEstimator(Communicator& communicator)
  : dcm(Eigen::Matrix3f::Identity()),
    attitudeMessageStream(communicator, 20) {
}

attitude_estimate_t DCMAttitudeEstimator::update(const sensor_reading_group_t& readings) {
  Eigen::Vector3f corr = Eigen::Vector3f::Zero();

  float accelWeight = 0.0f;
  float magWeight = 0.0f;

  // If an accelerometer is available, use the provided gravity vector to
  // correct for drift in the DCM.
  if(readings.accel) {
    Eigen::Vector3f accel((*readings.accel).axes.data());

    // Calculate accelerometer weight before normalization
    accelWeight = getAccelWeight(accel);

    accel.normalize();
    corr += dcm.col(2).cross(-accel) * accelWeight;
  }

  // If a magnetometer is available, use the provided north vector to correct
  // the yaw.
  if(readings.mag) {
    Eigen::Vector3f mag((*readings.mag).axes.data());

    // TODO: Calculate mag weight?
    magWeight = 0.001f;

    // "Project" the magnetometer into the global xy plane.
    Eigen::Vector3f magGlobal = dcm.transpose() * mag;
    magGlobal(2) = 0.0f;

    // Convert back to body coordinates.
    Eigen::Vector3f magBody = dcm * magGlobal;

    // Normalize because we only care about the prevailing direction.
    magBody.normalize();

    corr += dcm.col(0).cross(magBody - dcm.col(0)) * magWeight;
  }

  // If a gyroscope is available, integrate the provided rotational velocity and
  // add it to the correction vector.
  if(readings.gyro) {
    Eigen::Vector3f gyro((*readings.gyro).axes.data());
    corr += gyro * unit_config::DT * (1.0f - accelWeight - magWeight);
  }

  // Use small angle approximations to build a rotation matrix modelling the
  // attitude change over the time step.
  Eigen::Matrix3f dcmStep;
  dcmStep <<      1.0f,  corr.z(), -corr.y(),
             -corr.z(),      1.0f,  corr.x(),
              corr.y(), -corr.x(),      1.0f;

  // Rotate the DCM.
  dcm = dcmStep * dcm;

  orthonormalize();

  updateStream();

  return makeEstimate(readings);
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

float DCMAttitudeEstimator::getAccelWeight(Eigen::Vector3f accel) const {
  // TODO(kyle): Pull these out as parameters
  float maxAccelWeight = 0.005f; // Accelerometer weight at exactly 1g
  float validAccelRange = 0.5f; // Maximum additional acceleration until accelWeight goes to 0

  // Deweight accelerometer as a linear function of the reading's difference
  // from 1g.
  float accelOffset = std::abs(1.0f - accel.norm());
  float accelWeight = -maxAccelWeight / validAccelRange * accelOffset + maxAccelWeight;

  // Limit weight to a minimum of 0
  accelWeight = std::max(0.0f, accelWeight);

  return accelWeight;
}

attitude_estimate_t DCMAttitudeEstimator::makeEstimate(const sensor_reading_group_t& readings) {
  attitude_estimate_t estimate = {
    // TODO: Are these trig functions safe at extreme angles?
    .roll = -atan2f(dcm(2, 1), dcm(2, 2)) * dcm(0, 0) + atan2f(dcm(2, 0), dcm(2, 2)) * dcm(0, 1),
    .pitch = atan2f(dcm(2, 0), dcm(2, 2)) * dcm(1, 1) - atan2f(dcm(2, 1), dcm(2, 2)) * dcm(1, 0),
    .yaw = 0.0f, // atan2f(dcm(1, 1), dcm(0, 1)),
    .roll_vel = 0.0f,
    .pitch_vel = 0.0f,
    .yaw_vel = 0.0f
  };

  // If a gyro is available then use the direct readings for velocity
  // calculation.
  if(readings.gyro) {
    estimate.roll_vel = (*readings.gyro).axes[0];
    estimate.pitch_vel = (*readings.gyro).axes[1];
    estimate.yaw_vel = (*readings.gyro).axes[2];
  } else {
    // TODO: Differentiate estimates or just ignore?
  }

  return estimate;
}

void DCMAttitudeEstimator::updateStream() {
  if(attitudeMessageStream.ready()) {
    protocol::message::attitude_message_t m {
      .dcm = {
        dcm(0, 0), dcm(0, 1), dcm(0, 2),
        dcm(1, 0), dcm(1, 1), dcm(1, 2),
        dcm(2, 0), dcm(2, 1), dcm(2, 2)
      }
    };

    attitudeMessageStream.publish(m);
  }
}
