#include "estimator/dcm_attitude_estimator.hpp"

#include "protocol/messages.hpp"

#include "unit_config.hpp"

DCMAttitudeEstimator::DCMAttitudeEstimator(Communicator& communicator)
  : dcm(Eigen::Matrix3f::Identity()),
    attitudeMessageStream(communicator, 20) {
}

AttitudeEstimate DCMAttitudeEstimator::update(const SensorReadingGroup& readings) {
  Eigen::Vector3f corr = Eigen::Vector3f::Zero();

  // Assume zero weights until we have verified the presence of these sensors.
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
  // Orthogonalize i and j vectors
  //
  // We assume small angles. We calculate the magnitude of error as the
  // projection of i on j and generate correction vectors i' and j' with
  // magnitude error/2 and direction opposite j and i, respectively. Then we
  // update i and j:
  //
  //   i = i + i'
  //   j = j + j'
  //
  float error = dcm.row(0).dot(dcm.row(1));   // Magnitude of error
  Eigen::Matrix3f corr = Eigen::Matrix3f::Zero();   // Generate correction vectors
  corr.row(0) = (-error / 2) * dcm.row(1);   // i'
  corr.row(1) = (-error / 2) * dcm.row(0);   // j'

  dcm.row(0) += corr.row(0);   // i = i + i'
  dcm.row(1) += corr.row(1);   // j = j + j'

  // Regenerate k from the corrected i and j and normalize the DCM.
  dcm.row(2) = dcm.row(0).cross(dcm.row(1));   // k = i x j
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

AttitudeEstimate DCMAttitudeEstimator::makeEstimate(const SensorReadingGroup& readings) {
  AttitudeEstimate estimate = {
    // TODO: Are these trig functions safe at extreme angles?
    .roll = -atan2f(dcm(2, 1), dcm(2, 2)) * dcm(0, 0) + atan2f(dcm(2, 0), dcm(2, 2)) * dcm(0, 1),
    .pitch = atan2f(dcm(2, 0), dcm(2, 2)) * dcm(1, 1) - atan2f(dcm(2, 1), dcm(2, 2)) * dcm(1, 0),
    .yaw = 0.0f, // atan2f(dcm(1, 1), dcm(0, 1)),

    // Velocities are set later if a gyro is available.
    .rollVel = 0.0f,
    .pitchVel = 0.0f,
    .yawVel = 0.0f,

    // TODO: Calculate accelerations.
    .rollAcc = 0.0f,
    .pitchAcc = 0.0f,
    .yawAcc = 0.0f
  };

  // If a gyro is available then use the direct readings for velocity
  // calculation.
  if(readings.gyro) {
    estimate.rollVel = (*readings.gyro).axes[0];
    estimate.pitchVel = (*readings.gyro).axes[1];
    estimate.yawVel = (*readings.gyro).axes[2];
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
