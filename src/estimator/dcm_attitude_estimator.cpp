#include "estimator/dcm_attitude_estimator.hpp"

#include "ch.hpp"
#include "chprintf.h"

#include "global_parameters.hpp"
#include "protocol/messages.hpp"
#include "util/time.hpp"

DCMAttitudeEstimator::DCMAttitudeEstimator(ParameterRepository& params, Communicator& communicator, Logger& logger)
  : params(params),
    dcm(Eigen::Matrix3f::Identity()),
    attitudeMessageStream(communicator, 10),
    logger(logger),
    // This estimator treats the acceleration vector as purely the negative
    // gravity vector because we lack a feed-forward model of any of our
    // actuators and the architecture to include one in the control loop. With
    // a perfect gyro, we should never see the gravity vector move, so we set
    // the acceleration cutoff frequency very low until feed-forward models can
    // be incorporated into the estimators. TODO(kylc,syoo)
    accelFilter(5.0), gyroFilter(100.0) {
}

AttitudeEstimate DCMAttitudeEstimator::update(const SensorMeasurements& meas) {
  Eigen::Vector3f corr = Eigen::Vector3f::Zero();

  // Assume zero weights until we have verified the presence of these sensors.
  float accelWeight = 0.0f;
  float magWeight = 0.0f;

  // If an accelerometer is available, use the provided gravity vector to
  // correct for drift in the DCM.
  if(meas.accel) {
    //static Eigen::Vector3f accel({0,0,0});
    Eigen::Vector3f accel((*meas.accel).axes.data());

    // Calculate accelerometer weight before normalization
    accelWeight = getAccelWeight(accel);

    accel.normalize();
    corr += accelFilter.apply(params.get(GlobalParameters::PARAM_DT), dcm.col(2).cross(-accel)) * accelWeight;
  }

  // If a magnetometer is available, use the provided north vector to correct
  // the yaw.
  if(meas.mag) {
    Eigen::Vector3f mag((*meas.mag).axes.data());

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
  if(meas.gyro) {
    Eigen::Vector3f gyro((*meas.gyro).axes.data());
    gyro = gyroFilter.apply(params.get(GlobalParameters::PARAM_DT), gyro);
    corr += gyro * params.get(GlobalParameters::PARAM_DT) * (1.0f - accelWeight - magWeight);
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

  return makeEstimate(meas);
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
  float maxAccelWeight = 0.002f; // Accelerometer weight at exactly 1g
  float validAccelRange = 0.2f; // Maximum additional acceleration until accelWeight goes to 0

  // Deweight accelerometer as a linear function of the reading's difference
  // from 1g.
  float accelOffset = std::abs(1.0f - accel.norm());
  float accelWeight = -maxAccelWeight / validAccelRange * accelOffset + maxAccelWeight;

  // Zero weight if correction would be too large. Specifically want to avoid
  // rockets flipping on deceleration.
  if (dcm.col(2).dot(accel) < -0.5) {
    accelWeight = 0;
  }

  // Limit weight to a minimum of 0
  accelWeight = std::max(0.0f, accelWeight);

  return accelWeight;
}

AttitudeEstimate DCMAttitudeEstimator::makeEstimate(const SensorMeasurements& meas) {
  AttitudeEstimate estimate = {
    .time = ST2MS(chibios_rt::System::getTime()),
    .dcm = {
      dcm(0, 0), dcm(0, 1), dcm(0, 2),
      dcm(1, 0), dcm(1, 1), dcm(1, 2),
      dcm(2, 0), dcm(2, 1), dcm(2, 2)
    },

    // Euler angles (yaw-pitch-roll)
    // TODO(yoos): Due to singularities, this is really only safe for
    // visualization. We can use this for position control but should not
    // deviate more than perhaps 30 degrees away from horizontal.
    .roll = atan2f(dcm(1,2), dcm(2,2)),
    .pitch = asinf(-dcm(0,2)),
    //.roll = -atan2(dcm(1,2), dcm(2,2)) * dcm(0,0) + atan2(dcm(0,2), dcm(2,2)) * dcm(1,0),
    //.pitch = atan2(dcm(0,2), dcm(2,2)) * dcm(1,1) - atan2(dcm(1,2), dcm(2,2)) * dcm(0,1),
    .yaw = atan2f(dcm(0,1), dcm(0,0)),

    // Velocities are set later if a gyro is available.
    .rollVel = 0.0f,
    .pitchVel = 0.0f,
    .yawVel = 0.0f,

    // TODO: Calculate accelerations.
    .rollAcc = 0.0f,
    .pitchAcc = 0.0f,
    .yawAcc = 0.0f
  };

  // If a gyro is available then use the direct measurements for velocity
  // calculation.
  if(meas.gyro) {
    estimate.rollVel = (*meas.gyro).axes[0];
    estimate.pitchVel = (*meas.gyro).axes[1];
    estimate.yawVel = (*meas.gyro).axes[2];
  } else {
    // TODO: Differentiate estimates or just ignore?
  }

  return estimate;
}

void DCMAttitudeEstimator::updateStream() {
  protocol::message::attitude_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .dcm = {
      dcm(0, 0), dcm(0, 1), dcm(0, 2),
      dcm(1, 0), dcm(1, 1), dcm(1, 2),
      dcm(2, 0), dcm(2, 1), dcm(2, 2)
    }
  };

  if(attitudeMessageStream.ready()) {
    attitudeMessageStream.publish(m);
  }

  // Full-resolution attitude log by itself accounts for nearly half of all
  // logging. Downsample to 100 Hz so the filesystem has an easier time keeping
  // up.
  static int i=0;
  if (i % 10 == 0) {
    logger.write(m);
  }
  i = (i+1) % 1000;
}
