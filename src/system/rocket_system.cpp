#include "system/rocket_system.hpp"

RocketSystem::RocketSystem(
    Gyroscope& gyroscope, Accelerometer& accelerometer,
    AttitudeEstimator& estimator, InputSource& inputSource,
    MotorMapper& motorMapper, Communicator& communicator)
  : VehicleSystem(communicator), MessageListener(communicator),
    gyroscope(gyroscope), accelerometer(accelerometer),
    estimator(estimator), inputSource(inputSource),
    motorMapper(motorMapper) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void RocketSystem::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyroReading = gyroscope.readGyro();
  accelerometer_reading_t accelReading = accelerometer.readAccel();

  // Update the attitude estimate
  attitude_estimate_t estimate = estimator.update(gyroReading, accelReading);

  // Poll for controller input
  controller_input_t input = inputSource.read();

  // Run the controllers
  actuator_setpoint_t actuatorSp;
  if(isArmed() && input.valid) {
    // Run the controller pipeline as determined by the subclass
    switch(mode) {
      case RocketStage::LAUNCH: {
        break;
      }
      case RocketStage::FLY: {
        angular_velocity_setpoint_t sp {
          .roll_vel_sp  = 0.0f,
          .pitch_vel_sp = 0.0f,
          .yaw_vel_sp   = 0.0f,
          .throttle_sp  = 0.0f
        };
        actuatorSp = pipeline.run(estimate, sp, attVelController, attAccController);
        break;
      }
    }
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(estimate, actuatorSp);
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

void RocketSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
