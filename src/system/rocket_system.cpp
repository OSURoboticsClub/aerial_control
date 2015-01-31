#include "system/rocket_system.hpp"

#include "unit_config.hpp"

RocketSystem::RocketSystem(
    Gyroscope& gyroscope, Accelerometer& accelerometer,
    AttitudeEstimator& estimator, InputSource& inputSource,
    MotorMapper& motorMapper, Communicator& communicator)
  : VehicleSystem(communicator), MessageListener(communicator),
    gyroscope(gyroscope), accelerometer(accelerometer),
    estimator(estimator), inputSource(inputSource),
    motorMapper(motorMapper), stage(RocketStage::DISABLED) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOA, 6);
}

void RocketSystem::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyroReading = gyroscope.readGyro();
  accelerometer_reading_t accelReading = accelerometer.readAccel();

  // Update the attitude estimate
  attitude_estimate_t estimate = estimator.update(gyroReading, accelReading);

  // Poll for controller input
  controller_input_t input = inputSource.read();

  // Keep moving average of acceleration
  static float accel = 0.0f;
  accel = 0.5*accel + 0.5*accelReading.axes[0];

  // Run the controllers
  actuator_setpoint_t actuatorSp;

  // Run the controller pipeline as determined by the subclass
  switch(stage) {
    case RocketStage::DISABLED:
      {
        // If armed, proceed to pad prep.
        if (isArmed()) {
          stage = RocketStage::PAD;
        }
        break;
      }
    case RocketStage::PAD:
      {
        // Set fins to neutral
        actuator_setpoint_t sp {
          .roll_sp     = 0.5f,
          .pitch_sp    = 0.0f,
          .yaw_sp      = 0.0f,
          .throttle_sp = 0.0f
        };
        actuatorSp = sp;

        // If acceleration moving average exceeds 2g (should occur around 0.44s
        // according to sim), proceed to ascent.
        if (accel > 0.8f) {
          stage = RocketStage::ASCENT;
          palSetPad(GPIOA, 6);
        }
        break;
      }
    case RocketStage::ASCENT:
      {
        unit_config::launchtime += 0.001;
        angular_position_setpoint_t sp {
          .roll_pos_sp  = 0.0f,
          .pitch_pos_sp = 0.0f,
          .yaw_pos_sp   = 0.0f,
          .throttle_sp  = 0.0f
        };
        actuatorSp = pipeline.run(estimate, sp, attPosController, attVelController, attAccController);

        // If deviated more than 60 deg past vertical, proceed to descent.
        if (estimate.pitch < 0.5 || unit_config::launchtime > 15) {
          stage = RocketStage::DESCENT;
          palClearPad(GPIOA, 6);
        }
        break;
      }
    case RocketStage::DESCENT:
      {
        setArmed(false);
        break;
      }
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

void RocketSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
