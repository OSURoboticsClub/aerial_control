#include "system/multirotor_vehicle_system.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(Communicator& communicator)
  : VehicleSystem(communicator), mode(MultirotorControlMode::ANGULAR_POS) {
}

void MultirotorVehicleSystem::init() {
  // TODO: For now, arm the vehicle on initialization. In the future, this
  // should be removed and arming should be designated by a control input or
  // from a communication link.
  setArmed(true);
}

template <>
actuator_setpoint_t MultirotorVehicleSystem::runPipeline(const attitude_estimate_t& estimate, const angular_velocity_setpoint_t& sp) {
  return attVelController.run(estimate, sp);
}

template <>
actuator_setpoint_t MultirotorVehicleSystem::runPipeline(const attitude_estimate_t& estimate, const angular_position_setpoint_t& sp) {
  return runPipeline(estimate, attPosController.run(estimate, sp));
}

template <>
actuator_setpoint_t MultirotorVehicleSystem::runPipeline(const attitude_estimate_t& estimate, const position_setpoint_t& sp) {
  return runPipeline(estimate, posController.run(estimate, sp));
}

void MultirotorVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyroReading = getGyroscope().readGyro();
  accelerometer_reading_t accelReading = getAccelerometer().readAccel();

  // Update the attitude estimate
  attitude_estimate_t estimate = getAttitudeEstimator().update(gyroReading, accelReading);

  // Poll for controller input
  controller_input_t input = getInputSource().read();

  // Run the controllers
  actuator_setpoint_t actuatorSp;
  if(isArmed()) {
    // Run the controller pipeline as determined by the subclass
    switch(mode) {
      case MultirotorControlMode::POSITION: {
        position_setpoint_t sp {
          .latitude_sp = input.roll_sp,
          .longitude_sp = input.pitch_sp,
          .yaw_pos_sp = input.yaw_sp,
          .altitude_sp = input.throttle_sp
        };
        actuatorSp = runPipeline(estimate, sp);
        break;
      }
      case MultirotorControlMode::VELOCITY: {
        // TODO: implement
        // actuatorSp = runPipeline(estimate, sp);
        break;
      }
      case MultirotorControlMode::ANGULAR_POS: {
        angular_position_setpoint_t sp {
          .roll_pos_sp = input.roll_sp,
          .pitch_pos_sp = input.pitch_sp,
          .yaw_pos_sp = input.yaw_sp,
          .throttle_sp = input.throttle_sp
        };
        actuatorSp = runPipeline(estimate, sp);
        break;
      }
      case MultirotorControlMode::ANGULAR_RATE: {
        angular_velocity_setpoint_t sp {
          .roll_vel_sp = input.roll_sp,
          .pitch_vel_sp = input.pitch_sp,
          .yaw_vel_sp = input.yaw_sp,
          .throttle_sp = input.throttle_sp
        };
        actuatorSp = runPipeline(estimate, sp);
        break;
      }
    }
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(estimate, actuatorSp);
  }

  // Update motor outputs
  getMotorMapper().run(actuatorSp);
}

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
