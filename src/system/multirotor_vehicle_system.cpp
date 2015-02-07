#include "system/multirotor_vehicle_system.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(
    Gyroscope& gyroscope, Accelerometer& accelerometer, GPS& gps,
    WorldEstimator& world, AttitudeEstimator& attitude, InputSource&
    inputSource, MotorMapper& motorMapper, Communicator& communicator)
  : VehicleSystem(communicator), MessageListener(communicator),
    gyroscope(gyroscope), accelerometer(accelerometer), gps(gps), world(world),
    attitude(attitude), inputSource(inputSource),
    motorMapper(motorMapper), mode(MultirotorControlMode::ANGULAR_POS) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void MultirotorVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyroReading = gyroscope.readGyro();
  accelerometer_reading_t accelReading = accelerometer.readAccel();

  gps_reading_t gpsReading;
  static int i=0;
  if (i++ % 100 == 0) {
    gpsReading = gps.readGPS();
  }

  // Update estimates
  world_estimate_t world_estimate = world.update(gpsReading);
  attitude_estimate_t estimate = attitude.update(gyroReading, accelReading);

  // Poll for controller input
  controller_input_t input = inputSource.read();

  // Run the controllers
  actuator_setpoint_t actuatorSp;
  if(isArmed() && input.valid) {
    // Run the controller pipeline as determined by the subclass
    switch(mode) {
      case MultirotorControlMode::POSITION: {
        position_setpoint_t sp {
          .latitude_sp = input.roll_sp,
          .longitude_sp = input.pitch_sp,
          .yaw_pos_sp = input.yaw_sp,
          .altitude_sp = input.throttle_sp
        };
        actuatorSp = pipeline.run(estimate, sp, posController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::VELOCITY: {
        // TODO: implement
        // actuatorSp = pipeline.run(estimate, sp, velController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_POS: {
        angular_position_setpoint_t sp {
          .roll_pos_sp = input.roll_sp,
          .pitch_pos_sp = input.pitch_sp,
          .yaw_pos_sp = input.yaw_sp,
          .throttle_sp = input.throttle_sp
        };
        actuatorSp = pipeline.run(estimate, sp, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_RATE: {
        angular_velocity_setpoint_t sp {
          .roll_vel_sp = input.roll_sp,
          .pitch_vel_sp = input.pitch_sp,
          .yaw_vel_sp = input.yaw_sp,
          .throttle_sp = input.throttle_sp
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

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
