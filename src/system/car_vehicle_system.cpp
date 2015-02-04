#include "system/car_vehicle_system.hpp"

#include "hal.h"

#include "util/optional.hpp"

CarVehicleSystem::CarVehicleSystem(Gyroscope& gyroscope, Accelerometer& accelerometer,
    PWMDeviceGroup<4>& motorDevices, PWMDeviceGroup<4>& servoDevices,
    Communicator& communicator)
  : VehicleSystem(communicator), MessageListener(communicator),
    gyroscope(gyroscope), accelerometer(accelerometer), estimator(communicator),
    inputSource(communicator), motorMapper(motorDevices, servoDevices, communicator) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void CarVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyroReading = gyroscope.readGyro();
  accelerometer_reading_t accelReading = accelerometer.readAccel();

  sensor_reading_group_t readings {
    .gyro = std::experimental::make_optional(gyroReading),
    .accel = std::experimental::make_optional(accelReading),
  };

  // Update the attitude estimate
  attitude_estimate_t estimate = estimator.update(readings);

  // Poll for controller input
  controller_input_t input = inputSource.read();

  // Run the controllers
  actuator_setpoint_t actuatorSp;
  if(isArmed()) {
    // Run the controller pipeline as determined by the subclass
    angular_velocity_setpoint_t sp {
      .roll_vel_sp = input.roll_sp,
      .pitch_vel_sp = input.pitch_sp,
      .yaw_vel_sp = input.yaw_sp,
      .throttle_sp = input.throttle_sp
    };
    actuatorSp = pipeline.run(estimate, sp, attVelController);
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(estimate, actuatorSp);
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

void CarVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
