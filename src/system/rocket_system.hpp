#ifndef ROCKET_SYSTEM_HPP_
#define ROCKET_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "input/input_source.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/vehicle_system.hpp"

template <int num_rotors>
class RocketSystem : public VehicleSystem {
public:
  RocketSystem(Communicator& communicator);

  void init() override;
  void update() override;

protected:
  virtual Gyroscope& getGyroscope() = 0;
  virtual Accelerometer& getAccelerometer() = 0;
  virtual AttitudeEstimator& getAttitudeEstimator() = 0;
  virtual InputSource& getInputSource() = 0;
  virtual MotorMapper& getMotorMapper() = 0;

  virtual actuator_setpoint_t runController(const attitude_estimate_t& estimate, const angular_position_setpoint_t& setpoint) = 0;
};

#include "system/rocket_system.tpp"

#endif
