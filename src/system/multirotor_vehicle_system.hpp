#ifndef MULTIROTOR_SYSTEM_HPP_
#define MULTIROTOR_SYSTEM_HPP_

#include <controller/setpoint_types.hpp>
#include <estimator/attitude_estimator.hpp>
#include <input/input_source.hpp>
#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>
#include <system/vehicle_system.hpp>

template <int num_rotors>
class MultirotorVehicleSystem : public VehicleSystem {
public:
  void init() override;
  void update() override;

protected:
  virtual Accelerometer *getAccelerometer() =0;
  virtual Gyroscope *getGyroscope() =0;
  virtual AttitudeEstimator *getAttitudeEstimator() =0;
  virtual InputSource *getInputSource() =0;
  virtual MotorMapper *getMotorMapper() =0;

  virtual actuator_setpoint_t runController(attitude_estimate_t &estimate, angular_position_setpoint_t& setpoint) =0;
};

#include <system/multirotor_vehicle_system.tpp>

#endif
