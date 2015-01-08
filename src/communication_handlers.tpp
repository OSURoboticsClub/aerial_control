// #include "system/multirotor_vehicle_system.hpp"

namespace msg = protocol::message;

template <typename M>
void CommunicationThread::on(const M& m) {
}

template <>
void CommunicationThread::on(const msg::heartbeat_message_t& m) {
  // TODO: This makes assumptions about the system type. This won't work in
  // general.
  // auto& system = static_cast<MultirotorVehicleSystem<4>&>(unit.getSystem());
  // attitude_estimate_t estimate = system.getAttitudeEstimator().getLastEstimate();
  //
  // msg::attitude_message_t message {
  //   .roll = estimate.roll,
  //   .pitch = estimate.pitch,
  //   .yaw = estimate.yaw
  // };
  //
  // send(message);
}

template <>
void CommunicationThread::on(const msg::set_control_mode_t& m) {
  if(m.mode == msg::set_control_mode_t::ControlMode::MANUAL) {
  } else if(m.mode == msg::set_control_mode_t::ControlMode::OFFBOARD) {
  }
}
