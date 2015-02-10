#ifndef SETPOINT_TYPES_HPP_
#define SETPOINT_TYPES_HPP_

// Setpoint types collected here to prevent circular dependencies between
// controllers.

struct PositionSetpoint {
  float latitude;
  float longitude;
  float yawPos;
  float altitude;
};

struct AngularPositionSetpoint {
  float rollPos;
  float pitchPos;
  float yawPos;
  float throttle;
};

struct AngularVelocitySetpoint {
  float rollVel;
  float pitchVel;
  float yawVel;
  float throttle;
};

struct AngularAccelerationSetpoint {
  float rollAcc;
  float pitchAcc;
  float yawAcc;
  float throttle;
};

struct ActuatorSetpoint {
  float roll;
  float pitch;
  float yaw;
  float throttle;
};

#endif
