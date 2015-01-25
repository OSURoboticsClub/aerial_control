#ifndef VEHICLE_SYSTEM_HPP_
#define VEHICLE_SYSTEM_HPP_

#include "communication/communicator.hpp"

/**
 * A collection of sensors, input sources, estimators, and controllers that
 * define a particular vehicle class, e.g. multirotors, fixed wings, etc.
 */
class VehicleSystem {
public:
  /**
   * Periodic update function. Called at a fixed interval.
   */
  virtual void update() = 0;

  bool isArmed();
  void setArmed(bool armed_);

protected:
  VehicleSystem(Communicator& communicator);

private:
  bool armed;

  Communicator& communicator;
};

#endif
