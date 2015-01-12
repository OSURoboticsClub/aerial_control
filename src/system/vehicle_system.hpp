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
   * Initialize any subsystems that were not passed into the constructor (those
   * should already have been initialized).
   */
  virtual void init() = 0;

  /**
   * Periodic update function. Called at a fixed interval.
   */
  virtual void update() = 0;

  bool isArmed();
  void setArmed(bool armed_);

protected:
  // TODO: Defining this in the implementation file causes linker errors...
  VehicleSystem(Communicator& communicator) : communicator(communicator) {
  }

private:
  bool armed;

  Communicator& communicator;
};

#endif
