#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

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
};

#endif
