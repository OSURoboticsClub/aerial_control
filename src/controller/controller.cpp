#include <controller/controller.hpp>

bool Controller::isPassthrough() const {
  return passthrough;
}

void Controller::setPassthrough(bool passthrough) {
  this->passthrough = passthrough;
}
