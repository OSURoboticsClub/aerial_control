#include <controller/controller.hpp>

bool Controller::isPassthrough() {
  return passthrough;
}

void Controller::setPassthrough(bool passthrough) {
  this->passthrough = passthrough;
}
