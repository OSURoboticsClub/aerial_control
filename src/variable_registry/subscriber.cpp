#include <variable_registry/subscriber.hpp>

void VariableRegistrySubscriber::subscribe(VariableRegistry& reg) {
  reg.registerSubscriber(this);   // TODO(syoo): eh
}
