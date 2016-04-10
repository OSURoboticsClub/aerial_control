#ifndef VARIABLE_REGISTRY_SUBSCRIBER_HPP
#define VARIABLE_REGISTRY_SUBSCRIBER_HPP

#include <variable_registry/registry.hpp>

class VariableRegistry;

class VariableRegistrySubscriber {
public:
  VariableRegistrySubscriber();

  void subscribe(VariableRegistry& reg);
};

#endif
