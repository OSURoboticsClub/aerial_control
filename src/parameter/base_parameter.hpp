#ifndef BASE_PARAMETER_HPP_
#define BASE_PARAMETER_HPP_

class BaseParameter {
public:
  BaseParameter(const char *name_);

  char *getName() const;

private:
  char *name;
};

#endif
