#ifndef MOVING_AVERAGE_HPP_
#define MOVING_AVERAGE_HPP_

class MovingAverage {
public:
  void add(float datum);

  float getAverage();

private:
  int count;
  float average;
};

#endif
