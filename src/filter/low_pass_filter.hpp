#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

class LowPassFilter {
public:
  LowPassFilter(float breakFrequency);
  LowPassFilter(float breakFrequency, float value);

  float apply(float dt, float x);

private:
  float breakFrequency;
  float value;
};

#endif // LOW_PASS_FILTER_H_
