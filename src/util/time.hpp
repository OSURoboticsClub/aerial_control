#ifndef TIME_HPP_
#define TIME_HPP_

#define ST2MS(n) ((uint32_t) ((((uint64_t ((n) - 1)) * 1000) / CH_FREQUENCY) + 1))
#define ST2US(n) ((uint32_t) ((((uint64_t ((n) - 1)) * 1000000) / CH_FREQUENCY) + 1))

#endif // TIME_HPP_
