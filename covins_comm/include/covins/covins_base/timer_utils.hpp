#ifndef SRC_UTILS_TIMER_H_
#define SRC_UTILS_TIMER_H_

#include <chrono>

namespace covins {

namespace time_utils {

using MicroSecs = std::chrono::microseconds;
using MilliSecs = std::chrono::milliseconds;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using std::chrono::duration_cast;

/**
 * @brief      A small utility time measurement class
 */
class Timer {
public:
  enum class Units { Micro, Milli };
  void start() { _start = Clock::now(); }
  uint64_t measure(const Units &units = Units::Micro) {
    auto end = Clock::now();
    uint64_t res = 0;
    switch (units) {
    case Units::Micro:
      res = duration_cast<MicroSecs>(end - _start).count();
      break;
    case Units::Milli:
      res = duration_cast<MilliSecs>(end - _start).count();
      break;
    }
    _start = Clock::now();
    return res;
  }

  uint64_t measure_from_init(const Units &units = Units::Micro) {
    auto end = Clock::now();
    uint64_t res = 0;
    switch (units) {
    case Units::Micro:
      res = duration_cast<MicroSecs>(end - _start).count();
      break;
    case Units::Milli:
      res = duration_cast<MilliSecs>(end - _start).count();
      break;
    }
    return res;
  }

private:
  TimePoint _start = Clock::now();
};

} // namespace time_utils

} // namespace covins

#endif // SRC_UTILS_TIMER_H_