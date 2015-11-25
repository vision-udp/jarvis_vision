//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_STEADY_TIMER_HPP
#define JARVIS_STEADY_TIMER_HPP

#include <chrono>

namespace jarvis {

class steady_timer {
  using clock = std::chrono::steady_clock;

public:
  void run(const char *task_name);
  clock::duration finish();

private:
  clock::time_point start;
};

} // end namespace jarvis

#endif // Header guard
