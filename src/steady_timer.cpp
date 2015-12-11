//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/steady_timer.hpp>

#include <iostream> // for std::clog, std::endl

using namespace jarvis;

void steady_timer::run(const char *task_name) {
  std::clog << "[ RUN      ] " << task_name << std::endl;
  start = clock::now();
}

auto steady_timer::finish() -> clock::duration {
  const auto elapsed = clock::now() - start;
  std::chrono::duration<double> seconds = clock::now() - start;
  std::clog << "[       OK ] " << seconds.count() << 's' << std::endl;
  return elapsed;
}
