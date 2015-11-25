//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef TOOLS_PCD_RECORDER_RECORDER_HPP
#define TOOLS_PCD_RECORDER_RECORDER_HPP

#include <chrono>  // For std::chrono::milliseconds
#include <memory>  // For std::unique_ptr
#include <string>  // For std::string
#include <cstddef> // For std::size_t

namespace jarvis {

void list_openni_devices();
void show_openni_device_info(const std::string &device_id);

class pcd_recorder {
public:
  struct param_type {
    std::string point_type;
    std::size_t frames_to_take;
    bool depth_registration;
  };

public:
  static std::unique_ptr<pcd_recorder> make(const param_type &params);

public:
  virtual ~pcd_recorder();
  virtual void start() = 0;

  virtual void wait() = 0;

  // Returns true if execution is ready. If no execution exists (for example was
  // stopped) invokes undefined behaviour.
  virtual bool wait_for(const std::chrono::milliseconds &) = 0;

  virtual void stop() = 0;
};

} // end namespace jarvis

#endif // Header guard
