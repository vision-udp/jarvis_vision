//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef TOOLS_PCD_RECORDER_RECORDER_HPP
#define TOOLS_PCD_RECORDER_RECORDER_HPP

#include <chrono> // For std::chrono::milliseconds
#include <memory> // For std::unique_ptr
#include <string> // For std::string

namespace jarvis {

void list_openni_devices();
void show_openni_device_info(const std::string &device_id);

class recorder {
public:
  virtual ~recorder();
  virtual void start() = 0;

  // Returns true if execution is ready.
  virtual bool wait_for(const std::chrono::milliseconds &) = 0;

  virtual void stop() = 0;
};

std::unique_ptr<recorder> make_pcd_recorder(const std::string &point_type,
                                            std::size_t max_num_frames);

} // end namespace jarvis

#endif // Header guard
