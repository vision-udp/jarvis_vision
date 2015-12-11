//          Copyright Diego Ramírez December 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_ANY_MAP_HPP
#define JARVIS_ANY_MAP_HPP

#include <boost/any.hpp>
#include <map>
#include <string>

namespace jarvis {

struct any_map : std::map<std::string, boost::any> {

  /// Adds the given pair to the map
  ///
  /// \returns true if the given pair was inserted, i.e, no pair with the given
  /// key was present.
  ///
  template <typename T>
  bool add(const std::string &key, const T &value) {
    auto &elem = (*this)[key];
    if (!elem.empty())
      return false;
    elem = value;
    return true;
  }

  template <typename T>
  const T &get(const std::string &str) const {
    return boost::any_cast<const T &>(at(str));
  }
};

} // end namespace jarvis

#endif // Header guard
