#ifndef __SCOPED_TIMER_H__
#define __SCOPED_TIMER_H__

#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace HybNav {
  class ScopedTimer {
    public:
      ScopedTimer(const std::string& name, std::ostream& str);
      ~ScopedTimer(void);

    private:
      boost::posix_time::ptime last_time;
      std::string name;
      std::ostream& str;
  };
}

#endif
