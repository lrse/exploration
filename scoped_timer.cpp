#include "scoped_timer.h"
using namespace HybNav;
using namespace std;

ScopedTimer::ScopedTimer(const std::string& _name, std::ostream& _str) : name(_name), str(_str)
{
  last_time = boost::posix_time::microsec_clock::local_time();
}

ScopedTimer::~ScopedTimer(void)
{
  float seconds = (boost::posix_time::microsec_clock::local_time() - last_time).total_microseconds() / 1000000.0;
  str << name << " " << seconds << endl;
}
