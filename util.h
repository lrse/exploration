#ifndef UTIL_H
#define	UTIL_H

#include <list>
#include <string>
#include <boost/lexical_cast.hpp>

namespace HybNav {
  std::list<std::string> glob(const std::string& pattern);
  
  template<class T> std::string to_s(const T& value) { return boost::lexical_cast<std::string>(value); }
}

#endif	/* UTIL_H */


