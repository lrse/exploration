
#include <glob.h>
#include "util.h"
using namespace std;

string HybNav::OUTPUT_DIRECTORY;

namespace HybNav {
  std::list<std::string> glob(const std::string& pattern) {
    std::list<std::string> out;
    glob_t g;
    int ret = glob(pattern.c_str(), 0, NULL, &g);
    if (ret == 0) {
      for (size_t i = 0; i < g.gl_pathc; i++) out.push_back(g.gl_pathv[i]);
    }
    globfree(&g);

    return out;
  }
}


