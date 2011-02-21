#ifndef __SUPER_MATRIX_H__
#define	__SUPER_MATRIX_H__

#include <map>

namespace HybNav {
  template<class T>
  class SuperMatrix {
    public:
      double& cell(ssize_t x, ssize_t y);
      T& submatrix(ssize_t x, ssize_t y);

      void to_dot(std::ostream& out);

      typedef typename std::map<ssize_t, std::map<ssize_t, T> >::iterator iterator_x;
      typedef typename std::map<ssize_t, T>::iterator iterator_y;

      std::map< ssize_t, std::map<ssize_t, T> > matrix_map;
  };
}

#endif	

