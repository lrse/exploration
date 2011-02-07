#ifndef __PLACE_H__
#define __PLACE_H__

#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_int.h>

namespace HybNav {
  class Place {
    public:
      Place(void);

      double operator()(uint x, uint y) const;
      static bool valid_coordinates(int x, int y);
      static gsl::vector_int world2grid(const gsl::vector& coord);

      /* constants */
      static double CELL_SIZE;
      static uint CELLS;
      static double SIZE;
      static double Locc;
      static double Lfree;
      
      gsl::matrix occupancy_matrix;
  };
}

#endif