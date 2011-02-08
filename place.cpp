#include <iostream>
#include "place.h"
using namespace HybNav;
using namespace std;

double Place::CELL_SIZE = 0.07;
uint Place::CELLS = 51;
double Place::SIZE = Place::CELL_SIZE * Place::CELLS;
double Place::Locc = 1.5;
double Place::Lfree = -1.5;

/**************************
 * Constructor/Destructor *
 **************************/

Place::Place(void) : occupancy_matrix(CELLS, CELLS, true)
{
}

double Place::operator()(uint x, uint y) const {
  double v = occupancy_matrix(CELLS - y - 1, x);
  return v;
}

bool Place::valid_coordinates(int x, int y) {
  return (x >= 0 && (uint)x < CELLS && y >= 0 && (uint)y < CELLS);
}

gsl::vector_int Place::world2grid(const gsl::vector& coord) {
  gsl::vector_int out(2);
  for (uint i = 0; i < 2; i++) {
    out(i) = (uint)floor(coord(i) / CELL_SIZE);
  }
  return out;
}
