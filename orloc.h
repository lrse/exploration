/*
 * File name: orloc.cc
 * Date:      2012/5/10 10:43
 * Author:    Miroslav Kulich
 */


#ifndef _orloc_h_
#define _orloc_h_

#include <libplayerc++/playerc++.h>
#include <vector>

namespace HybNav {
  const double DIST_THRESHOLD  = 0.07;
  const double DIST_THRESHOLD_2  = DIST_THRESHOLD*DIST_THRESHOLD;
  const unsigned int NUMBER_THRESHOLD = 20;
  const double PRECISION  = 0.05;

  typedef std::vector<player_point_2d_t> Scan;

  struct SPosition {
    double x;
    double y;
    double yaw;

    SPosition() {}
    SPosition(double x, double y) : x(x), y(y) {}
    SPosition(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
  };

  // modify yaw of pos so that laser data are aligned with axes of the coordinate system
  void angleCorrection(SPosition &pos, const PlayerCc::LaserProxy &laser, Scan& scan);


  // transform distances measured with laser into points in Cartesian space
  Scan getPoints(const SPosition pos, const PlayerCc::LaserProxy &laser);
}


#endif // _orloc_h_
