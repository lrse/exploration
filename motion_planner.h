#ifndef MOTION_PLANNER_H
#define	MOTION_PLANNER_H

#include <libplayerc++/playerc++.h>
#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_double.h>
#include <gslwrap/vector_int.h>
#include "singleton.h"

namespace HybNav {
  class MotionPlanner : public Singleton<MotionPlanner> {
    public:
      MotionPlanner(PlayerCc::Position2dProxy& position_proxy);
      void process_distances(PlayerCc::LaserProxy& laser_proxy);

      double winner_direction_angle(void);
      double cost_function(uint c, uint target);
      uint delta_function(int c1, int c2);

      void update_position(void);
      void slide_window(const gsl::vector_int& delta);

      enum Motion { ForwardMotion, LeftTurn, RightTurn };
      void compute_motion(double target_angle);
      double compute_motion_direction(double target_angle);
      Motion compute_motion_from_target(double target_angle);

      uint WINDOW_CELLS;
      uint WINDOW_HALF_CELLS;
      double WINDOW_SIZE;
      double CENTER_POS;
      uint POLAR_SAMPLES;
      double POLAR_SAMPLE_ANGLE;
      uint MU1, MU2, MU3;
      double MINIMUM_DISTANCE;
      double ENLARGEMENT_RADIUS;
      double TAU_HIGH;
      double TAU_LOW;
      uint NARROW_LIMIT;

      std::list<uint> candidates;
      gsl::matrix window;
      gsl::vector polar_histogram;
      gsl::vector_int binary_histogram;
      uint winner_direction;
      
    private:
      PlayerCc::Position2dProxy& position_proxy;
      gsl::vector last_position, delta_position;
  };
}

#endif	/* MOTION_PLANNER_H */

