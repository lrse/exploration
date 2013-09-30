#ifndef MOTION_PLANNER_H
#define	MOTION_PLANNER_H

#include <libplayerc++/playerc++.h>
#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_double.h>
#include <gslwrap/vector_int.h>
//#include <boost/timer.hpp>
#include "singleton.h"

namespace HybNav {
  class MotionPlanner : public Singleton<MotionPlanner> {
    public:
      MotionPlanner(PlayerCc::PlayerClient* player_client);
      
      void set_goal(double x, double y, double theta);
      bool valid_path(void);
      void stop(void);

      bool reached_goal(void);
      bool reached(const gsl::vector& pos, double angle);
      void update(void);

      double goal_x, goal_y, goal_theta;
      bool goal_set;

      float seconds_elapsed;
      boost::posix_time::ptime last_time;

      float REACHED_POS_EPSILON, REACHED_ANGLE_EPSILON;      
      PlayerCc::Position2dProxy position_proxy;
  };
}

#endif	/* MOTION_PLANNER_H */


