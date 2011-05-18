#ifndef __EXABOT_H__
#define __EXABOT_H__

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <ctime>
#include <libplayerc++/playerc++.h>
#include "singleton.h"
#include "plotter.h"
#include "metric_map.h"
#include "explorer.h"
#include "local_explorer.h"
#include "global_explorer.h"
#include "motion_planner.h"

using namespace boost::numeric;

namespace HybNav {
  class ExaBot : public Singleton<ExaBot>  {
    public:
      ExaBot(void);
      ~ExaBot(void);

      void update(void);
      void stop(void);
      void deinitialize(void);

      void update_player(void);

    private:
      PlayerCc::PlayerClient player_client;
      PlayerCc::LaserProxy laser_proxy;
      PlayerCc::Position2dProxy position_proxy;
      PlayerCc::Position2dProxy target_position_proxy;
      PlayerCc::SimulationProxy simulator_proxy;

      MetricMap metric_map;
      Explorer explorer;
      LocalExplorer local_explorer;
      GlobalExplorer global_explorer;
      TopoMap topo_map;
      MotionPlanner motion_planner;

      gsl::vector initial_position, last_position;
      double initial_rotation, last_rotation;
      double trajectory_length;

      void get_pose(gsl::vector& abs_pos, double& abs_rot);
      void update_position(void);
      bool first_plot;

      std::time_t graph_timer;
  };
}

#endif