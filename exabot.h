#ifndef __EXABOT_H__
#define __EXABOT_H__

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libplayerc++/playerc++.h>
#include <libplayerc/playerc.h>
#include <fstream>
#include "singleton.h"
#include "metric_map.h"
#include "explorer.h"
#include "local_explorer.h"
#include "global_explorer.h"
#include "motion_planner.h"
#include "config.h"

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
      
      PlayerCc::PlayerClient player_client;
      PlayerCc::LaserProxy laser_proxy;
      PlayerCc::Position2dProxy position_proxy;
      ONLY_ON_SIMULATION(PlayerCc::Graphics2dProxy graphics_proxy;)
      
      static double ROBOT_RADIUS;

      gsl::vector initial_position, last_position;
      double initial_rotation, last_rotation;
      std::ofstream timings_file, map_statistics_file;

      Scan last_scan;
      
    private:     
      
      double trajectory_length;
      std::list<gsl::vector> trajectory;

      MetricMap metric_map;
      Explorer explorer;
      LocalExplorer local_explorer;
      GlobalExplorer global_explorer;
      TopoMap topo_map;
      MotionPlanner motion_planner;

      void get_pose(gsl::vector& abs_pos, double& abs_rot);
      void correct_pose(gsl::vector& absolute_position, double& absolute_rotation);
      void update_position(void);
      bool first_plot;

      std::time_t graph_timer, start_timer;
      
      cv::VideoWriter *graph_writer, *debug_writer;
  };
}

#endif

