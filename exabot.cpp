#include <gsl/gsl_sf_trig.h>
#include <iostream>
#include "metric_map.h"
#include "exabot.h"
#include "explorer.h"
#include "util.h"
#include "config.h"

#ifdef ENABLE_PLOTS
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

using namespace HybNav;
using namespace std;

/**************************
 * Constructor/Destructor *
 **************************/

ExaBot::ExaBot(void) : Singleton<ExaBot>(this), player_client("localhost"), laser_proxy(&player_client),
  position_proxy(&player_client, 0), simulator_proxy(&player_client),
  trajectory_length(0), motion_planner(&player_client)
{
  laser_proxy.RequestGeom();
  position_proxy.RequestGeom();

  player_client.Read();

  // initialize variables
  double dummy;
  last_position = gsl::vector(2, true);
  get_pose(last_position, dummy);
  trajectory.push_back(last_position);
  
  initial_position = last_position;
  trajectory_length = 0;

  graph_timer = std::time(NULL);
  start_timer = std::time(NULL);
  first_plot = false;
  
  cvStartWindowThread();
  cv::namedWindow("grid");

  sleep(1);
}

ExaBot::~ExaBot(void) {
}

/**************************
 *     Public Methods     *
 **************************/

void ExaBot::update_player(void)
{
  player_client.Read();
}

void ExaBot::update(void) {
  try {
    update_player();
    //if (position_proxy.GetStall()) throw std::runtime_error("collision detected!");

    update_position();

    MetricMap::instance()->process_distances(position_proxy, laser_proxy);
    //MotionPlanner::instance()->process_distances(laser_proxy);
    if (std::time(NULL) - start_timer > 3) {
      Explorer::instance()->update();
    }

#ifdef ENABLE_PLOTS
    // Graphics
    cout << "timer " << (std::time(NULL) - graph_timer) << endl;
    if (!first_plot || (std::time(NULL) - graph_timer) >= 1) {
      first_plot = true;
      graph_timer = std::time(NULL);
      
      cv::Mat graph(OccupancyGrid::CELLS, OccupancyGrid::CELLS, CV_8UC3);
      
      // plot grid      
      graph = cv::Scalar(0,0,0);
      for (uint i = 0; i < OccupancyGrid::CELLS; i++) {
        for (uint j = 0; j < OccupancyGrid::CELLS; j++) {
          double value_real = 1 - (MetricMap::instance()->current_grid->m(i,j) + OccupancyGrid::Locc) / (OccupancyGrid::Locc * 2);
          unsigned char value = (unsigned char)(255.0f * value_real);
          graph.at<cv::Vec3b>(i,j) = cv::Vec3b(value, value, value);
        }
      }
      
      // plot robot position
      gsl::vector_int robot_position = MetricMap::instance()->grid_position();
      robot_position = robot_position + 0.5;
      cv::circle(graph, cv::Point(robot_position(0), OccupancyGrid::CELLS - robot_position(1) - 1), MetricMap::ROBOT_RADIUS / OccupancyGrid::CELL_SIZE, cv::Scalar(0,0,255), -1);
      
      // plot path
      if (!LocalExplorer::instance()->follow_path.empty()) {
        list<gsl::vector_int>& path = LocalExplorer::instance()->follow_path;
        vector<cv::Point> path_points(path.size());
        size_t i = 0;
        for (list<gsl::vector_int>::iterator it = path.begin(); it != path.end(); ++it, i++) {
          path_points[i] = cv::Point((*it)(0), OccupancyGrid::CELLS - (*it)(1) - 1);
        }
        cv::polylines(graph, vector< vector<cv::Point> >(1, path_points), false, cv::Scalar(0, 255, 0));
      }
      
      cout << MetricMap::instance()->current_grid->debug_graph.size().width << endl;
      graph += MetricMap::instance()->current_grid->debug_graph;
      
      cv::Mat graph_big;
      cv::resize(graph, graph_big, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);
      cv::imshow("grid", graph_big);
#endif
    }  

    Explorer::instance()->compute_motion(position_proxy);
  }
  catch(const PlayerCc::PlayerError& err) {
    cout << "player error!" << endl;
    throw;
  }
}

void ExaBot::stop(void) {
  position_proxy.SetSpeed(0, 0);
  player_client.Read();
}

void ExaBot::deinitialize(void) {
  cout << "Trajectory length: " << trajectory_length << endl;
  ofstream trajectory_csv("csv/trajectory.csv");
  for (list<gsl::vector>::iterator it = trajectory.begin(); it != trajectory.end(); ++it) trajectory_csv << (*it)[0] << "," << (*it)[1] << endl;
  trajectory_csv.close();
  
  cout << "Saving Map..." << endl;
  MetricMap::instance()->save();
  TopoMap::instance()->save();
}


/**************************
 *    Private Methods     *
 **************************/

void ExaBot::get_pose(gsl::vector& absolute_position, double& absolute_rotation) {
  absolute_position(0) = position_proxy.GetXPos();
  absolute_position(1) = position_proxy.GetYPos();

  absolute_rotation = gsl_sf_angle_restrict_pos(position_proxy.GetYaw());
  cout << "Position: " << absolute_position(0) << "," << absolute_position(1) << " Yaw: " << absolute_rotation << endl;
}

void ExaBot::update_position(void) {
  gsl::vector absolute_position(2);
  double absolute_rotation;
  
  get_pose(absolute_position, absolute_rotation);
  
  gsl::vector delta_position = absolute_position;
  delta_position -= last_position;
  trajectory_length += delta_position.norm2();

  double delta_rotation = gsl_sf_angle_restrict_symm(absolute_rotation - last_rotation);

  MetricMap::instance()->update_position(delta_position, delta_rotation); // TODO: no need for this
  //MotionPlanner::instance()->update_position();
  last_position = absolute_position;
  last_rotation = absolute_rotation;

  trajectory_length += delta_position.norm2();    
  if ((trajectory.back() - absolute_position).norm2() > 0.3) {
    trajectory.push_back(absolute_position);
  }
}
