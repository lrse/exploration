#include <gsl/gsl_sf_trig.h>
#include <iostream>
#include "metric_map.h"
#include "exabot.h"
#include "explorer.h"
#include "util.h"
#include "config.h"

using namespace HybNav;
using namespace std;

#ifdef SYROTEK
double ExaBot::ROBOT_RADIUS = 0.04;
#else
double ExaBot::ROBOT_RADIUS = 0.09;
#endif

/**************************
 * Constructor/Destructor *
 **************************/

ExaBot::ExaBot(void) : Singleton<ExaBot>(this), player_client("localhost"), ranger_proxy(&player_client),
  position_proxy(&player_client, 0),
  trajectory_length(0), motion_planner(&player_client)
{  
  ranger_proxy.RequestGeom();
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
  
#ifdef ENABLE_DISPLAY
  cvStartWindowThread();
  cv::namedWindow("grid");
  cv::namedWindow("debug");
#endif
  graph_writer = new cv::VideoWriter("graph.avi", CV_FOURCC('M','J','P','G'), 1, cv::Size(OccupancyGrid::CELLS, OccupancyGrid::CELLS) * 4);
  debug_writer = new cv::VideoWriter("debug.avi", CV_FOURCC('M','J','P','G'), 1, cv::Size(OccupancyGrid::CELLS, OccupancyGrid::CELLS) * 4);

  sleep(1);
}

ExaBot::~ExaBot(void) {
}

/**************************
 *     Public Methods     *
 **************************/

void ExaBot::update_player(void)
{
  while (player_client.Peek(0))
    player_client.Read();
}

void ExaBot::update(void) {
  try {
    update_player();
    //if (position_proxy.GetStall()) throw std::runtime_error("collision detected!");

    update_position();

    MetricMap::instance()->process_distances(position_proxy, ranger_proxy);
    //MotionPlanner::instance()->process_distances(laser_proxy);
    if (std::time(NULL) - start_timer > 3) {
      Explorer::instance()->update();
    }

    // Graphics
    cout << "timer " << (std::time(NULL) - graph_timer) << endl;
    if (!first_plot || (std::time(NULL) - graph_timer) >= 1) {
      first_plot = true;
      graph_timer = std::time(NULL);
      
      cv::Mat graph;
      
      // plot grid      
      MetricMap::instance()->current_grid->draw(graph);
      
      // plot robot position
      gsl::vector_int robot_position = MetricMap::instance()->grid_position();
      robot_position = robot_position + 0.5;
      cv::circle(graph, cv::Point(robot_position(0), OccupancyGrid::CELLS - robot_position(1) - 1), floor(ExaBot::ROBOT_RADIUS / OccupancyGrid::CELL_SIZE), cv::Scalar(0,0,255), -1, 4);
      
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
      
      // plot debug overlay
      graph += MetricMap::instance()->current_grid->debug_graph;
      
      // make window bigger
      cv::Mat graph_big;
      cv::resize(graph, graph_big, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);
      
      cv::Mat debug_big;
      cv::resize(LocalExplorer::instance()->frontier_pathfinder.cost_grid, debug_big, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);
      
#ifdef ENABLE_DISPLAY      
      cv::imshow("grid", graph_big);
      cv::imshow("debug", debug_big);
#endif
      *graph_writer << graph_big;
      cv::Mat debug_big_color;
      cv::cvtColor(debug_big, debug_big_color, CV_GRAY2BGR);
      *debug_writer << debug_big_color;
    }  

    Explorer::instance()->compute_motion(position_proxy);
  }
  catch(const PlayerCc::PlayerError& err) {
    cout << "player error!" << endl;
    throw;
  }
}

void ExaBot::stop(void) {
  MotionPlanner::instance()->stop();
  position_proxy.SetMotorEnable(false);
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
  
  cout << "Closing videos..." << endl;
  if (debug_writer) { delete debug_writer; debug_writer = NULL; }
  if (graph_writer) { delete graph_writer; graph_writer = NULL; }
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

