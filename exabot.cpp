#include <gsl/gsl_sf_trig.h>
#include <iostream>
#include <sys/stat.h>
#include "metric_map.h"
#include "exabot.h"
#include "explorer.h"
#include "util.h"
#include "config.h"
#include "orloc.h"
#include "scoped_timer.h"

using namespace HybNav;
using namespace std;

double ExaBot::ROBOT_RADIUS = 0.09;

int draw_gateways = 1;



/**************************
 * Constructor/Destructor *
 **************************/

ExaBot::ExaBot(void) : Singleton<ExaBot>(this), player_client(PLAYER_SERVER), laser_proxy(&player_client, 0),
  position_proxy(&player_client, POSITION_PROXY_ID),
  #ifndef ENABLE_SYROTEK
  graphics_proxy(&player_client, 0),
  #endif
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

  mkdir(HybNav::OUTPUT_DIRECTORY.c_str(), 0755);

  timings_file.open((HybNav::OUTPUT_DIRECTORY + "/timings.txt").c_str());
  map_statistics_file.open((HybNav::OUTPUT_DIRECTORY + "/statistics.txt").c_str());
  
#ifdef ENABLE_DISPLAY
  cv::startWindowThread();
  cv::namedWindow("grid", CV_WINDOW_NORMAL);
  cv::namedWindow("cost grid", CV_WINDOW_NORMAL);
  cv::namedWindow("planning grid", CV_WINDOW_NORMAL);
  cv::namedWindow("complete_map", CV_WINDOW_NORMAL);
  cv::namedWindow("topo map", CV_WINDOW_NORMAL);
  
#endif
  graph_writer = new cv::VideoWriter((HybNav::OUTPUT_DIRECTORY + "/graph.avi").c_str(), CV_FOURCC('M','J','P','G'), 1, cv::Size(OccupancyGrid::CELLS, OccupancyGrid::CELLS) * 4);
  debug_writer = new cv::VideoWriter((HybNav::OUTPUT_DIRECTORY + "/debug.avi").c_str(), CV_FOURCC('M','J','P','G'), 1, cv::Size(OccupancyGrid::CELLS, OccupancyGrid::CELLS) * 4);

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
  
  update_player();
  //if (position_proxy.GetStall()) throw std::runtime_error("collision detected!");

  if (position_proxy.IsFresh() && position_proxy.IsValid()) {
    update_position();
    position_proxy.NotFresh();
  }
  if (laser_proxy.IsFresh() && laser_proxy.IsValid()) {
    update_position();
    {
      cout << "received laser reading" << endl;
      ScopedTimer t("process_distances", timings_file);
      MetricMap::instance()->process_distances(position_proxy, /*laser_proxy*/last_scan);
    }
    
    laser_proxy.NotFresh();
  }

  MotionPlanner::instance()->update();
  Explorer::instance()->compute_motion(position_proxy);

  if (std::time(NULL) - start_timer > 3) {
    ScopedTimer t("explorer_iteration", timings_file);
    Explorer::instance()->update();
    map_statistics_file <<
      "grids " << MetricMap::instance()->super_matrix.count << " " <<
      "nodes " << TopoMap::instance()->graph.nodes.size() << " " <<
      "edges " << TopoMap::instance()->graph.edges.size() << endl;    
  }

  // Graphics
  cout << "timer " << (std::time(NULL) - graph_timer) << endl;
  if (!first_plot || (std::time(NULL) - graph_timer) >= 1) {
    first_plot = true;
    graph_timer = std::time(NULL);
    
    cv::Mat graph;
    ONLY_ON_SIMULATION(graphics_proxy.Clear();)
    
    // plot grid      
    MetricMap::instance()->current_grid->draw(graph, draw_gateways);
    
    // plot robot position
    gsl::vector_int robot_position = MetricMap::instance()->grid_position();
    robot_position = robot_position + 0.5;
    cv::circle(graph, cv::Point(robot_position(0), OccupancyGrid::CELLS - robot_position(1) - 1), floor(ExaBot::ROBOT_RADIUS / OccupancyGrid::CELL_SIZE), cv::Scalar(0,0,255), -1, 4);
    
    // plot path
    if (!LocalExplorer::instance()->follow_path.empty()) {
      list<gsl::vector_int>& path = LocalExplorer::instance()->follow_path;
      vector<cv::Point> path_points(path.size());
      vector<player_point_2d_t> path_points_player(path_points.size());
      size_t i = 0;
      for (list<gsl::vector_int>::iterator it = path.begin(); it != path.end(); ++it, i++) {
        path_points[i] = cv::Point((*it)(0), OccupancyGrid::CELLS - (*it)(1) - 1);
        path_points_player[i].px = path_points[i].x * OccupancyGrid::CELL_SIZE/* - robot_position(1)*/;
        path_points_player[i].py = path_points[i].y * OccupancyGrid::CELL_SIZE/* - robot_position(0)*/;
      }
      cv::polylines(graph, vector< vector<cv::Point> >(1, path_points), false, cv::Scalar(0, 255, 0));
      #ifndef ENABLE_SYROTEK
      graphics_proxy.DrawPolyline(&path_points_player[0], path_points.size());
      #endif
    }
    
    // plot debug overlay
    //graph += MetricMap::instance()->current_grid->debug_graph;
    
    // make window bigger
    cv::Mat graph_big;
    cv::resize(graph, graph_big, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);
    
    cv::Mat cost_grid;
    cv::resize(LocalExplorer::instance()->frontier_pathfinder.cost_grid, cost_grid, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);

    cv::Mat planning_grid;
    cv::resize(LocalExplorer::instance()->frontier_pathfinder.grid, planning_grid, cv::Size(0,0), 4, 4, cv::INTER_NEAREST);

    // plot (save) topo map
    TopoMap::instance()->plot();
    
    #ifdef ENABLE_DISPLAY
    cv::imshow("topo map", cv::imread((HybNav::OUTPUT_DIRECTORY + "/topo_map.png").c_str()));
    cv::imshow("grid", graph_big);
    //cv::imshow("cost grid", cost_grid);
    cv::imshow("planning grid", planning_grid);
    cv::Mat complete_map;
    MetricMap::instance()->draw(complete_map, draw_gateways);
    cv::imshow("complete_map", complete_map);
    #endif
    *graph_writer << graph_big;
    /*cv::Mat cost_grid_color;
    cv::cvtColor(cost_grid, cost_grid_color, CV_GRAY2BGR);
    *debug_writer << cost_grid_color;*/
  }
}

void ExaBot::stop(void) {
  MotionPlanner::instance()->stop();
  position_proxy.SetMotorEnable(false);
  player_client.Read();
}

void ExaBot::deinitialize(void) {
  cout << "Saving trajectory..." << endl;
  ofstream trajectory_csv((HybNav::OUTPUT_DIRECTORY + "/trajectory.txt").c_str());
  for (list<gsl::vector>::iterator it = trajectory.begin(); it != trajectory.end(); ++it)
    trajectory_csv << (*it)[0] << " " << (*it)[1] << endl;
  trajectory_csv.close();

  cout << "Saving results..." << endl;
  ofstream results((HybNav::OUTPUT_DIRECTORY + "/results.txt").c_str());
  results << "trajectory_length: " << trajectory_length << endl;
  results << "elapsed_time: " << MotionPlanner::instance()->seconds_elapsed << endl;
  results << "exploration_speed: " << trajectory_length / MotionPlanner::instance()->seconds_elapsed << endl;
  results << "grid_cells: " << OccupancyGrid::CELLS << endl;
  results << "grid_cell_size: " << OccupancyGrid::CELL_SIZE << endl;
  
  cout << "Saving Map..." << endl;
  MetricMap::instance()->save();
  TopoMap::instance()->save();

  cout << "Saving cost grids..." << endl;
  cv::imwrite((HybNav::OUTPUT_DIRECTORY + "/frontier_planning_grid.png").c_str(), LocalExplorer::instance()->frontier_pathfinder.grid);
  cv::imwrite((HybNav::OUTPUT_DIRECTORY + "/connectivity_planning_grid.png").c_str(), LocalExplorer::instance()->connectivity_pathfinder.grid);
  
  cout << "Closing videos..." << endl;
  if (debug_writer) { delete debug_writer; debug_writer = NULL; }
  if (graph_writer) { delete graph_writer; graph_writer = NULL; }
}


/**************************
 *    Private Methods     *
 **************************/

void ExaBot::correct_pose(gsl::vector& absolute_position, double& absolute_rotation)
{
  SPosition pos;
  pos.x = absolute_position(0);
  pos.y = absolute_position(1);
  pos.yaw = absolute_rotation;
  angleCorrection(pos, laser_proxy, last_scan);
 
  absolute_position(0) = pos.x;
  absolute_position(1) = pos.y;
  absolute_rotation = gsl_sf_angle_restrict_pos(pos.yaw);  

  cout << "Corrected Position: " << absolute_position(0) << "," << absolute_position(1) << " Yaw: " << absolute_rotation << endl;
}

void ExaBot::get_pose(gsl::vector& absolute_position, double& absolute_rotation) {    
  absolute_position(0) = position_proxy.GetXPos();
  absolute_position(1) = position_proxy.GetYPos();
  absolute_rotation = gsl_sf_angle_restrict_pos(position_proxy.GetYaw());
  cout << "Position: " << absolute_position(0) << "," << absolute_position(1) << " Yaw: " << absolute_rotation << endl;  

  //#ifdef ENABLE_SYROTEK
  if (laser_proxy.IsFresh() && laser_proxy.IsValid()) correct_pose(absolute_position, absolute_rotation);
  //#endif
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
  if ((trajectory.back() - absolute_position).norm2() > 0.1) {
    trajectory.push_back(absolute_position);
  }
}

