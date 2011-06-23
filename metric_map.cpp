#include <iostream>
#include <map>
#include <cmath>
#include <gsl/gsl_sf_trig.h>
#include <cv.h>
#include <opencv/cxcore.hpp>
#include <plotter.h>
#include <sys/stat.h>
#include "metric_map.h"
#include "util.h"
using namespace HybNav;
using namespace std;
using PlayerCc::LaserProxy;
using PlayerCc::Position2dProxy;

// These numbers account for 4m of sensors range (sick has 8m)
uint MetricMap::WINDOW_SIZE_CELLS = 115; // must be odd!
uint MetricMap::WINDOW_RADIUS_CELLS = (MetricMap::WINDOW_SIZE_CELLS - 1) / 2;
double MetricMap::ROBOT_RADIUS = 0.4;

/**************************
 * Constructor/Destructor *
 **************************/

MetricMap::MetricMap(void) : Singleton<MetricMap>(this),
  position(2), window(WINDOW_SIZE_CELLS, WINDOW_SIZE_CELLS, true)
{
  double center_pos = (OccupancyGrid::SIZE / 2.0);
  position.set_all(center_pos); // starting at the center of the first OccupancyGrid
  rotation = 0.0;

  // set the (0,0) grid as current
  current_grid = &super_matrix.submatrix(0,0);
}

/**************************
 *     Public Methods     *
 **************************/

Direction MetricMap::vector2direction(const gsl::vector_int& v) {
  if (v(0) == 0) {
    if (v(1) == 1) return North;
    else if (v(1) == -1) return South;
  }
  else if (v(1) == 0) {
    if (v(0) == 1) return East;
    else if (v(0) == -1) return West;
  }

  throw std::runtime_error("Not an immediate neighbor");
}

gsl::vector_int MetricMap::direction2vector(Direction dir) {
  gsl::vector_int out(2);
  switch(dir) {
    case North: out(0) =  0; out(1) =  1; break;
    case South: out(0) =  0; out(1) = -1; break;
    case East:  out(0) =  1; out(1) =  0; break;
    case West:  out(0) = -1; out(1) =  0; break;
  }
  return out;
}

Direction MetricMap::opposite_direction(Direction dir) {
  switch(dir) {
    case North: return South; break;
    case South: return North; break;
    case West: return East; break;
    case East: return West; break;
  }
}

void MetricMap::update_position(const gsl::vector& delta_pos, double delta_rot) {
  cout << "in-node position before: " << position << endl;
  cout << "Delta pos: " << delta_pos << " delta rot: " << delta_rot << endl;
  position += delta_pos;
  rotation += delta_rot * (180.0 / M_PI); // TODO: convert to radians!
  rotation = remainder(rotation, 360.0);
  cout << "in-node position now: " << position << endl;

  if (!in_grid(position)) {
    cout << "left current metric node, adding new one" << endl;
    gsl::vector_int delta(2);
    for (uint i = 0; i < 2; i++) {
      delta(i) = (int)floor(position(i) / OccupancyGrid::SIZE);
      position(i) = (position(i) < 0 ? position(i) + OccupancyGrid::SIZE : fmod(position(i),OccupancyGrid::SIZE));
    }

    cout << "corrected position: " << position << endl;

    gsl::vector_int new_position = current_grid->position + delta;
    cout << "OccupancyGrid delta: " << delta(0) << "," << delta(1) << endl;
    current_grid = &super_matrix.submatrix(new_position(0), new_position(1));
    cout << "Current node is now: " << current_grid->position << endl;
  }
}

void MetricMap::process_distances(Position2dProxy& position_proxy, LaserProxy& laser_proxy)
{
  // apply sensor readings to window
  double max_range = 4;

  // create a cv::Mat out of the window
  cv::Mat_<double> cv_window(WINDOW_SIZE_CELLS, WINDOW_SIZE_CELLS);
  cv_window = 0;

  // create a free circular area around robot
  cv::circle(cv_window, cv::Point(WINDOW_RADIUS_CELLS, WINDOW_RADIUS_CELLS), (int)floor(ROBOT_RADIUS / OccupancyGrid::CELL_SIZE), OccupancyGrid::Lfree, -1);

  // create a free polygonal area between robot and each sensed point
  size_t laser_samples = laser_proxy.GetCount();
  list<cv::Point> pts;
  vector<cv::Point> pts2(laser_samples);

  for (size_t i = 0; i < laser_samples; i++) {
    double angle = laser_proxy.GetBearing(i) + position_proxy.GetYaw();
    double dist = laser_proxy.GetRange(i);
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    pts2[i].x = (int)floor(x / OccupancyGrid::CELL_SIZE) + (int)WINDOW_RADIUS_CELLS;
    pts2[i].y = (int)floor(-y / OccupancyGrid::CELL_SIZE) + (int)WINDOW_RADIUS_CELLS; // the fillConvexPoly requires the angles to go CCW for some reason
    //cout << "x,y: " << pts2[i].x << "," << pts2[i].y << " " << x << "," << y << "," << angle << endl;
    if (dist < max_range) {
      cv::Point p;
      p.x = pts2[i].x;
      p.y  = (int)floor(y / OccupancyGrid::CELL_SIZE) + (int)WINDOW_RADIUS_CELLS;
      if (p.x >= 0 && p.y >= 0 && (uint)p.x < WINDOW_SIZE_CELLS && (uint)p.y < WINDOW_SIZE_CELLS) pts.push_back(p);
    }
  }
  const cv::Point* ptr = &pts2[0];
  int contours = laser_samples;
  fillPoly(cv_window, &ptr, &contours, 1, OccupancyGrid::Lfree, 4);

  // mark individual cells as occupied for each sensed point
  for (list<cv::Point>::const_iterator it = pts.begin(); it != pts.end(); ++it)
    cv_window(WINDOW_SIZE_CELLS - it->y - 1, it->x) = OccupancyGrid::Locc * 0.8;

  // apply window to corresponding grids
  gsl::vector_int window_offset = current_grid->position * OccupancyGrid::CELLS + grid_position();

  gsl::vector_int xy(2);
  for (size_t i = 0; i < WINDOW_SIZE_CELLS; i++) {
    xy(1) = MetricMap::WINDOW_SIZE_CELLS - i - 1;
    for (size_t j = 0; j < WINDOW_SIZE_CELLS; j++) {
      xy(0) = j;
      gsl::vector_int absolute_cell = window_offset + (xy - WINDOW_RADIUS_CELLS);
      //cout << "abs cell: " << absolute_cell << " " << window_offset << " " << WINDOW_RADIUS_CELLS << " " << xy << endl;
      
      double& v = super_matrix.cell(absolute_cell(0), absolute_cell(1));
      double new_v = cv_window(i, j);
      if (v > 0 && new_v < 0) continue;
      
      v += new_v;
      if (v >= OccupancyGrid::Locc) v = OccupancyGrid::Locc;
      else if (v <= OccupancyGrid::Lfree) v = OccupancyGrid::Lfree;
      window(i, j) = cv_window(i, j); // DEBUG
    }
  }
}

bool MetricMap::in_grid(const gsl::vector& coord) {
  for (uint i = 0; i < 2; i++) {
    if (coord(i) < 0 || coord(i) >= OccupancyGrid::SIZE) return false;
  }
  return true;
}

gsl::vector_int MetricMap::grid_position(void) {
  return OccupancyGrid::world2grid(position);
}

void MetricMap::save(void) {
  Plotter& p = *Plotter::instance();
  p << "set term svg";
  p << "set xrange [0:" + to_s(OccupancyGrid::CELLS) + "]";
  p << "set yrange [0:" + to_s(OccupancyGrid::CELLS) + "]";
  p << "unset xtics; unset ytics; unset ztics; unset key; unset colorbox";
  p << "set palette gray negative";
  p << "set cbrange [" + to_s(OccupancyGrid::Lfree) + ":" + to_s(OccupancyGrid::Locc) + "]";

  mkdir("csv", 0777);
  list<string> map_files(glob("csv/grid.*.svg"));
  for (list<string>::iterator it = map_files.begin(); it != map_files.end(); ++it) unlink(it->c_str());

  for (SuperMatrix<OccupancyGrid>::iterator_x itx = super_matrix.matrix_map.begin(); itx != super_matrix.matrix_map.end(); ++itx) {
    for (SuperMatrix<OccupancyGrid>::iterator_y ity = itx->second.begin(); ity != itx->second.end(); ++ity) {
      OccupancyGrid& g = ity->second;
      string svg_name = "csv/grid." + to_s(g.position(0)) + "." + to_s(g.position(1)) + ".svg";
      p << "set output \"" + svg_name + "\"";
      cout << "guardando " << svg_name << endl;
      p.plot(Plot(g.m, "image", "flipy origin=(0.5,0.5)"));
      usleep(500000); // sleep 0.5s
    }
  }

  ofstream dot_file("csv/metric_map.dot", ios_base::trunc | ios_base::out);
  super_matrix.to_dot(dot_file);
  dot_file.close();
}