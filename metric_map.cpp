#include <iostream>
#include <map>
#include <cmath>
#include <gsl/gsl_sf_trig.h>
#include <cv.h>
#include <opencv/cxcore.hpp>
#include <plotter.h>
#include "metric_map.h"
#include "util.h"
using namespace HybNav;
using namespace std;
using PlayerCc::LaserProxy;
using PlayerCc::Position2dProxy;

uint MetricMap::WINDOW_CELLS = 39;
uint MetricMap::WINDOW_HALF_CELLS = ((WINDOW_CELLS - 1)/2);
double MetricMap::ROBOT_RADIUS = 0.16;

/**************************
 * Constructor/Destructor *
 **************************/

MetricMap::MetricMap(void) : Singleton<MetricMap>(this),
  position(2), window(WINDOW_CELLS, WINDOW_CELLS), debug_window(WINDOW_CELLS, WINDOW_CELLS), temporary_matrix(2 * Place::CELLS, 2 * Place::CELLS)
{
  double center_pos = (Place::SIZE / 2.0);
  position.set_all(center_pos); // starting at the center of the first place
  rotation = 0.0;

  // add the initial metric node to the metric graph
  gsl::vector_int zero_pos(2, true);
  shared_ptr<Node> new_node(new Node(zero_pos));
  graph.add_node(new_node);
  current_node = new_node.get();
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

void MetricMap::update_position(const gsl::vector& delta_pos, double delta_rot) {
  cout << "in-node position before: " << position << endl;
  cout << "Delta pos: " << delta_pos << " delta rot: " << delta_rot << endl;
  position += delta_pos;
  rotation += delta_rot * (180.0 / M_PI); // TODO: convert to radians!
  rotation = remainder(rotation, 360.0);
  cout << "in-node position now: " << position << endl;

  if (!in_place(position)) {
    cout << "left current metric node, adding new one" << endl;
    gsl::vector_int delta(2);
    for (uint i = 0; i < 2; i++) {
      delta(i) = (int)floor(position(i) / Place::SIZE);
      position(i) = (position(i) < 0 ? position(i) + Place::SIZE : fmod(position(i),Place::SIZE));
    }

    cout << "corrected position: " << position << endl;

    cout << "Place delta: " << delta(0) << "," << delta(1) << endl;
    Node* new_node = get_node(delta);

    if (delta.norm2() > 1)
      cout << "Reached node in diagonal, not adding edge";
    else {
      if (!graph.is_connected(current_node, new_node)) {
        graph.connect(current_node, new_node);
      }
    }
    current_node = new_node;
    cout << "Current node is now: " << new_node->position(0) << "," << new_node->position(1) << endl;
  }
}

MetricMap::Node* MetricMap::get_node(const gsl::vector_int& delta) {
  cout << "get node: (" << delta(0) << "," << delta(1) << ")" << endl;
  if (delta(0) == 0 && delta(1) == 0) return current_node;
  else {
    gsl::vector_int neighbor_position(current_node->position);
    neighbor_position += delta;
    
    Node* neighbor = find_node(neighbor_position);
    Node* node;
    if (!neighbor) {
      cout << "new node at " << neighbor_position << endl;
      shared_ptr<Node> new_node(new Node(neighbor_position));
      graph.add_node(new_node);
      node = new_node.get();
    }
    else {
      node = neighbor;
    }

    return node;
  }
}

MetricMap::Node* MetricMap::find_node(const gsl::vector_int& position) {
  for (Graph<Node>::NodeIterator it = graph.nodes.begin(); it != graph.nodes.end(); ++it) {
    if ((*it)->position == position) return it->get();
  }
  return NULL;
}

void MetricMap::process_distances(Position2dProxy& position_proxy, LaserProxy& laser_proxy)
{
  /* Applies the window to the corresponding places
   * The following coordinates of the window are computed, relative to the current robot position inside the current place
   * -----
   * | v |
   * *----
   * (x0,y0)
  */
  gsl::vector_int v = Place::world2grid(position);
  cout << "v: " << v << " half cells: " << WINDOW_HALF_CELLS << endl;
  double x0 = v(0) - (int)WINDOW_HALF_CELLS;
  double y0 = v(1) - (int)WINDOW_HALF_CELLS;
  double window_x0 = (x0 < 0 ? x0 + Place::CELLS : x0);
  double window_y0 = (y0 < 0 ? y0 + Place::CELLS : y0);

  double temporary_matrix_size[] = { 1,1 };
  if (window_x0 + WINDOW_CELLS > Place::CELLS) temporary_matrix_size[0] = 2;
  if (window_y0 + WINDOW_CELLS > Place::CELLS) temporary_matrix_size[1] = 2;

  /* Determine which places need to be affected by window */
  map<NodeDirections, Node*> affected_nodes;
  if (temporary_matrix_size[0] == 1 && temporary_matrix_size[1] == 1)
    affected_nodes[NorthWest] = get_node(0,0);
  else if (temporary_matrix_size[0] == 2 && temporary_matrix_size[1] == 1) {
    if (x0 < 0) {
      // Nodes: (-1,0)(0,0)
      affected_nodes[NorthWest] = get_node(-1,0);
      affected_nodes[NorthEast] = get_node( 0,0);
    }
    else {
      // Nodes: (0,0)(1,0)
      affected_nodes[NorthWest] = get_node(0,0);
      affected_nodes[NorthEast] = get_node(1,0);
    }
  }
  else if (temporary_matrix_size[0] == 1 && temporary_matrix_size[1] == 2) {
    if (y0 < 0) {
      // Nodes:
      // (0,0)
      // (0,-1)
      affected_nodes[NorthWest] = get_node(0, 0);
      affected_nodes[SouthWest] = get_node(0,-1);
    }
    else {
      // Nodes:
      // (0,1)
      // (0,0)
      affected_nodes[NorthWest] = get_node(0,1);
      affected_nodes[SouthWest] = get_node(0,0);
    }
  }
  else {
    if (x0 < 0) {
      if (y0 < 0) {
        // Nodes:
        // (-1, 0)(0, 0)
        // (-1,-1)(0,-1)
        affected_nodes[NorthWest] = get_node(-1, 0);
        affected_nodes[NorthEast] = get_node( 0, 0);
        affected_nodes[SouthWest] = get_node(-1,-1);
        affected_nodes[SouthEast] = get_node( 0,-1);
      }
      else {
        // Nodes:
        // (-1, 1)(0, 1)
        // (-1, 0)(0, 0)
        affected_nodes[NorthWest] = get_node(-1, 1);
        affected_nodes[NorthEast] = get_node( 0, 1);
        affected_nodes[SouthWest] = get_node(-1, 0);
        affected_nodes[SouthEast] = get_node( 0, 0);
      }
    }
    else {
      if (y0 < 0) {
        // Nodes:
        // (0, 0)(1, 0)
        // (0,-1)(1,-1)
        affected_nodes[NorthWest] = get_node(0, 0);
        affected_nodes[NorthEast] = get_node(1, 0);
        affected_nodes[SouthWest] = get_node(0,-1);
        affected_nodes[SouthEast] = get_node(1,-1);
      }
      else {
        // Nodes:
        // ( 0, 1)(1, 1)
        // ( 0, 0)(1, 0)
        affected_nodes[NorthWest] = get_node(0,1);
        affected_nodes[NorthEast] = get_node(1,1);
        affected_nodes[SouthWest] = get_node(0,0);
        affected_nodes[SouthEast] = get_node(1,0);
      }
    }
  }
  
  // compute views of temporary matrix
  gsl_matrix_view views[4];
  views[NorthWest] = gsl_matrix_submatrix(temporary_matrix.gslobj(), 0, 0, Place::CELLS, Place::CELLS);
  views[NorthEast] = gsl_matrix_submatrix(temporary_matrix.gslobj(), 0, Place::CELLS, Place::CELLS, Place::CELLS);
  views[SouthWest] = gsl_matrix_submatrix(temporary_matrix.gslobj(), Place::CELLS, 0, Place::CELLS, Place::CELLS);
  views[SouthEast] = gsl_matrix_submatrix(temporary_matrix.gslobj(), Place::CELLS, Place::CELLS, Place::CELLS, Place::CELLS);
  
  // load affected node's values into temporary matrix using the corresponding views
  for (map<NodeDirections, Node*>::iterator it = affected_nodes.begin(); it != affected_nodes.end(); ++it) {
    gsl_matrix_memcpy(&views[it->first].matrix, it->second->place.occupancy_matrix.gslobj());
  }

  // load temporary matrix into window through another view
  uint view_y = (temporary_matrix_size[1] == 1 ? Place::CELLS - (window_y0 + WINDOW_CELLS) : 2 * Place::CELLS - (window_y0 + WINDOW_CELLS) + 1);
  cout << "v: " << v(0) << "," << v(1) << " x0,y0: " << x0 << "," << y0 << " win: " << window_x0 << "," << window_y0 << " view: " << view_y << "," << window_x0 << endl;
  gsl_matrix_view temporary_view = gsl_matrix_submatrix(temporary_matrix.gslobj(), view_y, window_x0, WINDOW_CELLS, WINDOW_CELLS);
  gsl_matrix_memcpy(window.gslobj(), &temporary_view.matrix);

  // Update window (add/substract values to it)
  update_window(position_proxy, laser_proxy);

  // Apply window to temporary matrix
  gsl_matrix_memcpy(&temporary_view.matrix, window.gslobj());

  // Set values from temporary matrix to affected nodes
  for (map<NodeDirections, Node*>::iterator it = affected_nodes.begin(); it != affected_nodes.end(); ++it) {
    gsl_matrix_memcpy(it->second->place.occupancy_matrix.gslobj(), &views[it->first].matrix);
  }
}

void MetricMap::update_window(Position2dProxy& position_proxy, LaserProxy& laser_proxy)
{
  double max_range = 8;

  // create a cv::Mat out of the window
  cv::Mat_<double> cv_window(WINDOW_CELLS, WINDOW_CELLS);
  cv_window = 0;

  // create a free circular area around robot
  cv::circle(cv_window, cv::Point(WINDOW_HALF_CELLS, WINDOW_HALF_CELLS), (int)floor(5 * ROBOT_RADIUS / Place::CELL_SIZE), Place::Lfree, -1);

  // create a free polygonal area between robot and each sensed point
  size_t laser_samples = laser_proxy.GetCount();
  cout << "laser samples: " << laser_samples << endl;
  list<cv::Point> pts(laser_samples);
  vector<cv::Point> pts2(laser_samples);

  for (size_t i = 0; i < laser_samples; i++) {
    double angle = laser_proxy.GetBearing(i) + position_proxy.GetYaw();
    double dist = laser_proxy.GetRange(i);
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    pts2[i].x = (int)floor(x / Place::CELL_SIZE) + WINDOW_HALF_CELLS;
    pts2[i].y = (int)round(-y / Place::CELL_SIZE) + WINDOW_HALF_CELLS; // the fillConvexPoly requires the angles to go CCW for some reason
    cout << "x,y: " << pts2[i].x << "," << pts2[i].y << " " << WINDOW_HALF_CELLS << endl;
    if (dist < max_range) {
      cv::Point p;
      p.x = pts2[i].x;
      p.y  = (int)round(y / Place::CELL_SIZE) + WINDOW_HALF_CELLS;
      if (p.x >= 0 && p.y >= 0 && (uint)p.x < WINDOW_CELLS && (uint)p.y < WINDOW_CELLS) pts.push_back(p);
    }
  }
  fillConvexPoly(cv_window, &pts2[0], laser_samples, Place::Lfree, 4);

  // mark individual cells as occupied for each sensed point
  for (list<cv::Point>::const_iterator it = pts.begin(); it != pts.end(); ++it)
    cv_window(WINDOW_CELLS - it->y - 1, it->x) = Place::Locc * 0.8;

  // add the results into the window
  for (uint i = 0; i < window.size1(); i++) {
    for (uint j = 0; j < window.size2(); j++) {
      window(i,j) += cv_window(i,j);
      if (window(i,j) >= Place::Locc) window(i,j) = Place::Locc;
      else if (window(i,j) <= Place::Lfree) window(i,j) = Place::Lfree;
    }
  }
}

bool MetricMap::in_place(const gsl::vector& coord) {
  for (uint i = 0; i < 2; i++) {
    if (coord(i) < 0 || coord(i) >= Place::SIZE) return false;
  }
  return true;
}

gsl::vector_int MetricMap::grid_position(void) {
  return Place::world2grid(position);
}

void MetricMap::save(void) {
  Plotter& p = *Plotter::instance();
  p << "set term svg";
  p << "set xrange [0:" + to_s(Place::CELLS) + "]";
  p << "set yrange [0:" + to_s(Place::CELLS) + "]";
  p << "unset xtics; unset ytics; unset ztics; unset key; unset colorbox";
  p << "set palette gray negative";
  p << "set cbrange [" + to_s(Place::Lfree) + ":" + to_s(Place::Locc) + "]";

  list<string> map_files(glob("csv/grid.*.svg"));
  for (list<string>::iterator it = map_files.begin(); it != map_files.end(); ++it) unlink(it->c_str());

  for (Graph<Node>::NodeIterator it = graph.nodes.begin(); it != graph.nodes.end(); ++it) {
    string svg_name = "csv/grid." + to_s((*it)->position(0)) + "." + to_s((*it)->position(1)) + ".svg";
    p << "set output \"" + svg_name + "\"";
    cout << "guardando " << svg_name << endl;
    p.plot(Plot((*it)->place.occupancy_matrix, "image", "flipy origin=(0.5,0.5)"));
  }

  ofstream dot_file("csv/metric_map.dot", ios_base::trunc | ios_base::out);
  graph.to_dot(dot_file);
  dot_file.close();
}