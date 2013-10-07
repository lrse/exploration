#include "explorer.h"
#include "local_explorer.h"
#include "exabot.h"
using namespace HybNav;
using namespace std;

LocalPathfinder::LocalPathfinder(double v) : frontier_value_condition(v) {
  grid.create(OccupancyGrid::CELLS, OccupancyGrid::CELLS, CV_8UC1);
  cost_grid.create(OccupancyGrid::CELLS, OccupancyGrid::CELLS, CV_8UC1);
  create_cost_brush();
}

void LocalPathfinder::create_cost_brush(void) {
  brush_size = 21;
  half_brush_size = brush_size / 2;
  brush.create(brush_size, brush_size, CV_8UC1);
  
  uchar* bptr = brush.ptr(0);
  for (int i = 0; i < brush_size; i++) {
    for (int j = 0; j < brush_size; j++, bptr++) {
      double d = hypot(i - half_brush_size, j - half_brush_size) / half_brush_size;
      *bptr = ceil(255 * (d > 1 ? 1 : d));
    }
  }
}

list<gsl::vector_int> LocalPathfinder::neighbors(const gsl::vector_int& v, const gsl::vector_int& previous) {
  list<gsl::vector_int> neighbors;
  gsl::vector_int n(2);
  if ((uint)v(1) < OccupancyGrid::CELLS - 1 && get_occupancy(v[0],v[1]+1) == 255) { n = v; n(1) += 1; neighbors.push_back(n); } // up
  if ((uint)v(0) > 0                        && get_occupancy(v[0]-1,v[1]) == 255) { n = v; n(0) -= 1; neighbors.push_back(n); } // right
  if ((uint)v(0) < OccupancyGrid::CELLS - 1 && get_occupancy(v[0]+1,v[1]) == 255) { n = v; n(0) += 1; neighbors.push_back(n); } // left
  if ((uint)v(1) > 0                        && get_occupancy(v[0],v[1]-1) == 255) { n = v; n(1) -= 1; neighbors.push_back(n); } // down
  //cout << "neighbors of: " << v(0) << "," << v(1);
  //LocalExplorer::instance()->print_path(neighbors);
  //cout << endl;
  return neighbors;
}

unsigned long LocalPathfinder::movement_cost(const gsl::vector_int& from, const gsl::vector_int& to, const gsl::vector_int& previous) {
  unsigned long cost = 1;

  int safety_radius_cells = ceil(ExaBot::ROBOT_RADIUS * 0.7 / OccupancyGrid::CELL_SIZE);

  // avoid grid edges
  int x = to(0);
  int y = to(1);
  if ((x < safety_radius_cells) || (x > OccupancyGrid::CELLS - safety_radius_cells - 1) ||
      (y < safety_radius_cells) || (y > OccupancyGrid::CELLS - safety_radius_cells - 1))
    cost += 15;

  // if there's a previous node in the path
  if (previous != from) {
    gsl::vector_int old2a(from);
    old2a -= previous;
    gsl::vector_int a2b(to);
    a2b -= from;
    if (a2b != old2a) cost += 10; // jaggy paths cost more
  }
  
  cost += (1 - (float)get_cost(x, y) / 255) * 20;
  
  return cost;
}

void LocalPathfinder::prepare(void) {
  process_current_grid();
}



void LocalPathfinder::process_current_grid(void) {
  OccupancyGrid& current_grid = *MetricMap::instance()->current_grid;
  grid = 255;
  
  // grow grid
  for (uint i = 0; i < OccupancyGrid::CELLS; i++) {
    for (uint j = 0; j < OccupancyGrid::CELLS; j++) {
      if (current_grid(i,j) >= frontier_value_condition)
        cv::circle(grid, cv::Point(i,OccupancyGrid::CELLS - j - 1),
          floor((ExaBot::ROBOT_RADIUS - MetricMap::SENSOR_MODEL_DELTA) * 0.8 / OccupancyGrid::CELL_SIZE), 0, -1, 4);
    }
  }
  
  // create cost grid
  grid.copyTo(cost_grid);
  for (int i = 0; i < OccupancyGrid::CELLS; i++) {
    for (int j = 0; j < OccupancyGrid::CELLS; j++) {
      if (grid.at<uchar>(i,j) == 255) continue;
      
      cv::Rect output_rect(j - half_brush_size, i - half_brush_size, brush_size, brush_size);
      cv::Mat brush_ref = brush;
      
      // detect borders
      if (i - half_brush_size < 0 || j - half_brush_size < 0 ||
          i + half_brush_size >= OccupancyGrid::CELLS || j + half_brush_size >= OccupancyGrid::CELLS)
      {
        cv::Rect output_rect2 = output_rect & cv::Rect(0, 0, OccupancyGrid::CELLS, OccupancyGrid::CELLS);
        cv::Rect brush_rect = cv::Rect(output_rect2.x - output_rect.x, output_rect2.y - output_rect.y, output_rect2.width, output_rect2.height);
        output_rect = output_rect2;
        brush_ref = brush(brush_rect);
      }
      
      cv::Mat subgrid(cost_grid, output_rect);
      cv::min(subgrid, brush_ref, subgrid);
    }
  }
}

uchar LocalPathfinder::get_occupancy(uint i, uint j) {
  return grid.at<uchar>(OccupancyGrid::CELLS - j - 1,i);
}

uchar LocalPathfinder::get_cost(uint i, uint j) {
  return cost_grid.at<uchar>(OccupancyGrid::CELLS - j - 1,i);
}

ConnectivityPathfinder::ConnectivityPathfinder(void) : LocalPathfinder(0), x_range(2), y_range(2) { }

bool ConnectivityPathfinder::is_goal(const gsl::vector_int& pos) {
  bool result = (pos(0) >= x_range(0) && pos(0) <= x_range(1) && pos(1) >= y_range(0) && pos(1) <= y_range(1));
  //cout << "is goal? " << pos(0) << "," << pos(1) << "gw [" << x_range(0) << "," << x_range(1) << "] [" << y_range(0) << "," << y_range(1) << " -> " << boolalpha << result << endl;
  return result;
}

FrontierPathfinder::FrontierPathfinder(void) : LocalPathfinder(OccupancyGrid::Locc * 0.2) { }

bool FrontierPathfinder::is_goal(const gsl::vector_int& v) {
  OccupancyGrid::FrontierList& frontiers = MetricMap::instance()->current_grid->frontiers;
  OccupancyGrid::FrontierList::iterator it = find(frontiers.begin(), frontiers.end(), v);
  bool result = (it != frontiers.end());
  //cout << "is frontier? " << v(0) << "," << v(1) << ": " << boolalpha << result << endl;
  return result;
}

LocalExplorer::LocalExplorer(void) : Singleton<LocalExplorer>(this), last_target(2) {
  clear_paths();
  last_target_valid = false;
}

bool LocalExplorer::target_is_frontier(void) {
  return (follow_path.size() > 0 && frontier_pathfinder.is_goal(follow_path.back()));
}


struct DistanceCost : public binary_function<list<gsl::vector_int>,list<gsl::vector_int>,bool> {
  const gsl::vector_int& last_target;
  DistanceCost(const gsl::vector_int& _last_target) : last_target(_last_target) { }

  bool operator()(const list<gsl::vector_int>& a, const list<gsl::vector_int>& b) {
    /*gsl::vector_int last_a = a.back();
    last_a -= last_target;

    gsl::vector_int last_b = b.back();
    last_b -= last_target;

    return (last_a.norm2() < last_b.norm2());*/
    return (a.size() < b.size());
  }
};

void LocalExplorer::update(void) {
  frontier_pathfinder.process_current_grid();
  connectivity_pathfinder.process_current_grid();
}

void LocalExplorer::sort_paths(void) {
  if (last_target_valid) all_paths.sort(DistanceCost(last_target));
}

void LocalExplorer::clear_paths(void) {
  all_paths.clear();
  follow_path.clear();
  found = false;
}

bool LocalExplorer::found_path(void) {
  return found;
}

bool LocalExplorer::other_paths_left(void) {
  return !all_paths.empty();
}

bool LocalExplorer::valid_path(void)
{
  if (follow_path.empty()) return true;
  for (list<gsl::vector_int>::iterator it = follow_path.begin(); it != follow_path.end(); ++it) {
    if (OccupancyGrid::valid_coordinates((*it)(0),(*it)(1))) {
      if ((Explorer::instance()->state == Explorer::ExploringLocally && frontier_pathfinder.get_occupancy((*it)(0), (*it)(1)) == 0) ||
          (Explorer::instance()->state == Explorer::ExploringGlobally && connectivity_pathfinder.get_occupancy((*it)(0), (*it)(1)) == 0))
      {
        cout << "path crosses obstacle" << endl;
        return false;
      }
    }    
  }
  return true;
}

// Given the array of paths, it follows the first path in it and removes it from the given array
void LocalExplorer::follow_next_path(void) {
  follow_path = all_paths.front();
  all_paths.pop_front();
  follow_path.pop_front();
  last_target = follow_path.back();
  last_target_valid = true;
  
  cout << "current follow path: " << follow_path << endl;
  print_all_paths();
}

void LocalExplorer::print_all_paths(void) {
  cout << "all paths (" << all_paths.size() << "): " << endl;
  for (list< list<gsl::vector_int> >::iterator it = all_paths.begin(); it != all_paths.end(); ++it) {
    cout << *it << endl;
  }
}

std::ostream& operator<<(std::ostream& out, const std::list<gsl::vector_int>& l) {
  for (list<gsl::vector_int>::const_iterator it = l.begin(); it != l.end(); ++it) {
    if (it != l.begin()) cout << ", ";
    cout << *it;
  }
  return out;
}

// Computes all possible paths to the detected frontiers. If at least one path is found, it is "followed"
void LocalExplorer::compute_frontier_paths(void) {
  MetricMap::instance()->current_grid->update_frontiers();
  if (MetricMap::instance()->current_grid->frontiers.empty()) { clear_paths(); found = false; }
  else {    
    gsl::vector_int grid_position = MetricMap::instance()->grid_position();
    all_paths = frontier_pathfinder.findpath(grid_position, false);
    if (!all_paths.empty()) {
      sort_paths();
      follow_next_path();
      /*if (follow_path.empty()) found = false;
      else*/ found = true;
    }
    else found = false;
  }
}

void LocalExplorer::compute_gateway_path(TopoMap::GatewayNode* gateway, bool follow) {
  cout << "Looking for paths to gateway" << endl;
  gsl::vector_int start = MetricMap::instance()->grid_position();
  all_paths = connectivity_pathfinder.findpath(start, gateway->position(), true);
  //gateway->get_ranges(connectivity_pathfinder.x_range, connectivity_pathfinder.y_range);
  //all_paths = connectivity_pathfinder.findpath(start, true);

  if (all_paths.empty()) {
    cout << "Movement is impossible" << endl;
    // TODO: break edge?
    found = false;
  }
  else {
    int extra_node_dist = 3;
    gsl::vector_int extra_node(2);
    switch(gateway->edge) {
      case North: extra_node(0) = 0; extra_node(1) = extra_node_dist; break;
      case South: extra_node(0) = 0; extra_node(1) = -extra_node_dist; break;
      case East: extra_node(0) = extra_node_dist; extra_node(1) = 0; break;
      case West: extra_node(0) = -extra_node_dist; extra_node(1) = 0; break;
    }

    // add a ficticious far node at the end of each path, to ensure crossing OccupancyGrids
    for (list< list<gsl::vector_int> >::iterator it = all_paths.begin(); it != all_paths.end(); ++it) {
      gsl::vector_int last = it->back();
      last += extra_node;
      it->push_back(last);
    }

    found = true;
    if (follow) follow_next_path();
  }
}

