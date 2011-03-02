#include <limits>
#include <iostream>
#include <gslwrap/vector_int.h>
#include "metric_map.h"
#include "local_explorer.h"
#include "global_explorer.h"
#include "occupancy_grid.h"
#include "motion_planner.h"
#include "util.h"
using namespace HybNav;
using namespace std;

/* constants */
double OccupancyGrid::CELL_SIZE = 0.07;
uint OccupancyGrid::CELLS = 51;
double OccupancyGrid::SIZE = OccupancyGrid::CELL_SIZE * OccupancyGrid::CELLS;
double OccupancyGrid::Locc = 1.5;
double OccupancyGrid::Lfree = -1.5;
uint OccupancyGrid::GATEWAY_LOOKAHEAD_CELLS = (uint)(1.0 / OccupancyGrid::CELL_SIZE);

/**************************
 * Constructor/Destructor *
 **************************/

OccupancyGrid::OccupancyGrid(size_t x, size_t y) : position(2), m(CELLS, CELLS, true)
{
  position(0) = x; position(1) = y;
}

/**************************
 *     Public Methods     *
 **************************/

gsl::vector_int OccupancyGrid::rowcol2xy(gsl::vector_int rowcol) {
  gsl::vector_int xy(2);
  xy(0) = rowcol(1);
  xy(1) = CELLS - rowcol(0) - 1;
  return xy;
}

double& OccupancyGrid::operator()(uint x, uint y) {
  return m(CELLS - y - 1, x);
}

bool OccupancyGrid::valid_coordinates(int x, int y) {
  return (x >= 0 && (uint)x < CELLS && y >= 0 && (uint)y < CELLS);
}

gsl::vector_int OccupancyGrid::world2grid(const gsl::vector& coord) {
  gsl::vector_int out(2);
  for (uint i = 0; i < 2; i++) {
    out(i) = (uint)floor(coord(i) / CELL_SIZE);
  }
  return out;
}

TopoMap::GatewayNode* OccupancyGrid::find_gateway(gsl::vector_int pos, Direction edge) {
  double distance = numeric_limits<double>::max();
  TopoMap::GatewayNode* closest_node = NULL;
  for (list<TopoMap::GatewayNode*>::iterator it = gateway_nodes.begin(); it != gateway_nodes.end(); ++it) {
    if ((*it)->edge == edge) {
      gsl::vector_int it_position = (*it)->position();
      it_position -= pos;
      distance = min(distance, it_position.norm2());
      closest_node = *it;
    }
  }
  if (!closest_node) throw std::runtime_error("No gateway found for that edge");
  else return closest_node;
}

void OccupancyGrid::update_gateways(bool and_connectivity) {
  GatewayCoordinates gateway_coordinates = detect_gateways();

  // Find GW nodes (and update coordinates, if necessary) which cover detected gateways
  list<TopoMap::GatewayNode*>::iterator it = gateway_nodes.begin();
  while (it != gateway_nodes.end()) {
    list< pair<uint,uint> >& edge_coordinates = gateway_coordinates[(*it)->edge];
    list< pair<uint,uint> >::iterator match_it = edge_coordinates.end();
    // find a gw coordinate covered by the gw node
    for (list< pair<uint,uint> >::iterator it2 = edge_coordinates.begin(); it2 != edge_coordinates.end(); ++it2) {
      if (it2->first <= (*it)->x0 || (*it)->xf <= it2->second) { match_it = it2; break; } // TODO: this can leave duplicated gateways, they should be merged (notice the ||)
    }
    // a coordinate was found, so update the gw node's dimensions (in case it changed) and consider it
    // covered by deleting it from the coord list
    if (match_it != edge_coordinates.end()) {
      (*it)->set_dimensions(match_it->first, match_it->second);
      edge_coordinates.erase(match_it);
      ++it;
    }
    // else, this gw node is not valid anymore, so delete it from the topo map and from the list of topo nodes associated
    // with this occupancy grid
    else {
      list<TopoMap::Node*>& follow_path = GlobalExplorer::instance()->follow_path;
      if (find(follow_path.begin(), follow_path.end(), *it) != follow_path.end()) {
        cout << "deleted gateway node " << *it << " which is present in follow path, so clearing that" << endl;
        GlobalExplorer::instance()->clear_paths();
      }
      TopoMap::instance()->del_node(*it);
      it = gateway_nodes.erase(it);

    }
  }

  // and now, add new GW nodes to cover the remaining detected gateway coordinates
  for (uint i = 0; i < 4; i++) {
    list< pair<uint,uint> >& edge_coordinates = gateway_coordinates[i];
    for (list< pair<uint,uint> >::iterator it = edge_coordinates.begin(); it != edge_coordinates.end(); ++it) {
      cout << "adding gateway node for [" << it->first << "," << it->second << "] at grid " << position << endl;
      TopoMap::GatewayNode* new_gateway = TopoMap::instance()->add_gateway(this, (Direction)i, it->first, it->second);
      gateway_nodes.push_back(new_gateway);
    }
  }

  if (MetricMap::instance()->current_grid == this && and_connectivity) update_connectivity();
}

void OccupancyGrid::update_connectivity(void) {
  cout << "Updating connectivity for " << position << endl;
  TopoMap::Node* current_topo_node = TopoMap::instance()->current_node;
  gsl::vector_int start_position(2);
  if (current_topo_node->is_area())
    start_position = MetricMap::instance()->grid_position();
  else
    start_position = ((TopoMap::GatewayNode*)current_topo_node)->position();

  cout << "current topo node: " << current_topo_node << endl;

  for (list<TopoMap::GatewayNode*>::iterator it = gateway_nodes.begin(); it != gateway_nodes.end(); ++it) {
    gsl::vector_int it_position = (*it)->position();
    cout << "finding connectivity to " << it_position(0) << "," << it_position(1) << endl;
    if (LocalExplorer::instance()->connectivity_pathfinder.exists_path(start_position, it_position)) {
      if (current_topo_node->is_area())
        TopoMap::instance()->connect(TopoMap::instance()->current_node, *it);
      else {
        TopoMap::AreaNode* area_node = (*it)->area_node();
        if (area_node) TopoMap::instance()->connect(TopoMap::instance()->current_node, area_node);
      }
      cout << "connected" << endl;
    }
    else {
      if (current_topo_node->is_area())
        TopoMap::instance()->disconnect(TopoMap::instance()->current_node, *it);
      else {
        TopoMap::AreaNode* area_node = (*it)->area_node();
        if (area_node) TopoMap::instance()->disconnect(TopoMap::instance()->current_node, area_node);
      }
      cout << "not connected" << endl;
    }
  }

  if (current_topo_node->is_gateway()) {
    TopoMap::AreaNode* area_of_current = ((TopoMap::GatewayNode*)current_topo_node)->area_node();
    if (!area_of_current) {
      TopoMap::AreaNode* new_area = TopoMap::instance()->add_area(this);
      TopoMap::instance()->connect(current_topo_node, new_area);
      TopoMap::instance()->current_node = new_area;
      cout << "adding new area node" << endl;
    }
    else
      TopoMap::instance()->current_node = area_of_current;
  }
}

Direction OccupancyGrid::direction_to(OccupancyGrid* other) {
  gsl::vector_int direction_vector = other->position;
  direction_vector -= this->position;
  cout << "direction vector: " << direction_vector << endl;
  return MetricMap::vector2direction(direction_vector);
}

void OccupancyGrid::to_dot(std::ostream& out) {
  gsl::vector_int node_position = position;
  node_position *= 2;

  string node_position_str = to_s(node_position(0)) + "," + to_s(node_position(1)) + "!";
  string svg_name = "./grid." + to_s(position(0)) + "." + to_s(position(1)) + ".svg";
  out << "label=\"" << position << "\" pos=\"" << node_position_str << "\" image=\"" <<
    svg_name << "\" shape=\"box\" imagescale=\"true\" width=\"1.7\" height=\"1.7\" fixedsize=\"true\" labelloc=\"b\"";
}

/**************************
 *    Private Methods     *
 **************************/

bool OccupancyGrid::gateway_condition(uint i, uint j, uint d) {
  if (m(i,j) >= 0) return false; // this cell needs to be free

  // also, cells in the neighboring grid (up to a certain distance) need also to be free
  gsl::vector_int dir_vec = position + MetricMap::direction2vector((Direction)d);
  OccupancyGrid& neighbor = MetricMap::instance()->super_matrix.submatrix(dir_vec(0), dir_vec(1));

  cout << "neighbor: " << neighbor.position << endl;
  int posneg = (d == North || d == West ? -1 : 1);
  int dim = (d == North || d == South ? i : j);
  for (int k = posneg; abs<int>(k) < GATEWAY_LOOKAHEAD_CELLS; k += posneg) {
    int kk = dim + k;
    if (kk < 0) kk += CELLS;
    else if (kk >= CELLS) kk %= CELLS;
    cout << "kk " <<  kk << " dim " << dim << " k " << k << endl;

    double neighbor_v;
    if (d == North || d == South) neighbor_v = neighbor.m(kk, j);
    else neighbor_v = neighbor.m(i, kk);
    cout << "value: " << neighbor_v << endl;
    if (neighbor_v > 0) return false;
  }

  return true;
}

vector< list< pair<uint, uint> > > OccupancyGrid::detect_gateways(void) {
  cout << "Gateways detected for " << position << ":" << endl;
  GatewayCoordinates gateways(4); // for each edge, a list of x0,xf of each gateway

  for (uint d = 0; d < 4; d++) {
    bool in_gateway = false;
    for (size_t k = 0; k < m.size1(); k++) {
      uint i = (d == East || d == West ? k : (d == North ? 0 : OccupancyGrid::CELLS - 1));
      uint j = (d == South || d == North ? k : (d == East ? OccupancyGrid::CELLS - 1 : 0));

      if (gateway_condition(i, j, d)) {
        if (!in_gateway) { gateways[d].push_back(pair<uint,uint>(k,k)); in_gateway = true; }
        if (k == m.size1() - 1 && in_gateway) { gateways[d].back().second = k; in_gateway = false; }
      }
      else {
        cout << "not a gateway cell: " << i << "," << j << " d: " << d << endl;
        if (in_gateway) { gateways[d].back().second = k - 1; in_gateway = false; }
      }
    }
  }

  /* filter out too small gateways and rearrange coordinates for valid gateways */
  for (uint d = 0; d < 4; d++) {
    cout << "\t" << (Direction)d << ":";
    list< pair<uint,uint> >::iterator it = gateways[d].begin();
    while (it != gateways[d].end()) {
      if (it->second - it->first < ceil((MotionPlanner::instance()->MINIMUM_DISTANCE) / OccupancyGrid::CELL_SIZE))
        it = gateways[d].erase(it);
      else {
        uint i, j;
        if (d == East || d == West) { i = OccupancyGrid::CELLS - 1 - it->second; j = OccupancyGrid::CELLS - 1 - it->first; }
        else { i = it->first; j = it->second; }
        cout << " [" << i << ":" << j << "]";
        it->first = i; it->second = j;
        ++it;
      }
    }
    cout << endl;
  }

  return gateways;
}

std::ostream& operator<<(std::ostream& out, const HybNav::OccupancyGrid* grid) {
  out << grid->position;
  return out;
}


