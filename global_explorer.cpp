#include <iostream>
#include "metric_map.h"
#include "global_explorer.h"
#include "local_explorer.h"
using namespace HybNav;
using namespace std;

struct ExplorationCost : public binary_function<TopoMap::Node*,TopoMap::Node*,bool> {
  TopoMap::Node* from, * previous;
  ExplorationCost(TopoMap::Node* _from, TopoMap::Node* _previous) : from(_from), previous(_previous) { }

  bool operator()(TopoMap::Node* const& a, TopoMap::Node* const& b) {
    bool result;
    if (((TopoMap::GatewayNode*)a)->unexplored_gateway() && !((TopoMap::GatewayNode*)b)->unexplored_gateway()) {
      cout << a << " is unexplored, so it is better than " << b << endl; 
      result = true;
    }
    else if (!((TopoMap::GatewayNode*)a)->unexplored_gateway() && ((TopoMap::GatewayNode*)b)->unexplored_gateway()) {
      cout << a << " is explored, so it is worst than " << b << endl; 
      result = false;
    }
    else {
      gsl::vector_int start_position = (from == TopoMap::instance()->current_node ? MetricMap::instance()->grid_position() : ((TopoMap::GatewayNode*)previous)->position());
      // TODO: this is very expensive, although the distance could be cached when testing for connectivity or at least cached
      /*list< list<gsl::vector_int> > paths_to_a = LocalExplorer::instance()->connectivity_pathfinder.findpath(start_position, ((TopoMap::GatewayNode*)a)->position(), true);
      list< list<gsl::vector_int> > paths_to_b = LocalExplorer::instance()->connectivity_pathfinder.findpath(start_position, ((TopoMap::GatewayNode*)b)->position(), true);
      unsigned long length_a = (paths_to_a.empty() ? 0 : paths_to_a.front().size());
      unsigned long length_b = (paths_to_b.empty() ? 0 : paths_to_b.front().size());*/
      double length_a = (start_position - ((TopoMap::GatewayNode*)a)->position()).norm2();
      double length_b = (start_position - ((TopoMap::GatewayNode*)b)->position()).norm2();
      cout << a << " is " << (length_a < length_b ? "closer" : "farther") << " than " << b << endl;
      result = (length_a < length_b);
    }
    return !result; // sorts from worst to best, since the order is reverses in pathfinder when the priorities are equal
  }
};

std::list<TopoMap::Node*> GlobalExplorer::GlobalPathfinder::neighbors(TopoMap::Node* const & current, TopoMap::Node* const& previous) {
  list<TopoMap::Node*> neighbors = current->neighbors();
  if (current->is_area()) {
    cout << "sorting neighbors of " << current << endl;
    neighbors.sort(ExplorationCost(current, previous));
    cout << "result: " << neighbors << endl;
  }
  return neighbors;
}

bool GlobalExplorer::GlobalPathfinder::is_goal(TopoMap::Node* const & node) {
  bool goal = (node != TopoMap::instance()->current_node) && ((node->is_area() && !((TopoMap::AreaNode*)node)->completely_explored) ||
    (node->is_gateway() && !((TopoMap::GatewayNode*)node)->is_inaccessible() && ((TopoMap::GatewayNode*)node)->unexplored_gateway()));
  gsl::vector_int node_position;
  cout << "node " << node << " is goal: " << boolalpha << goal << endl;
  return goal;
}

GlobalExplorer::GlobalExplorer(void) : Singleton<GlobalExplorer>(this){
  clear_paths();
}

void GlobalExplorer::follow_next_path(void) {
  follow_path = all_paths.front();
  all_paths.pop_front();
  follow_path.pop_front();
}

void GlobalExplorer::clear_paths(void) {
  all_paths.clear();
  follow_path.clear();
  found = false;
}

bool GlobalExplorer::found_path(void) {
  return (found || other_paths_left());
}

bool GlobalExplorer::other_paths_left(void) {
  return !all_paths.empty();
}

void GlobalExplorer::recompute_paths(void) {
  MetricMap::instance()->current_grid->update_gateways();

  all_paths = pathfinder.findpath(TopoMap::instance()->current_node);
  if (!all_paths.empty()) { found = true; follow_next_path(); }
  else found = false;

  //cout << "Global all paths: " << all_paths << " follow " << follow_path << endl;
}