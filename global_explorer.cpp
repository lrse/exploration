#include <iostream>
#include "metric_map.h"
#include "global_explorer.h"
using namespace HybNav;
using namespace std;

std::list<TopoMap::Node*> GlobalExplorer::GlobalPathfinder::neighbors(TopoMap::Node* const & current) {
  return current->neighbors();
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