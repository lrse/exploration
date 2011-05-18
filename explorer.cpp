#include <gsl/gsl_sf_trig.h>
#include "explorer.h"
#include "global_explorer.h"
#include "local_explorer.h"
#include "util.h"
#include "motion_planner.h"
#include "exabot.h"
using namespace HybNav;
using namespace std;
using PlayerCc::Position2dProxy;
using PlayerCc::PlannerProxy;

/**************************
 * Constructor/Destructor *
 **************************/

Explorer::Explorer(void) : Singleton<Explorer>(this) {
  state = ExploringLocally;
  current_grid = MetricMap::instance()->current_grid;
}


/**************************
 *     Public Methods     *
 **************************/

void Explorer::start(State _state) {
  state = _state;
  if (state == ExploringGlobally) start_exploring_globally();
}

void Explorer::fsm_advance(void) {
  if (state == ExploringLocally) while_exploring_locally();
  else while_exploring_globally();
}

void Explorer::update(void) {
  cout << "Current grid: " << MetricMap::instance()->current_grid->position << endl;
  if (current_grid != MetricMap::instance()->current_grid) {
    ExaBot::instance()->stop(); // stop robot motion during this iteration
    gsl::vector_int new_position = MetricMap::instance()->current_grid->position;
    cout << "Changed places " << current_grid->position << "->" << new_position << endl;

    LocalExplorer::instance()->clear_paths(); // When changing places, the last path is not valid anymore
    LocalExplorer::instance()->last_target_valid = false;

    gsl::vector_int exit_vector = MetricMap::instance()->current_grid->position;
    exit_vector -= current_grid->position;
    if (exit_vector.norm2() > 1) {
      // TODO!!
    }
    Direction exit_direction = MetricMap::vector2direction(exit_vector);
    Direction entrance_direction;
    switch(exit_direction) {
      case North: entrance_direction = South; break;
      case South: entrance_direction = North; break;
      case East: entrance_direction = West; break;
      case West: entrance_direction = East; break;
    }

    cout << "Follow path: " << GlobalExplorer::instance()->follow_path << " current node: " << TopoMap::instance()->current_node << endl;
    current_grid->update_gateways(); // try to re-detect gateways of previous place
    cout << "Follow path: " << GlobalExplorer::instance()->follow_path << " current node: " << TopoMap::instance()->current_node << endl;

    gsl::vector_int grid_position = MetricMap::instance()->grid_position();

    gsl::vector_int exit_position = grid_position;
    exit_position -= MetricMap::direction2vector(exit_direction);

    cout << "exit position before: " << exit_position << endl;
    for (uint i = 0; i < 2; i++) if (exit_position(i) < 0) exit_position(i) += OccupancyGrid::CELLS; else exit_position(i) %= OccupancyGrid::CELLS;
    cout << "exit position now: " << exit_position << endl;

    TopoMap::GatewayNode* exit_gateway = current_grid->find_gateway(exit_position, exit_direction);
    cout << "Current position: " << grid_position << " Entrance Direction V: " << exit_vector << endl;

    current_grid = MetricMap::instance()->current_grid;
    cout << "Follow path: " << GlobalExplorer::instance()->follow_path << " current node: " << TopoMap::instance()->current_node << endl;

    // determine gateway used for entrance, connect to current gateway
    MetricMap::instance()->current_grid->update_gateways(false);
    TopoMap::GatewayNode* current_gateway = MetricMap::instance()->current_grid->find_gateway(grid_position, entrance_direction);
    cout << "connecting current node to exit gateway" << endl;
    TopoMap::instance()->connect(TopoMap::instance()->current_node, exit_gateway);
    cout << "connecting exit gateway to entrance gateway" << endl;
    TopoMap::instance()->connect(exit_gateway, current_gateway);
    exit_gateway->set_accessible();
    current_gateway->set_accessible();
    TopoMap::instance()->current_node = current_gateway;
    cout << "Follow path: " << GlobalExplorer::instance()->follow_path << " current node: " << TopoMap::instance()->current_node << endl;
    MetricMap::instance()->current_grid->update_connectivity(); // may set current_node to a new area node
    cout << "Follow path: " << GlobalExplorer::instance()->follow_path << " current node: " << TopoMap::instance()->current_node << endl;

    cout << "Exit: " << exit_gateway << " Entrance: " << current_gateway << endl;

    // Advance/Clear global path
    if (state == ExploringGlobally) {
      bool valid = false;

      if (!GlobalExplorer::instance()->follow_path.empty()) {
        cout << "Follow path: " << GlobalExplorer::instance()->follow_path << endl;
        TopoMap::Node* next_node = GlobalExplorer::instance()->follow_path.front();
        if (next_node->is_gateway()) {
          TopoMap::GatewayNode* next_gateway = (TopoMap::GatewayNode*)next_node;

          gsl::vector_int x_range(2), y_range(2);
          next_gateway->get_ranges(x_range, y_range);
          if (exit_position(0) >= x_range(0) && exit_position(0) <= x_range(1) && exit_position(1) >= y_range(0) && exit_position(1) <= y_range(1)) {
            cout << "correct gateway, removing from global path" << endl;
            GlobalExplorer::instance()->follow_path.pop_front(); // pop first gateway
            if (!GlobalExplorer::instance()->follow_path.empty() && GlobalExplorer::instance()->follow_path.front()->is_gateway())
              GlobalExplorer::instance()->follow_path.pop_front(); // pop second gateway

            if (GlobalExplorer::instance()->follow_path.empty())
              valid = true;
            else {
              TopoMap::AreaNode* next_area = dynamic_cast<TopoMap::AreaNode*>(GlobalExplorer::instance()->follow_path.front());
              if (TopoMap::instance()->current_node == next_area) valid = true;
              else cout << "unexpected node " << TopoMap::instance()->current_node << " according to next in path " << next_area << endl;
            }
          }
          else cout << "incorrect exit: rx,ry " << x_range << " " << y_range << " | x,y " << exit_position << endl;
        }
        else cout << "next node in path is not a gateway" << endl;
      }

      if (!valid) {
        cout << "Global path is not valid anymore..." << endl;
        GlobalExplorer::instance()->clear_paths();
      }
      else
        cout << "Going where expected... advancing global path" << endl;
    }
  }

  fsm_advance();

  // Check for valid local path
  list<gsl::vector_int>& follow_path = LocalExplorer::instance()->follow_path;
  if (!follow_path.empty()) {
    // invalid path or unnecessary path
    bool valid = true;
    for (list<gsl::vector_int>::iterator it = follow_path.begin(); it != follow_path.end(); ++it) {
      if (OccupancyGrid::valid_coordinates((*it)(0),(*it)(1)) && (*current_grid)((*it)(0), (*it)(1)) >= OccupancyGrid::Locc) {
        cout << "path crosses obstacle" << endl;
        valid = false; break;
      }
    }

    // TODO: this does not work correctly. Frontiers should be re-detected periodically and this would use LocalExplorer's target_is_frontier()
    // to check validity of the current target
    /*const gsl::vector_int& last = follow_path.back();
    if (state == ExploringLocally && fabs((*current_grid)(last(0), last(1))) > 2 * MetricMap::frontier_cell_threshold) {
      cout << "target is no longer frontier cell" << endl;
      valid = false;
    }

    if (!valid) {
      LocalExplorer::instance()->clear_paths();
      cout << "path is invalid" << endl;
    }*/
  }
}

void Explorer::while_exploring_locally(void) {
  if (LocalExplorer::instance()->follow_path.empty()) {
    ExaBot::instance()->stop(); // stop robot motion during this iteration
    cout << "Looking for new frontiers..." << endl;
    LocalExplorer::instance()->compute_frontier_paths();
    if (!LocalExplorer::instance()->found_path()) {
      cout << "No more frontiers... marking area node as explored" << endl;
      dynamic_cast<TopoMap::AreaNode*>(TopoMap::instance()->current_node)->completely_explored = true;
      start(ExploringGlobally);
    }
    else {
      cout << "Found frontier path: " << LocalExplorer::instance()->follow_path << endl;
    }
  }
}

void Explorer::start_exploring_globally(void) {
  GlobalExplorer::instance()->clear_paths();
  LocalExplorer::instance()->clear_paths();
}

void Explorer::while_exploring_globally(void) {
  if (GlobalExplorer::instance()->follow_path.empty()) {
    ExaBot::instance()->stop(); // stop robot motion during this iteration
    if (dynamic_cast<TopoMap::AreaNode*>(TopoMap::instance()->current_node)->completely_explored) {
      cout << "Computing global path..." << endl;
      recompute_path();
    }
    else {
      cout << "global path points to current place, and this is unexplored... exploring locally" << endl;
      start(ExploringLocally);
      return;
    }
  }

  cout << "Following global path..." << endl;
  TopoMap::Node* target_node = GlobalExplorer::instance()->follow_path.front();
  cout << "Global path: " << GlobalExplorer::instance()->follow_path << " Current metric node: " << current_grid << " Current topo node: " << TopoMap::instance()->current_node << endl;
  if (TopoMap::instance()->current_node == target_node) {
    GlobalExplorer::instance()->follow_path.pop_front();
    cout << "Reached node " << target_node << endl;
    if (GlobalExplorer::instance()->follow_path.empty()) {
      cout << "Reached destination place..." << endl;
      start(ExploringLocally);
    }
    else {
      cout << "Computing local path to next place..." << endl;
      recompute_local_path();
    }
  }
  else {
    if (LocalExplorer::instance()->follow_path.empty()) {
      cout << "Cant find local path to place..." << endl;
      recompute_path();
    }
    else cout << "Trying to reach place " << target_node << endl;
  }
}


bool Explorer::recompute_local_path(void) {
  if (state == ExploringLocally) {
    cout << "Recomputing local path to frontiers" << endl;
    LocalExplorer::instance()->compute_frontier_paths();
  }
  else {
    cout << "Recomputing local path to global node" << endl;
    TopoMap::Node* target_node = GlobalExplorer::instance()->follow_path.front();
    if (target_node->is_gateway()) {
      TopoMap::GatewayNode* target_gateway = (TopoMap::GatewayNode*)target_node;
      if (target_gateway->grid == MetricMap::instance()->current_grid) {
        cout << "computing gateway path" << endl;
        LocalExplorer::instance()->compute_gateway_path(target_gateway);
      }
      else cout << "gateway is in different place" << endl;
    }
    else cout << "target is not a gateway" << endl;
  }

  cout << "Recomputed paths..." << endl;
  if (!LocalExplorer::instance()->found_path()) { cout << "No path found" << endl; return false; }
  else { cout << "Computed Local path: " << LocalExplorer::instance()->follow_path << endl; return true; }
}

void Explorer::recompute_path(void) {
  if (state == ExploringLocally) {
    bool paths_found = recompute_local_path();
    if (!paths_found) {
      start(ExploringGlobally);
      dynamic_cast<TopoMap::AreaNode*>(TopoMap::instance()->current_node)->completely_explored = true;
      cout << "No more frontier paths, so marking as explored" << endl;
    }
    else {
      cout << "Local paths available" << endl;
      return;
    }
  }
  else {
    if (!(dynamic_cast<TopoMap::AreaNode*>(TopoMap::instance()->current_node)->completely_explored)) { start(ExploringLocally); return; }
  }

  // At this point: only exploring globally
  cout << "Recomputing Global paths..." << endl;
  GlobalExplorer::instance()->recompute_paths();
  if (!GlobalExplorer::instance()->found_path()) { cout << "EXPLORATION IS OVER!" << endl; throw false; }
  cout << "done: " << GlobalExplorer::instance()->follow_path << endl;

  while(true) {
    bool paths_found = recompute_local_path();
    if (paths_found) {
      //cout << "Found local path: " << LocalExplorer::instance()->follow_path << endl;
      dynamic_cast<TopoMap::GatewayNode*>(GlobalExplorer::instance()->follow_path.front())->add_reach_attempt();
      break;
    }
    else {
      cout << "Couldn't find path to edge, so recomputing global path" << endl;
      GlobalExplorer::instance()->recompute_paths();
      if (!GlobalExplorer::instance()->found_path()) { cout <<  "EXPLORATION IS OVER!" << endl; throw false; }
      else { /*cout << "Proposing global path: " << GlobalExplorer::instance()->follow_path << endl*/; }
    }
  }
}

MotionPlanner::Motion Explorer::compute_motion(Position2dProxy& position_proxy) {
  // if the robot is trying to reach a target or not
  list<gsl::vector_int>& follow_path = LocalExplorer::instance()->follow_path;
  const gsl::vector& own_position = MetricMap::instance()->position;

  // remove all path nodes already considered as "reached"
  double reached_distance_threshold = 0.4;
  double far_distance_threshold = 0.9;
  for (list<gsl::vector_int>::iterator it = follow_path.begin(); it != follow_path.end();) {
    gsl::vector target_distance = (gsl::vector)(*it) * OccupancyGrid::CELL_SIZE - own_position;
    cout << "this target: " << *it << " own position: " << own_position << " in cell coords: " << (*it) * OccupancyGrid::CELL_SIZE << endl;
    cout << "target distance: " << target_distance.norm2() << endl;

    list<gsl::vector_int>::iterator it2(it); ++it2;
    if (it2 == follow_path.end()) break; // leave at least one target
    if (target_distance.norm2() < reached_distance_threshold) { cout << "removing " << *it << " from follow path" << endl; it = follow_path.erase(it); }
    else break;
  }

  if (!follow_path.empty()) {
    gsl::vector_int& target = follow_path.front();
    gsl::vector target_distance = (gsl::vector)target * OccupancyGrid::CELL_SIZE - own_position;
    double target_distance_norm = target_distance.norm2();
    cout << "target: " << target << " distance: " << target_distance_norm << endl;

    double target_angle = (target_distance_norm == 0 ? 0 : atan2(target_distance(1), target_distance(0))); // cartesian to polar
    target_angle = gsl_sf_angle_restrict_pos(target_angle);

    // follow waypoint
    double safe_angle = MotionPlanner::instance()->compute_motion_direction(target_angle);

    // threshold given by robot size
    //double target_distance_threshold = (state == ExploringLocally && follow_path.size() == 1 ? 0.4 : 0.2);
    
    if (target_distance_norm < reached_distance_threshold) {
      cout << "reached point " << target(0) << " " << target(1) << " in path (distance " << target_distance_norm << ")" << endl;
      follow_path.pop_front();
    }
    else {
      if (!OccupancyGrid::valid_coordinates(target(0),target(1))) target_distance_norm = remainder(target_distance_norm, OccupancyGrid::SIZE);
      if (abs(gsl_sf_angle_restrict_symm(safe_angle - target_angle)) > (135.0 * M_PI / 180.0) ||
        target_distance_norm > far_distance_threshold)
      {
        cout << "Can't go where expected..." << "delta angle: " << abs(gsl_sf_angle_restrict_symm(safe_angle - target_angle)) << " d: " << target_distance << " target: "
          << target << endl;
        recompute_path();
      }
    }

    return MotionPlanner::instance()->compute_motion_from_target(safe_angle);
  }
  else {
    return MotionPlanner::instance()->compute_motion(MotionPlanner::instance()->winner_direction_angle());
  }
}