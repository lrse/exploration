#include <gsl/gsl_sf_trig.h>
#include <algorithm>
#include "motion_planner.h"
#include "metric_map.h"
#include "exabot.h"
using namespace HybNav;
using namespace std;


/**************************
 * Constructor/Destructor *
 **************************/

MotionPlanner::MotionPlanner(PlayerCc::PlayerClient* client_proxy) : Singleton<MotionPlanner>(this),
  position_proxy(client_proxy, 1)
{
  REACHED_POS_EPSILON = ExaBot::ROBOT_RADIUS;
  REACHED_ANGLE_EPSILON = 2 * M_PI / 180.0;
  
  goal_set = false;
  goal_x = goal_y = goal_theta = 0;
}

/**************************
 *     Public Methods     *
 **************************/
 
void MotionPlanner::set_goal(double x, double y, double theta)
{
  cout << "setting pose to: " << x << " " << y << " " << theta << endl;
  position_proxy.GoTo(x, y, theta);
  position_proxy.SetMotorEnable(true);
  goal_x = x;
  goal_y = y;
  goal_theta = theta;
  goal_set = true;
}

void MotionPlanner::update(void) {
  if (reached_goal()) goal_set = false;
}

void MotionPlanner::stop(void) {
  position_proxy.GoTo(position_proxy.GetXPos(), position_proxy.GetYPos(), position_proxy.GetYaw());
  position_proxy.SetMotorEnable(false);
}

bool MotionPlanner::reached_goal(void) {
  if (!goal_set) return true;
  
  return reached(goal_x, goal_y, goal_theta);
}

bool MotionPlanner::reached(double other_x, double other_y, double other_theta) {
  double x = ExaBot::instance()->position_proxy.GetXPos();
  double y = ExaBot::instance()->position_proxy.GetYPos();
  double theta = ExaBot::instance()->position_proxy.GetYaw();
  double delta_norm = hypot(other_x - x, other_y - y);
  double delta_angle = fabsf(gsl_sf_angle_restrict_pos(other_theta - theta));
  //bool was_reached = (delta_norm < REACHED_POS_EPSILON && delta_angle < REACHED_ANGLE_EPSILON);
  bool was_reached = delta_norm < REACHED_POS_EPSILON;
  cout << "reached? " << delta_norm << " < " << REACHED_POS_EPSILON << " AND " << delta_angle << " < " <<  REACHED_ANGLE_EPSILON << " : " << was_reached << endl;
  return was_reached;
}

bool MotionPlanner::valid_path(void) {
  bool valid = (position_proxy.GetXSpeed() != 0 || position_proxy.GetYawSpeed() != 0);
  cout << "speeds: " << position_proxy.GetXSpeed() << " "  << position_proxy.GetYawSpeed() << " valid?: " << valid << endl;
  cout << "real speeds: " << ExaBot::instance()->position_proxy.GetXSpeed() << " " << ExaBot::instance()->position_proxy.GetYawSpeed() << endl;
  cout << "enabled? " << position_proxy.GetStall() << " " << ExaBot::instance()->position_proxy.GetStall() << endl;
  
  return true;
}
