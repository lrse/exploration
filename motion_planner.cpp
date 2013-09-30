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
  REACHED_ANGLE_EPSILON = 5 * M_PI / 180.0;
  
  goal_set = false;
  goal_x = goal_y = goal_theta = 0;
  seconds_elapsed = 0;
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
  last_time = boost::posix_time::microsec_clock::local_time();
}

void MotionPlanner::update(void) {
  if (reached_goal()) goal_set = false;
}

void MotionPlanner::stop(void) {
  position_proxy.GoTo(position_proxy.GetXPos(), position_proxy.GetYPos(), position_proxy.GetYaw());
  position_proxy.SetMotorEnable(false);
  seconds_elapsed += (boost::posix_time::microsec_clock::local_time() - last_time).total_milliseconds() / 1000.0;
}

bool MotionPlanner::reached_goal(void) {
  if (!goal_set) return true;

  gsl::vector goal_vec(2);
  goal_vec(0) = goal_x;
  goal_vec(1) = goal_y;
  return reached(goal_vec, goal_theta);
}

bool MotionPlanner::reached(const gsl::vector& other_pos, double other_theta) {
  double x = ExaBot::instance()->last_position(0);
  double y = ExaBot::instance()->last_position(1);
  double theta = ExaBot::instance()->last_rotation;
  double delta_norm = hypot(other_pos(0) - x, other_pos(1) - y);
  double delta_angle = fabsf(gsl_sf_angle_restrict_pos(other_theta - theta));
  //bool was_reached = (delta_norm < REACHED_POS_EPSILON && delta_angle < REACHED_ANGLE_EPSILON);

  double reached_threshold = REACHED_POS_EPSILON;
  bool was_reached = delta_norm < reached_threshold;
  cout << "reached? " << delta_norm << " < " << reached_threshold << /*" AND " << delta_angle << " < " <<  REACHED_ANGLE_EPSILON <<*/ " : " << was_reached << endl;
  return was_reached;
}

bool MotionPlanner::valid_path(void) {
  bool valid = (position_proxy.GetXSpeed() != 0 || position_proxy.GetYawSpeed() != 0);
  cout << "speeds: " << position_proxy.GetXSpeed() << " "  << position_proxy.GetYawSpeed() << " valid?: " << valid << endl;
  cout << "real speeds: " << ExaBot::instance()->position_proxy.GetXSpeed() << " " << ExaBot::instance()->position_proxy.GetYawSpeed() << endl;
  cout << "enabled? " << position_proxy.GetStall() << " " << ExaBot::instance()->position_proxy.GetStall() << endl;
  
  return true;
}
