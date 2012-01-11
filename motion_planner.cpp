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
  /*WINDOW_CELLS = 81; // Must be odd!
  WINDOW_HALF_CELLS = (WINDOW_CELLS - 1) / 2;
  WINDOW_SIZE = WINDOW_CELLS * OccupancyGrid::CELL_SIZE;
  CENTER_POS = (WINDOW_SIZE * 0.5);
  POLAR_SAMPLES = 72; // Must be even!
  POLAR_SAMPLE_ANGLE = ((2 * M_PI) / POLAR_SAMPLES);
  MU1 = 5; // MU1 > MU2 + MU3
  MU2 = 2;
  MU3 = 2;*/
  /*MINIMUM_DISTANCE = 0.01;
  ENLARGEMENT_RADIUS = MINIMUM_DISTANCE + ExaBot::ROBOT_RADIUS;*/
/*#if SYROTEK
  TAU_HIGH = 0.55; // [0,1]. 1 = al lado del robot, 0 = a WINDOW_CELLS/2 de distancia. Valores mayores a TAU_HIGH se interpretan como obstaculo
#else
  TAU_HIGH = 0.55; // [0,1]. 1 = al lado del robot, 0 = a WINDOW_CELLS/2 de distancia. Valores mayores a TAU_HIGH se interpretan como obstaculo
#endif
  //TAU_LOW = 0.5;
  NARROW_LIMIT = 16;*/
}

/**************************
 *     Public Methods     *
 **************************/
 
void MotionPlanner::set_goal(double x, double y, double theta)
{
  cout << "setting pose to: " << x << " " << y << " " << theta << endl;
  position_proxy.GoTo(x, y, theta);
  position_proxy.SetMotorEnable(true);  
}

void MotionPlanner::stop(void) {
  position_proxy.GoTo(position_proxy.GetXPos(), position_proxy.GetYPos(), position_proxy.GetYaw());
  position_proxy.SetMotorEnable(false);
}

bool MotionPlanner::valid_path(void) {
  bool valid = (position_proxy.GetXSpeed() != 0 || position_proxy.GetYawSpeed() != 0);
  cout << "speeds: " << position_proxy.GetXSpeed() << " "  << position_proxy.GetYawSpeed() << " valid?: " << valid << endl;
  cout << "real speeds: " << ExaBot::instance()->position_proxy.GetXSpeed() << " " << ExaBot::instance()->position_proxy.GetYawSpeed() << endl;
  cout << "enabled? " << position_proxy.GetStall() << " " << ExaBot::instance()->position_proxy.GetStall() << endl;
  
  return true;
}

/*double MotionPlanner::winner_direction_angle(void) {
  return winner_direction * POLAR_SAMPLE_ANGLE;
}

int MotionPlanner::cost_function(uint c, uint target) {
  int c1 = delta_function(c, target);
  int c2 = delta_function(c, (int)(gsl_sf_angle_restrict_pos(position_proxy.GetYaw()) / POLAR_SAMPLE_ANGLE));
  int c3 = delta_function(c, winner_direction);
  int ret = MU1 * c1 + MU2 * c2 + MU3 * c3;
  //cout << "target slot: " << target << " (" << target * POLAR_SAMPLE_ANGLE << ")" << endl;
  //cout << "cost for " << c * POLAR_SAMPLE_ANGLE * (180.0 / M_PI) << " " << c1 << " " << c2 << " " << c3 << " | " << MU1 * c1 << " " << MU2 * c2 << " " << MU3 * c3 << " " << ret << endl;
  return ret;
}

int MotionPlanner::delta_function(int c1, int c2) {
  int a = min<int>(abs<int>(c1 - c2), abs<int>(c1 - c2 - POLAR_SAMPLES));
  int b = min<int>(a, abs<int>(c1 - c2 + POLAR_SAMPLES));
  return b;
}

void MotionPlanner::update_position(void) {
  gsl::vector current_position(2);
  current_position(0) = position_proxy.GetXPos();
  current_position(1) = position_proxy.GetYPos();

  gsl::vector_int cell_delta(2);
  delta_position += current_position - last_position;
  last_position = current_position;
  for (uint i = 0; i < 2; i++) {
    if (abs(delta_position(i)) >= OccupancyGrid::CELL_SIZE) {
      cell_delta(i) = (int)(delta_position(i) / OccupancyGrid::CELL_SIZE);
      delta_position(i) = (delta_position(i) < 0 ? -fmod(-delta_position(i), OccupancyGrid::CELL_SIZE) : fmod(delta_position(i), OccupancyGrid::CELL_SIZE));
    }
  }
  slide_window(cell_delta);
}

void MotionPlanner::process_distances(PlayerCc::LaserProxy& laser_proxy)
{
  double max_range = 4;

  for (size_t i = 0; i < laser_proxy.GetCount(); i++) {
    double angle = laser_proxy.GetBearing(i) + position_proxy.GetYaw();
    double dist = laser_proxy.GetRange(i);
    if (dist < max_range) {
      gsl::vector coord(2);
      coord(0) = cos(angle);
      coord(1) = sin(angle);
      coord *= dist;

      gsl::vector_int point(2);
      for (uint i = 0; i < 2; i++)
        point(i) = (int)floor(coord(i) / OccupancyGrid::CELL_SIZE) + WINDOW_HALF_CELLS;

      if (point(0) >= 0 && point(1) >= 0 && (uint)point(0) < WINDOW_CELLS && (uint)point(1) < WINDOW_CELLS)
        window(WINDOW_CELLS - point(1) - 1, point(0)) = (WINDOW_CELLS * 0.5 - (point - WINDOW_HALF_CELLS).norm2()) / (WINDOW_CELLS * 0.5);
    }
  }

  polar_histogram.set_zero();
  binary_histogram.set_zero();

  for (size_t k = 0; k < window.size1(); k++) {
    for (size_t i = 0; i < window.size2(); i++) {
      size_t j = WINDOW_CELLS - k - 1;
      double e = window(k, i);

      double position = (floor(CENTER_POS / OccupancyGrid::CELL_SIZE) + 0.5) * OccupancyGrid::CELL_SIZE;
      double coord[] = { i, j };
      for (size_t l = 0; l < 2; l++)
        coord[l] = (coord[l] + 0.5) * OccupancyGrid::CELL_SIZE - position; // coordinate now relative to robot position

      // convert cartesian to polar
      double d = hypot(coord[0], coord[1]);
      double beta = (d == 0 ? 0 : atan2(coord[1], coord[0]));
      if (d != 0) {
        double gamma = abs(asin(min(ENLARGEMENT_RADIUS / d, 1.0)));
        unsigned int l = (unsigned int)floor(gsl_sf_angle_restrict_pos(beta) / POLAR_SAMPLE_ANGLE) % POLAR_SAMPLES;

        polar_histogram(l) = e;
        if (polar_histogram(l) > TAU_HIGH) binary_histogram(l) = 1;

        for (double a = -gamma; a <= gamma; a += POLAR_SAMPLE_ANGLE) {
          unsigned int m = (unsigned int)((int)floor(gsl_sf_angle_restrict_pos(beta + a) / POLAR_SAMPLE_ANGLE) % POLAR_SAMPLES);
          if (m != l) {
            polar_histogram(m) = max(polar_histogram(m), e);
            if (polar_histogram(m) > TAU_HIGH) binary_histogram(m) = 1;
          }
        }
      }
    }
  }
}

double MotionPlanner::compute_motion_direction(double target_angle) {
  uint target_slot = (uint)(gsl_sf_angle_restrict_pos(target_angle) / POLAR_SAMPLE_ANGLE);

  // determine valleys in binary histogram
  list< std::pair<uint,uint> > valleys;
  uint first_peak = binary_histogram.size();
  for (uint i = 0; i < binary_histogram.size(); i++) {
    if (binary_histogram(i) == 1) { first_peak = i; break;}
  }

  // if no peaks, only one valley covering entire 360 degrees
  if (first_peak == binary_histogram.size()) {
    valleys.push_back(make_pair(0, POLAR_SAMPLES - 1));
  }
  else {
    uint i = first_peak;
    uint valley_start = binary_histogram.size();
    do {
      i = (i + 1) % POLAR_SAMPLES;
      if (valley_start == binary_histogram.size()) {
        if (binary_histogram(i) == 0) valley_start = i;
      }
      else {
        if (binary_histogram(i) == 1) {
          valleys.push_back(make_pair(valley_start, (i - 1) % POLAR_SAMPLES));
          valley_start = binary_histogram.size();
        }
      }
    } while (i != first_peak);
  }

  // determine candidates
  candidates.clear();
  cout << "valleys: ";
  for (list< pair<uint,uint> >::iterator it = valleys.begin(); it != valleys.end(); ++it) {
    int l = (int)it->first; int r = (int)it->second;
    cout << "[" << l << "," << r << "] ";

    if (l == 0 && r == POLAR_SAMPLES - 1) {
      candidates.push_back(target_slot); // If no obstacle => follow target
      winner_direction = target_slot;
      cout << "*";
    }
    else {
      uint d;
      if (l <= r) d = r - l;
      else d = (POLAR_SAMPLES - (l - r));

      if (d <= NARROW_LIMIT) {
        candidates.push_back((l + (d / 2)) % POLAR_SAMPLES);
      }
      else {
        int cl, cr;
        cl = (l + NARROW_LIMIT / 2);
        cr = (r - NARROW_LIMIT / 2);
        if (cr < 0) cr += POLAR_SAMPLES;
        cl %= POLAR_SAMPLES;
        cr %= POLAR_SAMPLES;

        cout << "(cl/cr: " << cl << " " << cr << ")";
        candidates.push_back(cl);
        candidates.push_back(cr);
        if ((cl <= cr && (target_slot >= cl && target_slot <= cr)) ||
            (cl > cr && (target_slot >= cl || target_slot <= cr))) { candidates.push_back(target_slot); }
      }
    }
  }
  cout << endl;
  
  if (candidates.empty()) throw std::runtime_error("no safe angle!");

  // should this go outside the loop?
  int minimum_cost = numeric_limits<int>::max();
  uint potential_winner_direction = winner_direction;
  for (list<uint>::iterator it = candidates.begin(); it != candidates.end(); ++it) {
    int this_cost = cost_function(*it, target_slot);
    cout << "this cost: " << this_cost << " minimum cost: " << minimum_cost << endl;
    if (this_cost < minimum_cost) { minimum_cost = this_cost; potential_winner_direction = *it; }
    cout << "minimum is now: " << minimum_cost << endl;
  }

  cout << "target: " << target_angle * (180.0 / M_PI) << " ";
  cout << "candidates: ";
  for (list<uint>::iterator it = candidates.begin(); it != candidates.end(); ++it) {
    cout << *it * POLAR_SAMPLE_ANGLE * (180.0 / M_PI) << " (" << cost_function(*it, target_slot) << ") ";
  }

  winner_direction = potential_winner_direction;
  
  double winner_direction_angle = winner_direction * POLAR_SAMPLE_ANGLE;
  cout << " winner: " << winner_direction_angle * (180.0 / M_PI) << endl;

  return winner_direction_angle;
}

MotionPlanner::Motion MotionPlanner::compute_motion_from_target(double target_angle) {
  double own_rotation = gsl_sf_angle_restrict_pos(position_proxy.GetYaw());
  double delta_rot = gsl_sf_angle_restrict_symm(target_angle - own_rotation);
  double rot_threshold = (6 * (M_PI / 180.0));

  Motion motion;
  if (abs(delta_rot) < rot_threshold) {
    cout << "Going straigth..." << endl;
    motion = ForwardMotion;
  }
  else {
    if (delta_rot > 0) motion = LeftTurn;
    else motion = RightTurn;
  }

  return motion;
}

MotionPlanner::Motion MotionPlanner::compute_motion(double target_angle) {
  double selected_direction = compute_motion_direction(target_angle);
  return compute_motion_from_target(selected_direction);
}

void MotionPlanner::slide_window(const gsl::vector_int& delta) {
  gsl::matrix m2(WINDOW_CELLS, WINDOW_CELLS, true);
  cout << "sliding " << delta << endl;

  ssize_t slide_i = delta(1);
  ssize_t slide_j = -delta(0);

  for (ssize_t i = 0; (size_t)i < window.size1(); i++) {
    for (ssize_t j = 0; (size_t)j < window.size2(); j++) {
      if (i - slide_i >= 0 && (size_t)(i - slide_i) < window.size1() && j - slide_j >= 0 && (size_t)(j - slide_j) < window.size2()) {
        double v = window((size_t)(i - slide_i), (size_t)(j - slide_j));
        m2((size_t)i, (size_t)j) = v;
      }
    }
  }

  window.copy(m2);
}
*/
