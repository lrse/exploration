#include <gsl/gsl_sf_trig.h>
#include <algorithm>
#include "motion_planner.h"
#include "metric_map.h"
using namespace HybNav;
using namespace std;


/**************************
 * Constructor/Destructor *
 **************************/

MotionPlanner::MotionPlanner(PlayerCc::Position2dProxy& _position_proxy) : Singleton<MotionPlanner>(this),
  position_proxy(_position_proxy), last_position(2), delta_position(2, true)
{
  position_proxy.GetPlayerClient()->Read();
  last_position(0) = position_proxy.GetXPos();
  last_position(1) = position_proxy.GetYPos();

  WINDOW_CELLS = 51; // Must be odd!
  WINDOW_HALF_CELLS = (WINDOW_CELLS - 1) / 2;
  WINDOW_SIZE = WINDOW_CELLS * OccupancyGrid::CELL_SIZE;
  CENTER_POS = (WINDOW_SIZE * 0.5);
  POLAR_SAMPLES = 72; // Must be even!
  POLAR_SAMPLE_ANGLE = ((2 * M_PI) / POLAR_SAMPLES);
  MU1 = 6; // MU1 > MU2 + MU3
  MU2 = 2;
  MU3 = 3;
  MINIMUM_DISTANCE = 0.13;
  ENLARGEMENT_RADIUS = MINIMUM_DISTANCE + MetricMap::ROBOT_RADIUS;
  TAU_HIGH = 0.15; // [0,1]. 1 = al lado del robot, 0 = a WINDOW_CELLS/2 de distancia. Valores mayores a TAU_HIGH se interpretan como obstaculo
  //TAU_LOW = 0.5;
  NARROW_LIMIT = 16;

  polar_histogram.resize(POLAR_SAMPLES);
  polar_histogram.set_zero();
  binary_histogram.resize(POLAR_SAMPLES);
  binary_histogram.set_zero();
  
  window.set_dimensions(WINDOW_CELLS, WINDOW_CELLS);
  window.set_zero();
}

/**************************
 *     Public Methods     *
 **************************/

double MotionPlanner::winner_direction_angle(void) {
  return winner_direction * POLAR_SAMPLE_ANGLE;
}

double MotionPlanner::cost_function(uint c, uint target) {
  return
    MU1 * delta_function(c, target) +
    MU2 * delta_function(c, (uint)(gsl_sf_angle_restrict_pos(position_proxy.GetYaw()) / POLAR_SAMPLE_ANGLE)) +
    MU3 * delta_function(c, winner_direction);
}

uint MotionPlanner::delta_function(int c1, int c2) {
  int a = min<int>(abs(c1 - c2), abs(c1 - c2 - POLAR_SAMPLES));
  int b = min<int>(a, abs(c1 - c2 + POLAR_SAMPLES));
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
        unsigned int l = (unsigned int)floor(gsl_sf_angle_restrict_pos(beta) / POLAR_SAMPLE_ANGLE);

        polar_histogram(l) += e;
        if (polar_histogram(l) > TAU_HIGH) binary_histogram(l) = 1;

        for (double a = -gamma; a <= gamma; a += POLAR_SAMPLE_ANGLE) {
          unsigned int m = (unsigned int)((int)floor(gsl_sf_angle_restrict_pos(beta + a) / POLAR_SAMPLE_ANGLE) % POLAR_SAMPLES);
          if (m != l) {
            polar_histogram(m) += e;
            if (polar_histogram(m) > TAU_HIGH) binary_histogram(m) = 1;
          }
        }
      }
    }
  }
}

double MotionPlanner::compute_motion_direction(double target_angle) {
  uint target_slot = (uint)(target_angle / POLAR_SAMPLE_ANGLE);

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

  // print results
  for (list< pair<uint,uint> >::iterator it = valleys.begin(); it != valleys.end(); ++it)
    cout << "[" << it->first << "," << it->second << "] ";
  cout << endl;

  // determine candidates
  candidates.clear();
  for (list< pair<uint,uint> >::iterator it = valleys.begin(); it != valleys.end(); ++it) {
    uint l = it->first; uint r = it->second;

    if (l == 0 && r == POLAR_SAMPLES - 1) {
      candidates.push_back(target_slot); // If no obstacle => follow target
      winner_direction = target_slot;
    }
    else {
      uint d;
      if (l <= r) d = r - l;
      else d = (POLAR_SAMPLES - (r - l));

      if (d <= NARROW_LIMIT) {
        candidates.push_back((l + (d / 2)) % POLAR_SAMPLES);
      }
      else {
        uint cl = (l + NARROW_LIMIT / 2) % POLAR_SAMPLES;
        uint cr = (r - NARROW_LIMIT / 2) % POLAR_SAMPLES;
        candidates.push_back(cl);
        candidates.push_back(cr);
        if ((cl <= cr && (target_slot >= cl && target_slot <= cr)) ||
            (cl > cr && (target_slot >= cl || target_slot <= cr))) { candidates.push_back(target_slot); }
      }
    }
  }

  cout << "candidates: ";
  uint minimum_cost = numeric_limits<uint>::max();
  for (list<uint>::iterator it = candidates.begin(); it != candidates.end(); ++it) {
    cout << *it * POLAR_SAMPLE_ANGLE * (180.0 / M_PI) << " ";
    winner_direction = *it;
    uint this_cost = cost_function(*it, target_slot);
    if (this_cost < minimum_cost) { minimum_cost = this_cost; winner_direction = *it; }
  }
  
  double winner_direction_angle = winner_direction * POLAR_SAMPLE_ANGLE;
  cout << " winner: " << winner_direction_angle << endl;

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

void MotionPlanner::compute_motion(double target_angle) {
  double selected_direction = compute_motion_direction(target_angle);
  compute_motion_from_target(selected_direction);
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