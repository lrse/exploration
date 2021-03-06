#include <set>
#include <map>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include "metric_map.h"
#include "exabot.h"
using namespace HybNav;
using namespace std;

double MetricMap::frontier_cell_threshold = 0.25;


struct Position {
  size_t x, y;
  bool operator<(const Position& other) const {
    return (x < other.x && y == other.y) || (y < other.y);
  }
};

struct centroid { size_t x, y; size_t n; };

void OccupancyGrid::update_frontiers(void)
{
  debug_graph = cv::Scalar(0,0,0);
  
  const uint minimum_frontier_length = (uint)round(ExaBot::ROBOT_RADIUS / OccupancyGrid::CELL_SIZE);
  cout << "minimum_frontier_length: " << minimum_frontier_length << endl;
  
  // copy to OpenCV mat
  cv::Mat_<double> mat(OccupancyGrid::CELLS, OccupancyGrid::CELLS);
  for (uint i = 0; i < OccupancyGrid::CELLS; i++) {
    for (uint j = 0; j < OccupancyGrid::CELLS; j++) {
      mat.at<double>(i,j) = m(i, j);
    }
  }
  
  // 
  
  /* compute frontier cell positions and add to list */
  std::map<Position, uint> frontier_cells_set;

  for (size_t i = 0; i < m.size1(); i++) {
    for (size_t j = 0; j < m.size2(); j++) {
      double v = m(i, j);
      if (fabs(v) > MetricMap::frontier_cell_threshold) continue;

      uint free_count = 0, unknown_count = 0;
      for (int ii = -1; ii <= 1; ii++) {
        for (int jj = -1; jj <= 1; jj++) {
          if (ii == 0 && jj == 0) continue;
          if ((ssize_t)i + ii < 0 || i + ii >= m.size1() || (ssize_t)j + jj < 0 || j + jj >= m.size2()) continue;

          double v2 = m(i + ii, j + jj);
          /*if (v2 < Lfree * 0.2) free_count++;
          else if (v2 < Locc * 0.2) unknown_count++;*/
          if (v2 < 0) free_count++;
          else if (v2 == 0) unknown_count++;
        }
      }

      // condition for "frontier cell"
      //if (free_count >= 3 && unknown_count >= 3) {
      if (v == 0 && free_count >= 1) {
        Position p = { i, j };
        frontier_cells_set[p] = numeric_limits<uint>::max();
        debug_graph.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
        //cout << "cell pos: " << i << " " << j << endl;
      }
    }
  }

  /* determine a set of frontiers by grouping nearby frontier cells */
  map< uint, set<uint> > set_equivalences;
  uint next_set_id = 1;
  for (map<Position, uint>::iterator it = frontier_cells_set.begin(); it != frontier_cells_set.end(); ++it) {
    size_t i = it->first.x;
    size_t j = it->first.y;

    // set of neighbor cells that are also frontiers
    set<uint> neighbour_ids;
    for (int ii = -1; ii <= 1; ii++) {
      for (int jj = -1; jj <= 1; jj++) {
        if (ii == 0 && jj == 0) continue;
        if ((ssize_t)i + ii < 0 || i + ii >= m.size1() || (ssize_t)j + jj < 0 || j + jj >= m.size2()) continue;

        Position p2 = { i + ii, j + jj };
        map<Position, uint>::const_iterator it2 = frontier_cells_set.find(p2);
        if (it2 != frontier_cells_set.end()) { neighbour_ids.insert(it2->second); }
      }
    }

    if (neighbour_ids.size() == 0) {
      it->second = next_set_id;
      set<uint> singleton_set = set<uint>(); singleton_set.insert(next_set_id);
      set_equivalences[next_set_id] = singleton_set;
      next_set_id++;
    }
    else {
      set<uint>::const_iterator min_it = std::min_element(neighbour_ids.begin(), neighbour_ids.end());
      if (*min_it == numeric_limits<uint>::max()) {
        it->second = next_set_id;
        set<uint> singleton_set = set<uint>(); singleton_set.insert(next_set_id);
        set_equivalences[next_set_id] = singleton_set;
        next_set_id++;
      }
      else {
        it->second = *min_it;

        for (set<uint>::const_iterator it2 = neighbour_ids.begin(); it2 != neighbour_ids.end(); ++it2) {
          if (set_equivalences.find(*it2) == set_equivalences.end()) {
            set_equivalences[*it2] = neighbour_ids;
          }
          else {
            set<uint>& s = set_equivalences[*it2];
            set<uint> out;
            set_union(s.begin(), s.end(), neighbour_ids.begin(), neighbour_ids.end(), inserter(out, out.begin()));
            set_equivalences[*it2] = out;
          }
        }
      }
    }
  }

  /* rearrange set ids to account for equivalence and construct the frontier list */
  map< uint, set<Position> > frontiers_map;
  for (map<Position, uint>::iterator it = frontier_cells_set.begin(); it != frontier_cells_set.end(); ++it) {
    set<uint>& equivalences = set_equivalences[it->second];
    uint s = *min_element(equivalences.begin(), equivalences.end());
    it->second = s;
    if (frontiers_map.find(s) == frontiers_map.end()) { set<Position> new_set; new_set.insert(it->first); frontiers_map[s] = new_set; }
    else { frontiers_map[s].insert(it->first); }
  }

  /* compute k-means for each frontier */
  set<Position> frontier_centers;
  for (map< uint, set<Position> >::const_iterator it = frontiers_map.begin(); it != frontiers_map.end(); ++it) {
    const set<Position>& s = it->second;
    if (s.size() == 1 || s.size() < minimum_frontier_length) {
      cout << "frontier of size " << s.size() << " discarded" << endl;
      continue; // k-means doesn't seem to work right with one frontier cell only
    }
      
    //uint clusters = (uint)ceil(s.size() / (float)20);
    uint clusters = 1;
    cv::Mat_<float> samples(s.size(), 2);
    cv::Mat_<float> centers(clusters, 2);
    uint i = 0;
    for (set<Position>::const_iterator it2 = s.begin(); it2 != s.end(); ++it2, ++i) {
      samples(i, 0) = it2->x;
      samples(i, 1) = it2->y;
    } 
    cout << s.size() << " " << clusters << endl;
    cv::Mat labels;
    cv::kmeans(samples, clusters, labels, cv::TermCriteria(cv::TermCriteria::MAX_ITER, 20, 0), 20, cv::KMEANS_RANDOM_CENTERS, centers);
    for (i = 0; i < clusters; i++) {
      Position p = { centers(i, 0), centers(i, 1) };
      frontier_centers.insert(p);
    }
  }

  /* compute k-means positions */
  cout << "detected frontiers:";
  frontiers.clear();
  uint i = 0;
  for (set<Position>::const_iterator it = frontier_centers.begin(); it != frontier_centers.end(); ++it, ++i) {
    gsl::vector_int pos(2);
    debug_graph.at<cv::Vec3b>(it->x, it->y) = cv::Vec3b(0, 0, 255);
    pos(0) = it->y;
    pos(1) = CELLS - it->x - 1;
    cout << " [" << pos(0) << "," << pos(1) << "]";
    frontiers.push_back(pos);
  }
  cout << endl;
}

