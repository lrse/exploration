#ifndef __METRIC_MAP_H__
#define __METRIC_MAP_H__

#include "graph.h"
#include "singleton.h"
#include "super_matrix.h"
#include <gslwrap/vector_double.h>
#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_int.h>
#include <libplayerc++/playerc++.h>

namespace HybNav {
  class OccupancyGrid;

  enum Direction { North, East, South, West };
  
  class MetricMap : public Singleton<MetricMap> {
    public:
      /* constructor */
      MetricMap(void);

      /* methods */
      void update_position(const gsl::vector& delta_pos, double delta_rot);
      
      void process_distances(PlayerCc::Position2dProxy& position_proxy, PlayerCc::LaserProxy& laser_proxy);
      void update_window(PlayerCc::Position2dProxy& position_proxy, PlayerCc::LaserProxy& laser_proxy);
      bool in_grid(const gsl::vector& coord);
      gsl::vector_int grid_position(void);

      static Direction vector2direction(const gsl::vector_int& v);
      static gsl::vector_int direction2vector(Direction dir);
      static Direction opposite_direction(Direction dir);

      // TODO: coord2index
      void save(void);
      void draw(bool draw_gateways = true);

      /* constants */
      static uint WINDOW_RADIUS_CELLS;
      static uint WINDOW_SIZE_CELLS;
      static double SENSOR_MODEL_DELTA;
      static double frontier_cell_threshold;

      OccupancyGrid* current_grid;

      gsl::vector position;
      gsl::matrix window, debug_window, temporary_matrix;
      double rotation;

      SuperMatrix<OccupancyGrid> super_matrix;
      
    private:
      enum NodeDirections { NorthEast, NorthWest, SouthEast, SouthWest };
      double sensor_model(double r, double delta);
  };
}

std::ostream& operator<<(std::ostream& out, HybNav::Direction dir);

#ifndef __TOPOMAP_H__

#include "topo_map.h"

#include "occupancy_grid.h"

#endif

#endif

