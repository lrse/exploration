#ifndef __METRIC_MAP_H__
#define __METRIC_MAP_H__

#include "place.h"
#include "graph.h"
#include "singleton.h"
#include <gslwrap/vector_double.h>
#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_int.h>
#include <libplayerc++/playerc++.h>

namespace HybNav {
  enum Direction { North, East, South, West };
  
  class MetricMap : public Singleton<MetricMap> {
    public:
      class Node;

      /* constructor */
      MetricMap(void);

      /* methods */
      void update_position(const gsl::vector& delta_pos, double delta_rot);
      Node* get_node(const gsl::vector_int& coord);
      Node* get_node(int x, int y) { gsl::vector_int coord(2); coord(0) = x; coord(1) = y; return get_node(coord); }
      
      void process_distances(PlayerCc::Position2dProxy& position_proxy, PlayerCc::LaserProxy& laser_proxy);
      void update_window(PlayerCc::Position2dProxy& position_proxy, PlayerCc::LaserProxy& laser_proxy);
      bool in_place(const gsl::vector& coord);
      gsl::vector_int grid_position(void);

      static Direction vector2direction(const gsl::vector_int& v);
      static gsl::vector_int direction2vector(Direction dir);

      // TODO: coord2index
      void save(void);

      /* constants */
      static uint WINDOW_CELLS;
      static uint WINDOW_HALF_CELLS;
      static double ROBOT_RADIUS;
      static double frontier_cell_threshold;

      Node* current_node;

      gsl::vector position;
      gsl::matrix window, debug_window, temporary_matrix;
      double rotation;
      
    private:
      Graph<Node> graph;
      enum NodeDirections { NorthEast, NorthWest, SouthEast, SouthWest };

      Node* find_node(const gsl::vector_int& coord);

  };
}

std::ostream& operator<<(std::ostream& out, HybNav::Direction dir);

#ifndef __TOPOMAP_H__

#include "topo_map.h"

namespace HybNav {
  class MetricMap::Node {
    public:
      Node(const gsl::vector_int& position);

      TopoMap::GatewayNode* find_gateway(gsl::vector_int pos, Direction edge);
      void update_gateways(bool and_connectivity = true);
      void update_connectivity(void);

      Direction direction_to(Node* other);
      gsl::vector_int position;

      void to_dot(std::ostream& out);

      typedef std::list<gsl::vector_int> FrontierList;
      void update_frontiers(void);
      FrontierList frontiers;

      Place place;

    private:
      std::list<TopoMap::GatewayNode*> gateway_nodes;

      typedef std::vector< std::list< std::pair<uint, uint> > > GatewayCoordinates;
      GatewayCoordinates detect_gateways(void);
  };
}

std::ostream& operator<<(std::ostream& out, const HybNav::MetricMap::Node* node);

#endif

#endif