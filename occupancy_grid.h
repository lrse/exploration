#ifndef __OCCUPANCY_GRID_H__
#define	__OCCUPANCY_GRID_H__

#include <gslwrap/matrix_double.h>
#include <gslwrap/vector_int.h>
#include <opencv2/core/core.hpp>

namespace HybNav {
  class OccupancyGrid {
    public:
      OccupancyGrid(ssize_t x, ssize_t y);

      double& operator()(uint x, uint y);
      static bool valid_coordinates(int x, int y);
      static gsl::vector_int world2grid(const gsl::vector& coord);

      TopoMap::GatewayNode* find_gateway(gsl::vector_int pos, Direction edge, bool accept_nonexistant = false);
      TopoMap::GatewayNode* find_gateway(const std::pair<uint,uint>& range, Direction edge, bool accept_nonexistant = false);
      void update_gateways(bool and_connectivity = true);
      void update_connectivity(void);

      OccupancyGrid& get_neighbor(Direction edge);

      Direction direction_to(OccupancyGrid* other);
      gsl::vector_int position;

      void to_dot(std::ostream& out);
      void draw(cv::Mat& graph, bool draw_gateways = true);

      typedef std::list<gsl::vector_int> FrontierList;
      void update_frontiers(void);
      FrontierList frontiers;

      gsl::matrix m;

      static gsl::vector_int rowcol2xy(gsl::vector_int rowcol);
      
      cv::Mat debug_graph;

      bool intersecting_gateways(int x0, int xf, int n_x0, int n_xf);

    private:
      std::list<TopoMap::GatewayNode*> gateway_nodes;
      OccupancyGrid(void);

      typedef std::vector< std::list< std::pair<uint, uint> > > GatewayCoordinates;
      void detect_gateways(void);
      GatewayCoordinates gateway_coordinates;

      bool gateway_condition(uint i, uint j, uint d);

    public:
      /* constants */
      static double CELL_SIZE;
      static uint CELLS;
      static double SIZE;
      static double Locc;
      static double Lfree;
      static uint GATEWAY_LOOKAHEAD_CELLS;
      static uint MINIMUM_GATEWAY_CELLS;
  };
}

std::ostream& operator<<(std::ostream& out, const HybNav::OccupancyGrid* node);


#endif


