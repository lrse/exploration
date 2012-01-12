#ifndef LOCAL_EXPLORER_H
#define	LOCAL_EXPLORER_H


#include "singleton.h"
#include "pathfinder.h"
#include "metric_map.h"

namespace HybNav {
  class LocalPathfinder : public Pathfinder<gsl::vector_int> {
    public:
      LocalPathfinder(double v);
      virtual ~LocalPathfinder(void) {  }
      std::list<gsl::vector_int> neighbors(const gsl::vector_int& current, const gsl::vector_int& previous);
      unsigned long movement_cost(const gsl::vector_int& from, const gsl::vector_int& to, const gsl::vector_int& previous) ;

      void prepare(void);
      
      void process_current_grid(void);
      uchar get_occupancy(uint i, uint j);
      cv::Mat grid;

    protected:
      double frontier_value_condition;

  };

  class FrontierPathfinder : public LocalPathfinder {
    public:
      FrontierPathfinder(void);
      bool is_goal(const gsl::vector_int& current);
  };

  class ConnectivityPathfinder : public LocalPathfinder {
    public:
      ConnectivityPathfinder(void);
      bool is_goal(const gsl::vector_int& current);
      gsl::vector_int x_range;
      gsl::vector_int y_range;
  };


  class LocalExplorer : public Singleton<LocalExplorer> {
    public:
      LocalExplorer(void);
      bool target_is_frontier(void);
      void sort_paths(void);
      void compute_frontier_paths(void);
      void clear_paths(void);
      bool found_path(void);
      bool other_paths_left(void);
      void follow_next_path(void);
      void compute_gateway_path(TopoMap::GatewayNode* gateway, bool follow = true);

      void print_all_paths(void);
      
      void update(void);
      bool valid_path(void);

      ConnectivityPathfinder connectivity_pathfinder;
      FrontierPathfinder frontier_pathfinder;

      std::list< std::list<gsl::vector_int> > all_paths;
      std::list<gsl::vector_int> follow_path;
      gsl::vector_int last_target;
      bool last_target_valid;
      bool found;
  };
}

std::ostream& operator<<(std::ostream& out, const std::list<gsl::vector_int>& l);

#endif	/* LOCAL_EXPLORER_H */


