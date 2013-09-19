#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include "singleton.h"
#include "metric_map.h"
#include "motion_planner.h"

namespace HybNav {
  class ExplorerException : public std::runtime_error {
    public:
      enum ExceptionCode { END, FAIL };
      ExplorerException(ExceptionCode code, const std::string& reason);
      virtual ~ExplorerException(void) throw() { }
      ExceptionCode code;
  };
  
  class Explorer : public Singleton<Explorer> {
    public:
      OccupancyGrid* current_grid;

      Explorer(void);
      void update(void);
      void compute_motion(PlayerCc::Position2dProxy& position_proxy);

      /* finite state machine */
      enum State { ExploringLocally, ExploringGlobally };
      State state;
      
      void start(State state);
      void fsm_advance(void);

      void while_exploring_locally(void);
      void start_exploring_globally(void);
      void while_exploring_globally(void);

      bool recompute_local_path(void);
      void recompute_path(void);
  };
}

#endif

