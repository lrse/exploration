#ifndef GLOBAL_EXPLORER_H
#define	GLOBAL_EXPLORER_H

#include <list>
#include "pathfinder.h"
#include "topo_map.h"
#include "singleton.h"

namespace HybNav {
  class GlobalExplorer : public Singleton<GlobalExplorer> {
    public:
      GlobalExplorer(void);

      void follow_next_path(void);
      void clear_paths(void);
      bool found_path(void);
      bool other_paths_left(void);
      void recompute_paths(void);

      std::list< std::list<TopoMap::Node*> > all_paths;
      std::list<TopoMap::Node*> follow_path;

    private:
      class GlobalPathfinder : public Pathfinder<TopoMap::Node*> {
        public:
          std::list<TopoMap::Node*> neighbors(TopoMap::Node* const & current, TopoMap::Node* const& previous);
          bool is_goal(TopoMap::Node* const& current);
      };

      bool found;
      GlobalPathfinder pathfinder;
  };
}

#endif	/* GLOBAL_EXPLORER_H */


