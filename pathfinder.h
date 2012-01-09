#ifndef PATHFINDER_H
#define	PATHFINDER_H

#include <map>
#include <list>

namespace HybNav {
  template <class T>
  class Pathfinder {
    public:
      virtual std::list<T> neighbors(const T& current, const T& predecessor) = 0;
      virtual unsigned long movement_cost(const T& from, const T& to, const T& previous) { return 1; }
      virtual unsigned long distance_heuristic(const T& target) { return 0; }
      virtual bool is_goal(const T& current) { return false; }
      virtual ~Pathfinder(void) { }
      virtual void prepare(void) { }

      bool exists_path(const T& start, const T& goal);
      
      std::list< std::list<T> > findpath(const T& start, const T& goal, bool first_solution = false);
      std::list< std::list<T> > findpath(const T& start, bool first_solution = false);

    private:
      std::list<T> reconstruct_path(std::map<T,T>& predecessors, const T& element);
      std::list< std::list<T> > findpath(const T& start, const T& goal, bool use_goal_function, bool first_solution);

      class priority_element {
        public:
          priority_element(T _element, unsigned long _priority) : element(_element), priority(_priority) { }
          T element;
          unsigned long priority;

          bool operator<(const priority_element& other) const { return (priority >= other.priority); }
      };
  };
}

#include <gslwrap/vector_int.h>

namespace gsl {
  bool operator<(const vector_int& a, const vector_int& b);
}

#endif	/* PATHFINDER_H */

