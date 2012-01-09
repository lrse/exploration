#include <queue>
#include <set>
#include "topo_map.h"
#include "metric_map.h"
#include "pathfinder.h"
using namespace HybNav;
using namespace std;

template<class T>
class priority_queue2 : public priority_queue<T> {
  public:
    vector<T>& container(void) { return priority_queue<T>::c; }
};

template<class T>
list< list<T> > Pathfinder<T>::findpath(const T& start, const T& goal, bool first_solution) {
  return findpath(start, goal, false, true/*first_solution*/);
}

template<class T>
list< list<T> > Pathfinder<T>::findpath(const T& start, bool first_solution) {
  return findpath(start, start, true, true/*first_solution*/);
}

template<class T>
bool Pathfinder<T>::exists_path(const T& start, const T& goal) {
  return !findpath(start, goal, false, true).empty();
}

template<class T>
list< list<T> > Pathfinder<T>::findpath(const T& start, const T& goal, bool use_goal_function, bool first_solution)
{
  prepare();
  
  map<T,T> predecessors;
  set<T> visited;
  map<T,unsigned long> g;
  g[start] = 0;

  list< list<T> > solutions;
  bool dijkstra = use_goal_function;

  priority_queue2<priority_element> pqueue;
  pqueue.push(priority_element(start, g[start] + distance_heuristic(start)));

  while (!pqueue.empty()) {
    priority_element current = pqueue.top(); pqueue.pop();
    if (visited.find(current.element) != visited.end()) continue;
    else visited.insert(current.element);

    //cout << "current_elemen: " << current.element << " " << goal << endl;
    if ((dijkstra && is_goal(current.element)) || (!dijkstra && current.element == goal)) {
      list<T> path = reconstruct_path(predecessors, current.element);
      solutions.push_back(path);
      if (dijkstra) {
        if (first_solution) return solutions; else continue;
      }
      else return solutions;
    }

    const T& predecessor = (predecessors.find(current.element) == predecessors.end() ? current.element : predecessors[current.element]);
    list<T> neighbor_list = neighbors(current.element, predecessor);
    for (typename list<T>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); ++it) {
      T neighbor = (*it);
      if (visited.find(neighbor) != visited.end()) continue;

      unsigned long tentative_cost = g[current.element] + movement_cost(current.element, neighbor, predecessor);
      bool found = false;
      size_t i;
      for (i = 0; i < pqueue.container().size(); i++) { if (pqueue.container()[i].element == neighbor) { found = true; break; } }
      if (!found || tentative_cost < g[neighbor]) {
        g[neighbor] = tentative_cost;
        predecessors[neighbor] = current.element;
        unsigned long priority = g[neighbor] + distance_heuristic(neighbor);
        if (!found) pqueue.push(priority_element(neighbor, priority));
        else { pqueue.container()[i].priority = priority; make_heap(pqueue.container().begin(), pqueue.container().end()); }
      }
    }
  }

  return solutions;
}


template<class T>
list<T> Pathfinder<T>::reconstruct_path(map<T,T>& predecessors, const T& goal) {
  list<T> path;
  path.push_back(goal);

  while(true) {
    if (predecessors.find(path.front()) == predecessors.end()) break;
    else {
      T& previous = predecessors[path.front()];
      path.push_front(previous);
    }
  }
  return path;
}

template class Pathfinder<TopoMap::Node*>;
template class Pathfinder<gsl::vector_int>;

namespace gsl {
  bool operator<(const vector_int& a, const vector_int& b) { return (a(0) < b(0) || (a(0) == b(0) && a(1) < b(1))); }
}
