#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <boost/shared_ptr.hpp>
#include <vector>
#include <list>
#include <stdexcept>
using boost::shared_ptr;

namespace HybNav {
  /************************************************
   *              Base Graph class                *
   ************************************************/
  template<class Node>
  class Graph {
    public:
      typedef typename std::list< shared_ptr<Node> > NodeArray;
      typedef typename std::list< std::pair<Node*, Node*> > EdgeArray;
      typedef typename NodeArray::iterator NodeIterator;
      typedef typename EdgeArray::iterator EdgeIterator;
      NodeArray nodes;
      EdgeArray edges;

      bool empty(void) const { return nodes.empty(); }
      uint size(void) const { return nodes.size(); }

      void add_node(shared_ptr<Node> n) {
        nodes.push_back(n);
      }

      void del_node(Node* n) {
        for (NodeIterator it = nodes.begin(); it != nodes.end(); ++it) {
          if (it->get() == n) {
            remove_all_edges(n);
            nodes.erase(it);
            return;
          }
        }
        throw std::runtime_error("Node not in graph");
      }

      void remove_all_edges(Node* n) {
        EdgeIterator it = edges.begin();
        while (it != edges.end()) {
          if (it->first == n || it->second == n)
            it = edges.erase(it);
          else
            ++it;
        }
      }

      void connect(Node* n1, Node* n2, bool ignore_existing = false) {
        for (EdgeIterator it = edges.begin(); it != edges.end(); ++it) {
          if ((it->first == n1 && it->second == n2) || (it->second == n1 && it->first == n2)) {
            if (ignore_existing) return;
            else throw std::runtime_error("Edge exists");
          }
        }
        edges.push_back(std::pair<Node*, Node*>(n1, n2));
      }

      void disconnect(Node* n1, Node* n2) {
        for (EdgeIterator it = edges.begin(); it != edges.end(); ++it) {
          if ((it->first == n1 && it->second == n2) || (it->second == n1 && it->first == n2)) {
            edges.erase(it);
            return;
          }
        }
        throw std::runtime_error("Nodes not connected");
      }

      bool is_connected(Node* n1, Node* n2) {
        for (EdgeIterator it = edges.begin(); it != edges.end(); ++it) {
          if ((it->first == n1 && it->second == n2) || (it->second == n1 && it->first == n2)) {
            return true;
          }
        }
        return false;
      }

      size_t node_index(Node* node) {
        size_t i = 0;
        for (NodeIterator it = nodes.begin(); it != nodes.end(); ++it, ++i) {
          if ((*it).get() == node) break;
        }
        return i;
      }

      void to_dot(std::ostream& out) {
        out << "graph G {" << std::endl;
        out << "\tnode [shape=\"circle\"]" << std::endl;

        for (NodeIterator it = nodes.begin(); it != nodes.end(); ++it) {
          out << "\t" << node_index(it->get()) << "[";
          (*it)->to_dot(out);
          out << "]" << std::endl;
        }

        for (EdgeIterator it = edges.begin(); it != edges.end(); ++it) {
          out << "\t" << node_index(it->first) << " -- " << node_index(it->second) << std::endl;
        }

        out << "}" << std::endl;
      }
  };
}

#endif