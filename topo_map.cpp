#include "metric_map.h"
#include "global_explorer.h"
using namespace HybNav;
using namespace std;

/**************************
 * Constructor/Destructor *
 **************************/

TopoMap::TopoMap(void) : Singleton<TopoMap>(this)
{
  current_node = add_area(MetricMap::instance()->current_grid);
}


/**************************
 *     Public Methods     *
 **************************/

TopoMap::AreaNode* TopoMap::add_area(OccupancyGrid* grid) {
  shared_ptr<Node> new_node(new AreaNode(grid));
  graph.add_node(new_node);
  return (AreaNode*)new_node.get();
}

TopoMap::GatewayNode* TopoMap::add_gateway(OccupancyGrid* grid, Direction edge, uint x0, uint xf) {
  shared_ptr<Node> new_node(new GatewayNode(grid, edge, x0, xf));
  graph.add_node(new_node);
  return (GatewayNode*)new_node.get();
}

void TopoMap::del_node(Node* node) {
  graph.del_node(node);
}

void TopoMap::connect(TopoMap::Node* node1, TopoMap::Node* node2) {
  if (graph.is_connected(node1, node2)) return;
  cout << "connecting " << node1 << " to " << node2 << endl;
  if (node1->is_area() && node2->is_gateway()) {
    AreaNode* gw_area_node = dynamic_cast<GatewayNode*>(node2)->area_node();
    if (gw_area_node) {
      cout << "Tried to connect area to gateway already connected to another area... merging area nodes" << endl;
      merge(dynamic_cast<AreaNode*>(node1), gw_area_node);
      return;
    }
  }

  graph.connect(node1, node2);
}

void TopoMap::disconnect(TopoMap::Node* node1, TopoMap::Node* node2) {
  if (graph.is_connected(node1, node2)) graph.disconnect(node1, node2);
}

void TopoMap::merge(TopoMap::AreaNode* area1, TopoMap::AreaNode* area2) {
  // bring connections from node to be deleted, before deleting it
  for (Graph<TopoMap::Node>::EdgeIterator it = graph.edges.begin(); it != graph.edges.end(); ++it) {
    if (it->first == area2 || it->second == area2) {
      TopoMap::Node* other = (it->first == area2 ? it->second : it->first);
      graph.connect(area1, other, true);
    }
  }
  graph.del_node(area2);
  
  // update GlobalExplorer paths to account for the node replacement
  list< list<TopoMap::Node*> >& all_paths = GlobalExplorer::instance()->all_paths;
  for(list< list<TopoMap::Node*> >::iterator p_it = all_paths.begin(); p_it != all_paths.end(); ++p_it) {
    replace(p_it->begin(), p_it->end(), area2, area1);
  }
  replace(GlobalExplorer::instance()->follow_path.begin(), GlobalExplorer::instance()->follow_path.end(), area2, area1);
  
  if (current_node == area2) current_node = area1;
}

void TopoMap::save(void) {
  cout << "Saving topo map" << endl;
  ofstream dot_file("csv/topo_map.dot", ios_base::trunc | ios_base::out);
  graph.to_dot(dot_file);
  dot_file.close();

  ofstream graphml_file("csv/topo_map.graphml", ios_base::trunc | ios_base::out);
  graph.to_graphml(graphml_file);
  graphml_file.close();
}

std::ostream& operator<<(std::ostream& out, const std::list<HybNav::TopoMap::Node*>& l) {
  for (list<HybNav::TopoMap::Node*>::const_iterator it = l.begin(); it != l.end(); ++it) {
    if (it != l.begin()) out << ", ";
    out << *it;
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const HybNav::TopoMap::Node* node) {
  if (node->is_area()) out << (TopoMap::AreaNode*)node;
  else out << (TopoMap::GatewayNode*)node;
  return out;
}


std::ostream& operator<<(std::ostream& out, const HybNav::TopoMap::AreaNode* node) {
  out << "area of " << node->grid->position;
  return out;
}

std::ostream& operator<<(std::ostream& out, const HybNav::TopoMap::GatewayNode* node) {
  out << "gw of " << node->grid << " at " << node->edge << " " << node->position();
  return out;
}

std::ostream& operator<<(std::ostream& out, HybNav::Direction dir) {
  switch(dir) {
    case North: out << "North"; break;
    case South: out << "South"; break;
    case East: out << "East"; break;
    case West: out << "West"; break;
  }
  return out;
}

void TopoMap::AreaNode::to_dot(std::ostream& out) {
  out << "label=\"" << grid->position << "\"";
}

void TopoMap::AreaNode::to_graphml(std::ostream& out) {
  out << "<node id=\"" << TopoMap::instance()->graph.node_index(this) << "\">" << endl;
  out << "  <data key=\"d0\">" << endl;
  out << "    <y:ShapeNode>" << endl;
  out << "      <y:NodeLabel alignment=\"center\" autoSizePolicy=\"content\" visible=\"true\">" << grid->position << "</y:NodeLabel>" << endl;
  out << "      <y:Shape type=\"ellipse\"/>" << endl;
  out << "    </y:ShapeNode>" << endl;
  out << "  </data>" << endl;
  out << "</node>" << endl;
}

void TopoMap::GatewayNode::set_dimensions(uint new_x0, uint new_xf) {
  x0 = new_x0;
  xf = new_xf;
  // the number of attempts equals to the number of robots that can fit into the gateway
  MAX_REACH_ATTEMPTS = MAX(2, (xf - x0) * (OccupancyGrid::CELL_SIZE / MetricMap::ROBOT_RADIUS));
}

gsl::vector_int TopoMap::GatewayNode::position(void) const {
  gsl::vector_int pos(2);
  if (edge == North || edge == South) { pos(0) = (uint)floor((xf + x0) * 0.5); pos(1) = (edge == North ? OccupancyGrid::CELLS - 1 : 0); }
  else { pos(1) = floor((xf + x0) * 0.5); pos(0) = (edge == East ? OccupancyGrid::CELLS - 1 : 0); }
  return pos;
}

void TopoMap::GatewayNode::get_ranges(gsl::vector_int& x_range, gsl::vector_int& y_range) {
  if (edge == North || edge == South) {
    x_range(0) = x0; x_range(1) = xf;
    y_range(0) = y_range(1) = (edge == North ? OccupancyGrid::CELLS - 1 : 0);
  }
  else {
    y_range(0) = x0; y_range(1) = xf;
    x_range(0) = x_range(1) = (edge == East ? OccupancyGrid::CELLS - 1 : 0);
  }
}

void TopoMap::GatewayNode::to_dot(std::ostream& out) {
  out << "label=\"" << edge << " [" << x0 << ":" << xf << "] of " << grid->position << "\",shape=\"box\"";
}

void TopoMap::GatewayNode::to_graphml(std::ostream& out) {
  out << "<node id=\"" << TopoMap::instance()->graph.node_index(this) << "\">" << endl;
  out << "  <data key=\"d0\">" << endl;
  out << "    <y:ShapeNode>" << endl;
  out << "      <y:NodeLabel alignment=\"center\" autoSizePolicy=\"content\" visible=\"true\">" << edge << "[" << x0 << ":" << xf << "] of " << grid->position << "</y:NodeLabel>" << endl;
  out << "      <y:Shape type=\"rectangle\"/>" << endl;
  out << "    </y:ShapeNode>" << endl;
  out << "  </data>" << endl;
  out << "</node>" << endl;
}

bool TopoMap::GatewayNode::unexplored_gateway(void) {  
  // try to find the gateway node connected to this one
  TopoMap::GatewayNode* connected_gateway = NULL;  
  Graph<TopoMap::Node>::EdgeArray& edges = TopoMap::instance()->graph.edges;
  for(Graph<TopoMap::Node>::EdgeIterator it = edges.begin(); it != edges.end(); ++it) {
    if (it->first == this && it->second->is_gateway()) connected_gateway = (TopoMap::GatewayNode*)it->second;
    else if (it->second == this && it->first->is_gateway()) connected_gateway = (TopoMap::GatewayNode*)it->first;
  }
  
  // there is no gateway connected, so try to see if there's at least a corresponding gateway node (but not connected)
  if (!connected_gateway) {
    OccupancyGrid& adjacent_grid = this->grid->get_neighbor(edge);
    gsl::vector_int adj_position = position() + MetricMap::direction2vector(edge);
    for (uint i = 0; i < 2; i++) { if (adj_position(i) < 0) adj_position(i) += OccupancyGrid::CELLS; else adj_position(i) %= OccupancyGrid::CELLS; }
    connected_gateway = adjacent_grid.find_gateway(adj_position, MetricMap::opposite_direction(edge), true);
    // no gateway node, let's assume that this gateway leads to unexplored area then
    if (!connected_gateway) { cout << this << " has no adj area node" << endl; return true; }
  }
  
  // a corresponding gateway node was found (either connected or not)
  AreaNode* area_node = connected_gateway->area_node();
  if (!area_node || !area_node->completely_explored) { cout << this << " has no adj area node or this area node is unexplored" << endl; return true; }
  else { cout << this << " has adj explored area node (" << area_node << ")" << endl; return false; }
}


