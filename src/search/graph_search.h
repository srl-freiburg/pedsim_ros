
#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/config.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>

#include <tuple>
#include <algorithm>
#include <iterator>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <simple_utils.h>
#include "trajectory.h"


template<typename T>
struct decltype_t
{
    typedef T type;
};

#define DECLTYPE(expr) decltype_t<decltype(expr)>::type

// contains the cost from the foal to a cell/node
typedef struct DijkstraCost
{
    int node;
    double cost;

    DijkstraCost(int n, double c) : node(n), cost(c) {}

} DCost;

// edges has weight property
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

// nodes have a name property corresponding the node id
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;

// the actual graph
typedef boost::adjacency_list < boost::listS,
                                boost::vecS,
                                boost::undirectedS,
                                NameProperty,
                                EdgeWeightProperty >
    Graph;

typedef boost::graph_traits < Graph >::vertex_descriptor graph_vertex;
typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

typedef boost::property_map < Graph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < Graph, boost::vertex_name_t >::type NameMap;
typedef boost::iterator_property_map < graph_vertex*, IndexMap, graph_vertex, graph_vertex& > PredecessorMap;
typedef boost::iterator_property_map < double*, IndexMap, double, double& > DistanceMap;


struct GridCell {
    GridCell() {}
    GridCell(float xx, float yy) : x(xx), y(yy) {}
    float x, y;
};

/// Exception thrown when the goal vertex is found
struct found_goal {};


/// Euclidean heuristic for a grid
template <class MutableGraph, class CostType, class LocMap>
class euclidean_heuristic : public boost::astar_heuristic<MutableGraph, CostType> {
public:
    typedef typename boost::graph_traits<MutableGraph>::vertex_descriptor Vertex;

    euclidean_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}

    CostType operator()(Vertex u) {
        CostType dx = m_location[m_goal].x - m_location[u].x;
        CostType dy = m_location[m_goal].y - m_location[u].y;
        return ::sqrt(dx * dx + dy * dy);
    }
private:
    LocMap m_location;
    Vertex m_goal;
};

/// Weighted euclidiean heuristic
template <class MutableGraph, class CostType, class LocMap>
class weighted_euclidean_heuristic : public boost::astar_heuristic<MutableGraph, CostType> {
public:
    typedef typename boost::graph_traits<MutableGraph>::vertex_descriptor Vertex;

    weighted_euclidean_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}

    CostType operator()(Vertex u) {
        CostType dx = m_location[m_goal].x - m_location[u].x;
        CostType dy = m_location[m_goal].y - m_location[u].y;

        // NOTE - weighting already doen in creating graph edges
        double weight = CONFIG.costmatrix->getValGrid(m_location[u].x, m_location[u].y, CONFIG.width, CONFIG.height);
        // double weight = CONFIG.cost_map[std::make_pair(m_location[u].x, m_location[u].y)];
        double scaled_weight = (weight - CONFIG.fmin) / ((CONFIG.fmax - CONFIG.fmin)+1e-30);

        double mid = sqrt(CONFIG.width*CONFIG.width + CONFIG.height*CONFIG.height);

        return ::sqrt(dx * dx + dy * dy) * (1*(scaled_weight) + mid);
    }
private:
    LocMap m_location;
    Vertex m_goal;
};




/// Visitor that terminates when we find the goal vertex
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class MutableGraph>
    void examine_vertex(Vertex u, MutableGraph& g) {
        if(u == m_goal) {
            throw found_goal();
        }
    }
private:
    Vertex m_goal;
};




class GraphSearch
{
public:
    explicit GraphSearch() {
        node_list_.clear();
        edge_list_.clear();
    }
    ~GraphSearch() {}

    void createGraphFromGrid(Grid* grid, bool weighted=false, bool replanning=false);

    std::vector<GridCell> getSinglePathDijkstra(Tpoint start, Tpoint goal);

    std::vector<GridCell> computePathAStar(Tpoint start_, Tpoint goal_);

    std::vector<GridCell> computePathWeightedAStar(Tpoint start_, Tpoint goal_);

    std::map<int, double> computeAllPathsToGoal(Grid *grid, Tpoint goal);

    void writeGraphToFile(std::string filename);

    Graph getGraph() { return graph_; }


private:
    // keep the list of nodes
    std::vector<graph_vertex> node_list_;

    // keep edges and their costs
    std::map<Edge, double> edge_list_;

    // list of grid cells
    GridCell* grid_cells_;

    Graph graph_;
    int cells_per_row;

    inline graph_vertex getVertex(unsigned int loc) {
        if (loc >= node_list_.size()) {
            // important to keep this check
            std::cerr << "Index out or range for node list: " << loc << std::endl;
            return node_list_[node_list_.size() - 1];
        }

        return node_list_[loc];
    }

    inline graph_vertex cellToVertex(Tpoint cell) {
        int xmod = (int)floor(cell.x / CONFIG.width);
        int ymod = (int)floor(cell.y / CONFIG.height);
        int v  = (xmod) + ( (ymod) * cells_per_row);
        return getVertex(v);
    }
};

/// helpers
typedef boost::shared_ptr<GraphSearch> GraphSearchPtr;
typedef boost::shared_ptr<GraphSearch const> GraphSearchConstPtr;



inline std::vector<Tpoint> getNeighbors(Tpoint n, Grid* grid)
{
    std::vector<Tpoint> neighbors;
    neighbors.clear();

    // the eight possible directions (costs are euclidean distances 
    // so that diagonal cells are automatically taken care of)
    if (withinGrid(n.x, n.y+CONFIG.height, grid))
    {
        Tpoint tn(n.x, n.y+CONFIG.height, 0);
        if (!grid->isOccupied(tn.x, tn.y) )
        {
            neighbors.push_back(tn);
        }
    }

   if (withinGrid(n.x-CONFIG.width,n.y+CONFIG.height, grid))
   {
       Tpoint tn(n.x-CONFIG.width,n.y+CONFIG.height, 0);
       if (!grid->isOccupied(tn.x, tn.y) )
       {
           neighbors.push_back(tn);
       }
   }

    if (withinGrid(n.x+CONFIG.width,n.y+CONFIG.height, grid))
    {
        Tpoint tn(n.x+CONFIG.width,n.y+CONFIG.height, 0);
        if (!grid->isOccupied(tn.x, tn.y) )
        {
            neighbors.push_back(tn);
        }
    }

   if (withinGrid(n.x-CONFIG.width,n.y, grid))
   {
       Tpoint tn(n.x-CONFIG.width,n.y, 0);
       if (!grid->isOccupied(tn.x, tn.y) )
       {
           neighbors.push_back(tn);
       }
   }

    if (withinGrid(n.x+CONFIG.width,n.y, grid))
    {
        Tpoint tn(n.x+CONFIG.width,n.y, 0);
        if (!grid->isOccupied(tn.x, tn.y) )
        {
            neighbors.push_back(tn);
        }
    }

    // 6
    if (withinGrid(n.x,n.y-CONFIG.height, grid))
    {
        Tpoint tn(n.x,n.y-CONFIG.height, 0);
        if (!grid->isOccupied(tn.x, tn.y) )
        {
            neighbors.push_back(tn);
        }
    }

   if (withinGrid(n.x-CONFIG.width,n.y-CONFIG.height, grid))
   {
       Tpoint tn(n.x-CONFIG.width,n.y-CONFIG.height, 0);
       if (!grid->isOccupied(tn.x, tn.y) )
       {
           neighbors.push_back(tn);
       }
   }

    if (withinGrid(n.x+CONFIG.width,n.y-CONFIG.height, grid))
     {
        Tpoint tn(n.x+CONFIG.width,n.y-CONFIG.height, 0);
        if (!grid->isOccupied(tn.x, tn.y) )
        {
            neighbors.push_back(tn);
        }
    }

    return neighbors;
}

/// Compute gradients of costmap from Dijkstra (using Sobel derivatives)
cv::Mat computeGradient(Grid* grid, std::map<int, double> dcosts, int direction);
/// Write costmap to a file (for debugging)
void writeMatToFile(cv::Mat& m, const char* filename);


/// Get the lowest cost cell around the neighborhood of the robot
Tpoint getLowestCostCell(Ped::Tagent* robot, cv::Mat cmap, Grid* grid);

Tpoint getLowestCostNeighbor(cv::Mat cmap, Tpoint cc, Grid *grid);


// better representation with cell indexing(matrix style)
inline boost::numeric::ublas::matrix<double> getCostMatrix(cv::Mat cmap)
{
    boost::numeric::ublas::matrix<double> cm(cmap.rows, cmap.cols);

    for(unsigned int i=0; i<cmap.rows; i++) {
        for(unsigned int j=0; j<cmap.cols; j++) {
            cm(i, j) =  cmap.at<float>(i, j);
        }
    }

    return cm;
}


#endif // GRAPH_SEARCH_H
