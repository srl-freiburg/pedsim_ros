#ifndef SEARCH_MAZE_H
#define SEARCH_MAZE_H

//#include <search/search_utils.h>
#include "../src/trajectory.h"
#include <grid.h>
#include "../src/config.h"



#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <iostream>


// Distance traveled in the maze
typedef double maze_distance;

#define GRID_RANK 2
typedef boost::grid_graph<GRID_RANK> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

// A hash function for vertices.
struct vertex_hash:std::unary_function<vertex_descriptor, std::size_t> {
    std::size_t operator()(vertex_descriptor const& u) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, u[0]);
        boost::hash_combine(seed, u[1]);
        return seed;
    }
};

typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;



/// evaluate the cost map from the current vertex to the goal
/// by summing the costs along a line joining them (bresenham)
double evalCostMap(vertex_descriptor u, vertex_descriptor v);




// Euclidean heuristic for a grid
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic : public boost::astar_heuristic<filtered_grid, double>
{
public:
    euclidean_heuristic(vertex_descriptor goal)
        : m_goal(goal) {}

    double operator()(vertex_descriptor v)
    {
        return sqrt(pow(m_goal[0] - v[0], 2) + pow(m_goal[1] - v[1], 2));
    }

private:
    vertex_descriptor m_goal;
};


// Weighted euclidiean heuristic
class weighted_euclidean_heuristic : public boost::astar_heuristic<filtered_grid, double>
{
public:
    weighted_euclidean_heuristic(vertex_descriptor goal)
        : m_goal(goal) {}

    double operator()(vertex_descriptor v)
    {
//        return sqrt(pow(m_goal[0] - v[0], 2) + pow(m_goal[1] - v[1], 2)) * (evalCostMap(v, m_goal));

//        std::cout << "Vertex: " << v[0]*cellwidth << ", " << v[1]*cellheight << ": " << CONFIG.cost_map[std::make_pair(v[0]*cellheight,v[1]*cellwidth)] << std::endl;

        return sqrt(pow(m_goal[0] - v[0], 2) + pow(m_goal[1] - v[1], 2)) * (CONFIG.cost_map[std::make_pair(v[1]*cellheight,v[0]*cellwidth)]);

    }

private:
    vertex_descriptor m_goal;
};


// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor:public boost::default_astar_visitor
{
    astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {}

    void examine_vertex(vertex_descriptor u, const filtered_grid&)
    {
        if (u == m_goal)
            throw found_goal();
    }

private:
    vertex_descriptor m_goal;
};



// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
class SearchMaze
{
public:
    friend std::ostream& operator<<(std::ostream&, const SearchMaze&);
    friend SearchMaze random_maze(std::size_t, std::size_t);
    friend SearchMaze createMazeFromGrid(Grid* grid, Tpoint start, Tpoint goal);
    friend Trajectory* writeTraj(const SearchMaze& m, Grid *grid);

    SearchMaze()
        : m_grid(create_grid(0, 0)),
          m_obstacle_grid(create_barrier_grid())
    {
        found_path.clear();
    }
    SearchMaze(std::size_t x, std::size_t y)
        : m_grid(create_grid(x, y)),
          m_obstacle_grid(create_barrier_grid())
    {
        found_path.clear();
    }

    // The length of the maze along the specified dimension.
    vertices_size_type length(std::size_t d) const
    {
        return m_grid.length(d);
    }

    bool has_barrier(vertex_descriptor u) const
    {
        return m_obstacles.find(u) != m_obstacles.end();
    }

    // Try to find a path from the lower-left-hand corner source (0,0) to the
    // upper-right-hand corner goal (x-1, y-1).
    vertex_descriptor source() const
    {
        return start_point;
//        return vertex(0, m_grid);
    }

    vertex_descriptor goal() const
    {
        return goal_point;
//        return vertex(num_vertices(m_grid)-1, m_grid);
    }

    void setGoal(vertex_descriptor g)
    {
        goal_point = g;
    }

    void setStart(vertex_descriptor s)
    {
        start_point = s;
    }

    bool solveWithAstar();
    bool solveWithThetaStar();
    bool solveWithDijkstra();

    bool solved() const {return !m_solution_path.empty();}

    std::vector<vertex_descriptor> getFoundPath()
    {
        return found_path;
    }

    bool solution_contains(vertex_descriptor u) const
    {
        return m_solution_path.find(u) != m_solution_path.end();
    }

    std::vector<vertex_descriptor> found_path;

private:
    // Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y)
    {
        boost::array<std::size_t, GRID_RANK> lengths = { {x, y} };
        return grid(lengths);
    }

    // Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid()
    {
        return boost::make_vertex_subset_complement_filter(m_grid, m_obstacles);
    }

    // The grid underlying the maze
    grid m_grid;
    // The underlying maze grid with obstacle vertices filtered out
    filtered_grid m_obstacle_grid;
    // The obstacles in the maze
    vertex_set m_obstacles;
    // The vertices on a solution path through the maze
    vertex_set m_solution_path;

    // The length of the solution path
    maze_distance m_solution_length;

    // goal and start
    vertex_descriptor goal_point, start_point;
};







std::size_t random_int(std::size_t a, std::size_t b);
std::ostream& operator<<(std::ostream& output, const SearchMaze& m);
SearchMaze random_maze(std::size_t x, std::size_t y);
SearchMaze createMazeFromGrid(Grid* grid, Tpoint start, Tpoint goal);
Trajectory* writeTraj(const SearchMaze& m, Grid *grid);
Trajectory* writePath(const SearchMaze& m, Grid *grid);




#endif // SEARCH_MAZE_H
