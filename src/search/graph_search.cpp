
#include <search/graph_search.h>

void GraphSearch::createGraphFromGrid(Grid* grid, bool weighted, bool replanning)
{
    node_list_.clear();
    edge_list_.clear();

    /// create the nodelist
    // each cell in the grid gets a node
    double cx = grid->minx;
    double cy = grid->miny;

    cells_per_row = (int)grid->width / CONFIG.width;
    int cells_per_col = grid->height / CONFIG.height;

    grid_cells_ = new GridCell[cells_per_col*cells_per_row];

    // std::cout << "Cells per row: " << cells_per_row << std::endl;

    int nodeid = 0;
    while (cy < grid->height ) {
        while(cx < grid->width) {
            // check if occupied
            graph_vertex v = boost::add_vertex(boost::lexical_cast<std::string>(nodeid), graph_);
            node_list_.push_back(v);

            grid_cells_[nodeid] = GridCell(cx, cy);

            cx += CONFIG.width;
            nodeid++;
        }
        cy += CONFIG.height;
        cx = grid->minx;
    }

    //std::cout << "created " << node_list_.size() << " nodes" << std::endl;

    /// add edges to the graph
    nodeid = 0;
    cx = grid->minx;
    cy = grid->miny;
    int edge_count = 0;

    while (cy < grid->height ) {
        while(cx < grid->width) {
            // get neighbors of the current cell
            std::vector<Tpoint> neighbors = getNeighbors(Tpoint(cx, cy, 0), grid);

            foreach(Tpoint p, neighbors) {

                // get the vertex for the neighbor
                int nv = (int)(p.x / CONFIG.width) + ( (int)(p.y / CONFIG.height) * cells_per_row);

                if (nv == node_list_.size()) continue;

                // check for loops
                if (getVertex(nv) == getVertex(nodeid))
                    continue;

                // each node can only have max 8 edges
                if (boost::degree(getVertex(nodeid), graph_) >= 8 ||  boost::degree(getVertex(nv), graph_) >= 8)
                    continue;

                // checkk if an edge with the same nodes already exists
                std::map<Edge, double>::iterator it1 = edge_list_.find(std::make_pair( getVertex(nodeid), getVertex(nv) ));
                std::map<Edge, double>::iterator it2 = edge_list_.find(std::make_pair( getVertex(nv), getVertex(nodeid) ));
                if (it1 == edge_list_.end() && it2 == edge_list_.end()) {

                    GridCell a = grid_cells_[getVertex(nodeid)];
                    GridCell b = grid_cells_[getVertex(nv)];

                    if (weighted) {
                        double wt = CONFIG.costmatrix->getValGrid(p.x, p.y, CONFIG.width, CONFIG.height);

                        if (wt > CONFIG.fmax) wt = CONFIG.fmax;
                        double scaled_weight = (wt - CONFIG.fmin) / ((CONFIG.fmax - CONFIG.fmin));
                        if (scaled_weight < 0.0) scaled_weight = 0.0;
                        double mid = sqrt(CONFIG.width*CONFIG.width + CONFIG.height*CONFIG.height);

                        EdgeWeightProperty weight(eDist(a.x, a.y, b.x, b.y) * ((100*scaled_weight) + mid) );
                        boost::add_edge(getVertex(nodeid), getVertex(nv), weight, graph_);
                        edge_count++;
                        std::pair<int, int> nedge = std::make_pair( getVertex(nodeid), getVertex(nv) );
                        edge_list_.insert(std::make_pair( nedge, eDist(a.x, a.y, b.x, b.y) * (scaled_weight + mid) ));

                    } else {
                        EdgeWeightProperty weight(eDist(a.x, a.y, b.x, b.y));
                        boost::add_edge(getVertex(nodeid), getVertex(nv), weight, graph_);
                        edge_count++;
                        std::pair<int, int> nedge = std::make_pair( getVertex(nodeid), getVertex(nv) );
                        edge_list_.insert(std::make_pair( nedge, eDist(a.x, a.y, b.x, b.y) ));
                    }
                }
                else {
                    continue;
                }

            }

            cx += CONFIG.width;
            nodeid++;
        }
        cy += CONFIG.height;
        cx = grid->minx;
    }
}


std::vector<GridCell> GraphSearch::getSinglePathDijkstra(Tpoint start, Tpoint goal)
{
    std::vector<GridCell> vpath;

    // Create things for Dijkstra
    std::vector<graph_vertex> predecessors(boost::num_vertices(graph_)); // To store parents
    std::vector<double> distances(boost::num_vertices(graph_)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, graph_);
    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);

    // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
    boost::dijkstra_shortest_paths(graph_, cellToVertex(goal), boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    // Extract a shortest path
    typedef std::vector<Graph::edge_descriptor> PathType;
    PathType path;

    graph_vertex v = cellToVertex(start);

    for(graph_vertex u = predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
        u != v; // Keep tracking the path until we get to the source
        v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph_);
        Graph::edge_descriptor edge = edgePair.first;

        path.push_back( edge );
    }

    //Write shortest path
    PathType::reverse_iterator pIterator = path.rbegin();
    vpath.push_back(grid_cells_[boost::source(*pIterator, graph_)]);

    for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator) {
        vpath.push_back(grid_cells_[boost::target(*pathIterator, graph_)]);
    }

    return vpath;
}

std::vector<GridCell> GraphSearch::computePathAStar(Tpoint start_, Tpoint goal_)
{
    using namespace boost;
    using namespace std;

    std::vector<GridCell> path; path.clear();

    std::vector<graph_vertex> p(num_vertices(graph_));
    std::vector<double> d(num_vertices(graph_));

    // start, goal, heuristic and visitor
    graph_vertex start = cellToVertex(start_);
    graph_vertex goal = cellToVertex(goal_);

    // search
    try
    {
        astar_search(graph_,
                     start,
                     euclidean_heuristic<Graph, double, GridCell*>(grid_cells_, goal),
                     predecessor_map(&p[0]).distance_map(&d[0]).visitor(astar_goal_visitor<graph_vertex>(goal))
                     );
        cout << endl << "No Path found." << endl << endl;
    }
    catch(found_goal fg)
    {
        // Walk backwards from the goal through the predecessor chain adding
        // vertices to the solution path.
        for (graph_vertex u = goal; u != start; u = p[u]) {
            path.push_back(grid_cells_[u]);
        }
        path.push_back(grid_cells_[start]);
    }

    return path;
}

std::vector<GridCell> GraphSearch::computePathWeightedAStar(Tpoint start_, Tpoint goal_)
{
    using namespace boost;
    using namespace std;

    std::vector<GridCell> path; path.clear();

    std::vector<graph_vertex> p(num_vertices(graph_));
    std::vector<double> d(num_vertices(graph_));

    // start, goal, heuristic and visitor
    graph_vertex start = cellToVertex(start_);
    graph_vertex goal = cellToVertex(goal_);

    // search
    try
    {
        astar_search(graph_,
                     start,
                     weighted_euclidean_heuristic<Graph, double, GridCell*>(grid_cells_, goal),
                     predecessor_map(&p[0]).distance_map(&d[0]).visitor(astar_goal_visitor<graph_vertex>(goal))
                     );
        cout << endl << "No Path found." << endl << endl;
    }
    catch(found_goal fg)
    {
        // Walk backwards from the goal through the predecessor chain adding
        // vertices to the solution path.
        for (graph_vertex u = goal; u != start; u = p[u]) {
            path.push_back(grid_cells_[u]);
        }
        path.push_back(grid_cells_[start]);
    }

    return path;    // always return a path regardless
}


std::map<int, double> GraphSearch::computeAllPathsToGoal(Grid* grid, Tpoint goal)
{
    std::map<int, double> all_costs;
    CONFIG.dmin = 10000.0;
    CONFIG.dmax = 0.0;

    // Create things for Dijkstra
    std::vector<graph_vertex> predecessors(boost::num_vertices(graph_)); // To store parents
    std::vector<double> distances(boost::num_vertices(graph_)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, graph_);
    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);

    // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
    boost::dijkstra_shortest_paths(graph_, cellToVertex(goal), boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    BGL_FORALL_VERTICES(v, graph_, Graph) {
        all_costs[(int)getVertex(v)] = distanceMap[v];

        if (distanceMap[v] < CONFIG.dmin) CONFIG.dmin = distanceMap[v];
        if ((distanceMap[v] > CONFIG.dmax) && (distanceMap[v] < 1e06)) CONFIG.dmax = distanceMap[v];
    }

    // make obstacles maximum cost cells
    double cx = grid->minx;
    double cy = grid->miny;
    while (cx < grid->width ) {
        while(cy < grid->height) {
            if (grid->isOccupied(cx, cy)) {
                graph_vertex v = cellToVertex(Tpoint(cx, cy, 0));
                all_costs[getVertex(v)] = CONFIG.dmax;
            }
            cy += CONFIG.height;
        }
        cx += CONFIG.width;
        cy = grid->miny;
    }

    return all_costs;
}


void GraphSearch::writeGraphToFile(std::string filename)
{
    std::ofstream dotfile(filename.c_str());

    boost::write_graphviz(dotfile, graph_);

    dotfile.close();
}


cv::Mat computeGradient(Grid* grid, std::map<int, double> dcosts, int direction)
{
    int cpr = grid->width / CONFIG.width;
    int cpc = grid->height / CONFIG.height;
    int cells_per_row = (int)floor(grid->width / CONFIG.width);


    cv::Mat mcost(cpc, cpr, CV_32FC1, cv::Scalar(100.0));

    for(int i = 0; i < dcosts.size(); i++) {
        std::div_t res = std::div(i, cells_per_row);
        int cx = res.rem;
        int cy = res.quot;

        mcost.at<float>(cy, cx) =  dcosts[i];
    }

    /// compute the gradient
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32F;
    int kernel_size = 5;

    if (direction == 0) {

        /// Gradient X
        cv::Mat grad_x;
        cv::Mat abs_grad_x;

        cv::Sobel( mcost, grad_x, ddepth, 1, 0, kernel_size, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );

        return grad_x;

    } else if (direction == 1) {

        /// Gradient Y
        cv::Mat grad_y;
        cv::Mat abs_grad_y;

        cv::Sobel( mcost, grad_y, ddepth, 0, 1, kernel_size, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );

        return grad_y;
    } 
    else 
    { // the original costmap just for debugging
        return mcost;
    }
}


void writeMatToFile(cv::Mat& m, const char* filename)
{
    using namespace std;

    ofstream fout(filename);

    if(!fout) {
        cout<<"File Not Opened"<<endl;  return;
    }

    for(unsigned int i=0; i<m.rows; i++) {
        for(unsigned int j=0; j<m.cols; j++) {
            fout<<m.at<float>(i,j)<<"\t";
        }
        fout<<endl;
    }

    fout.close();
}


Tpoint getLowestCostCell(Ped::Tagent* robot, cv::Mat cmap, Grid* grid)
{
    // get neighbor with the lowest cost;
    Tpoint ec = getEnclosingCell(grid, robot->getx(), robot->gety());
    Tpoint rp(ec.x*CONFIG.width, ec.y*CONFIG.height, 0);

    Tpoint lcc(rp.x-CONFIG.width, rp.y+CONFIG.height, 0);
    double lc = cmap.at<float>(ec.x-1, ec.y+1);

    if ( (withinGrid(rp.x, rp.y+CONFIG.height, grid)) && (cmap.at<float>(ec.x, ec.y+1) < lc) ) {
        lcc = Tpoint(rp.x, rp.y+CONFIG.height, 0);
        lc = cmap.at<float>(ec.x, ec.y+1);
    }

    if ( (withinGrid(rp.x+CONFIG.width, rp.y+CONFIG.height, grid)) && ( cmap.at<float>(ec.x+1, ec.y+1) < lc) ) {
         lcc = Tpoint(rp.x+CONFIG.width, rp.y+CONFIG.height, 0);
         lc = cmap.at<float>(ec.x+1, ec.y+1);
    }

    if ( (withinGrid(rp.x-CONFIG.width, rp.y, grid)) && ( cmap.at<float>(ec.x-1, ec.y) < lc) ) {
        lcc = Tpoint(rp.x-CONFIG.width, rp.y, 0);
        lc = cmap.at<float>(ec.x-1, ec.y);
    }

    if ( (withinGrid(rp.x+CONFIG.width, rp.y, grid)) && (cmap.at<float>(ec.x+1, ec.y) < lc) ) {
        lcc = Tpoint(rp.x+CONFIG.width, rp.y, 0);
        lc = cmap.at<float>(ec.x+1, ec.y);
    }

    if ( (withinGrid(rp.x-CONFIG.width, rp.y-CONFIG.height, grid)) && (cmap.at<float>(ec.x-1, ec.y-1) < lc) ) {
        lcc = Tpoint(rp.x-CONFIG.width, rp.y-CONFIG.height, 0);
        lc = cmap.at<float>(ec.x-1, ec.y-1);
    }

    if ( (withinGrid(rp.x, rp.y-CONFIG.height, grid)) && (cmap.at<float>(ec.x, ec.y-1) < lc) ) {
        lcc = Tpoint(rp.x, rp.y-CONFIG.height, 0);
        lc = cmap.at<float>(ec.x, ec.y-1);
    }

    if ( (withinGrid(rp.x+CONFIG.width, rp.y-CONFIG.height, grid)) && (cmap.at<float>(ec.x+1, ec.y-1) < lc) ) {
        lcc = Tpoint(rp.x+CONFIG.width, rp.y-CONFIG.height, 0);
        lc = cmap.at<float>(ec.x+1, ec.y-1);
    }

    return lcc;
}

Tpoint getLowestCostNeighbor(cv::Mat cmap, Tpoint cc, Grid* grid)
{
    // cycle over 8 neighbors are pick the one with the lowest cost
    std::vector<Tpoint> neighbors  = getNeighbors(Tpoint(cc.x*CONFIG.width, cc.y*CONFIG.height, 0), grid);
    Tpoint lcn(neighbors[0].x, neighbors[0].x, 0);
    double lc = cmap.at<float>(lcn.x/CONFIG.width, lcn.y/CONFIG.height);

    foreach(Tpoint n, neighbors) {
        double nc = cmap.at<float>(n.x/CONFIG.width, n.y/CONFIG.height);
        if (nc < lc) {
            lcn = n;
        }
    }

    return lcn;
}

