#ifndef SCENE_TEST_H
#define SCENE_TEST_H

#include <ped_scene.h>
#include <ped_tree.h>

#include <QMap>
#include <QTimer>
#include <QKeyEvent>
#include <QGraphicsScene>

#include <boost/bind.hpp>

#include <qwt/qwt_color_map.h>
#include <qwt/qwt_interval.h>

#include <logging.h>
#include <config.h>
#include <grid.h>
#include <agent.h>
#include <obstacle.h>
#include <waypoint.h>

#include <search/graph_search.h>
#include <simple_utils.h>
#include <features/features_icra.h>
#include <trajectory.h>
#include <3rdparty/yagsbpl/planners/A_star.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <clustering/clustering.h>

#include <features/costmap.h>


// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

// messages and services
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AllAgentsState.h>

#include <pedsim_srvs/SetAgentState.h>
#include <pedsim_srvs/SetAllAgentsState.h>
#include <pedsim_srvs/GetAgentState.h>
#include <pedsim_srvs/GetAllAgentsState.h>


// #define FLIP


class QGraphicsScene;
class Agent;
class Grid;
class Obstacle;
class Waypoint;
class GNode;



class Scene : public QObject, public Ped::Tscene
{
    Q_OBJECT
public:
    Scene(QGraphicsScene* guiSceneIn, const ros::NodeHandle& node);
    virtual ~Scene();

    /// =====================================================
    /// Slots
    /// =====================================================
public slots:
    void updateWaypointVisibility(bool visible);

protected slots:
    void moveAllAgents();
    void cleanupSlot();
    void changeSpeeds();
    void teleopControl();
    void driveWithPolicy();
    void logExperimentData();


    /// =====================================================
    /// Methods
    /// =====================================================
public:
    bool isPaused() const;
    void pauseUpdates();
    void unpauseUpdates();
    void clear();
    static Grid* getGrid();
    void initializeAll();
    void visualizePath(std::vector<GridCell> path, QColor col);
    void updateFullCostMap();
    void updateLocalCostMap();
    void visualizeCostMap(bool skeleton=true);
    void checkGoalReached();
    void checkWaypointReached();
    void generateNewTrajectory(bool initial);
    void generateNewDijkstraCostMap();
    void driveSingleStep();
    void driveSingleStepGradient();
    void driveSingleStepSocialForce();
    void visualizeDCosts(std::map<int, double> dcosts, bool skeleton = true);
    void spawnKillAgents();

    // TODO - remove this
    std::set<const Ped::Tagent*> getNeighbors(double x, double y, double maxDist);
    
    void updateMetrics();

    void generatePath(std::vector<std::vector<GNode> >& paths, GNode st);
    void convertPathToTrajectory(std::vector<GNode>& path, bool replan);
    void visualizePath2();


    // smoke tests
    std::array<float, 5> smokeFeature();
    float computeSmokeCost(const std::array<float, 5> f);
    void simpleTestCostMap();
    float getClosestDistance(Tpoint p);
    void smokeFS();


    /// =====================================================
    /// Attributes
    /// =====================================================
public:
    QGraphicsScene* guiScene;
    QList<Agent*> agents;
    QList<Obstacle*> obstacles;
    QMap<QString, Waypoint*> waypoints;
    static Grid* grid_;
    size_t timestep;

private:
    QTimer movetimer;
    QTimer cleanuptimer;
    QTimer speed_timer;
    QTimer teleop_timer;
    QTimer robot_timer;
    QTimer log_timer;

    // robot and agents
    Ped::Tagent* robot_;
    std::vector<Ped::Tagent*> all_agents_;

    // sampler for generating random velocities
    Sampler *sampler_;

    // policy (learned weights)
    PolicyICRA icra_policy_;
    // PolicyKIM kim_policy_;

    // feature extraction
    CFeatures_ICRA *ficra_;

    // start, goal, waypoint control, trajectory
    Tpoint start_, goal_, next_wp_, old_wp_, pwp;
    int traj_step_;
    Trajectory *trajectory_;

    // dijkstra related
    std::map<int, double> dcosts_;
    Tpoint lcc_;

    // graph search module
    GraphSearchPtr graph_search_;

    // Metrics
    Anisotropic anisotropics_;
    Proxemics proxemics_;
    double closest_distance_;


    // path visuals
    std::vector<QGraphicsItem*> path_elements_;


    // TODO - select only one search 
    GenericSearchGraphDescriptor<GNode,double> *sgraph_;


    /// service handler for moving agents
    bool srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request&, pedsim_srvs::SetAgentState::Response& );


private:

    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_all_agents_;

    void publicAgentStatus();

    ros::ServiceServer srv_move_agent_;

};





// graph search
class GNode
{
public:
    double x, y;
    bool occupied;
    double theta;
    Grid* gd;

    GNode()
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        gd = Scene::grid_;
    }

    GNode(double xx, double yy)
        : x(xx), y(yy)
    {
        theta = 0.0;
        gd = Scene::grid_;
    }

    GNode(double xx, double yy, double th)
        : x(xx), y(yy), theta(th)
    {
        gd = Scene::grid_;
    }

    GNode(double xx, double yy, Grid* g)
        : x(xx), y(yy)
    {
        theta = 0.0;
        gd = g;
    }

    bool operator==(const GNode& n)
    {
        return (x == n.x && y == n.y && theta == n.theta);
    }

    GNode& operator=(GNode const& copy) 
    {
        x = copy.x;
        y = copy.y;
        theta = copy.theta;
        return  *this;
    }


    // already takes care of diagonal vs straight cells
    double costTo(const GNode& n)
    {
        double xs = abs(n.x - x);
        double ys = abs(n.y - y);
        return sqrt(xs*xs + ys*ys);
    }

    // the cell enclosing the node
    Tpoint getCell()
    {
        if (((x-gd->minx) < 0) || ((y-gd->miny) < 0))
            std::cerr << "Cell coordinates out of grid!" << std::endl;

        unsigned int cellx = (x-gd->minx)/CONFIG.width;
        unsigned int celly = (y-gd->miny)/CONFIG.height;

        if ((cellx >= (unsigned int)gd->width || (celly >= (unsigned int)gd->height)))
            std::cerr << "Cell coordinates out of grid!" << std::endl;

        return Tpoint(cellx, celly, 0);
    }
};


// simple jet colomap for visualization
inline void generateColormMap(QwtLinearColorMap& colormap_)
{
    colormap_.setColorInterval(QColor(0,0,189), QColor(132,0,0));
    double pos;

    pos = 1.0 / 13.0 * 1.0; colormap_.addColorStop(pos, QColor(0,0,255));
    pos = 1.0 / 13.0 * 2.0; colormap_.addColorStop(pos, QColor(0,66,255));
    pos = 1.0 / 13.0 * 3.0; colormap_.addColorStop(pos, QColor(0,132,255));
    pos = 1.0 / 13.0 * 4.0; colormap_.addColorStop(pos, QColor(0,189,255));
    pos = 1.0 / 13.0 * 5.0; colormap_.addColorStop(pos, QColor(0,255,255));
    pos = 1.0 / 13.0 * 6.0; colormap_.addColorStop(pos, QColor(66,255,189));
    pos = 1.0 / 13.0 * 7.0; colormap_.addColorStop(pos, QColor(132,255,132));
    pos = 1.0 / 13.0 * 8.0; colormap_.addColorStop(pos, QColor(189,255,66));
    pos = 1.0 / 13.0 * 9.0; colormap_.addColorStop(pos, QColor(255,255,0));
    pos = 1.0 / 13.0 * 10.0; colormap_.addColorStop(pos, QColor(255,189,0));
    pos = 1.0 / 13.0 * 12.0; colormap_.addColorStop(pos, QColor(255,66,0));
    pos = 1.0 / 13.0 * 13.0; colormap_.addColorStop(pos, QColor(189,0,0));
}



/// graph search helpers
inline bool withinGrid(double cx, double cy)
{
    if (cx < Scene::grid_->minx || cy < Scene::grid_->miny)
        return false;

    if (cx >= (Scene::grid_->minx + Scene::grid_->width) || cy >= ( Scene::grid_->miny + Scene::grid_->height) )
        return false;

    return true;
}

inline int getHashBin(GNode& n)
{
    return ((int)fabs(n.x));
}

inline bool isAccessible(GNode& n)
{
    if (Scene::grid_->isOccupied(n.x, n.y) ) return false;
    else return true;
}

inline double getHeuristics(GNode& n1, GNode& n2)
{
    int dx = abs(n1.x - n2.x);
    int dy = abs(n1.y - n2.y);

    // double weight = (CONFIG.cost_map[std::make_pair(n1.x,n1.y)]);
    double weight = CONFIG.costmatrix->getValGrid(n1.x, n1.y, CONFIG.width, CONFIG.height);
    double scaled_weight = (weight - CONFIG.fmin) / ((CONFIG.fmax - CONFIG.fmin)+1e-50); // prevent division by zero

    double buf = eDist(CONFIG.width, CONFIG.height, 0, 0);

    //    return sqrt((double)(dx*dx + dy*dy));
    return sqrt((double)(dx*dx + dy*dy)) * ((100*scaled_weight) + buf);
}

inline void getSuccessors(GNode& nn, std::vector<GNode>* s, std::vector<double>* c)
{
    s->clear();
    c->clear();

    // cycle over 8 possible successors
    // TODO - move all these into one module
    Tpoint nc = nn.getCell();
    Tpoint n(nc.x*CONFIG.width, nc.y*CONFIG.height, 0);

    if (withinGrid(n.x, n.y+CONFIG.height)) {
        GNode tn(n.x, n.y+CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x-CONFIG.width,n.y+CONFIG.height)) {
        GNode tn(n.x-CONFIG.width,n.y+CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x+CONFIG.width,n.y+CONFIG.height)) {
        GNode tn(n.x+CONFIG.width,n.y+CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x-CONFIG.width,n.y)) {
        GNode tn(n.x-CONFIG.width,n.y);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x+CONFIG.width,n.y)) {
        GNode tn(n.x+CONFIG.width,n.y);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    //
    if (withinGrid(n.x,n.y-CONFIG.height)) {
        GNode tn(n.x,n.y-CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x-CONFIG.width,n.y-CONFIG.height)) {
        GNode tn(n.x-CONFIG.width,n.y-CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }

    if (withinGrid(n.x+CONFIG.width,n.y-CONFIG.height)) {
        GNode tn(n.x+CONFIG.width,n.y-CONFIG.height);
        if (!Scene::grid_->isOccupied(tn.x, tn.y) ) {
            s->push_back(tn);
            c->push_back(nn.costTo(tn));
        }
    }
}





#endif // SCENE_ICRA14_H
