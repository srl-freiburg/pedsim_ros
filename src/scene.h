#ifndef SCENE_H
#define SCENE_H

#include <ped_scene.h>
#include <ped_tree.h>

#include <QMap>
#include <QTimer>
#include <QKeyEvent>
#include <QFile>
#include <QXmlStreamReader>
#include <QRect>


#include <boost/bind.hpp>

#include <config.h>
#include <grid.h>
#include <agent.h>
#include <obstacle.h>
#include <waypoint.h>
// #include <scenarioreader.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>


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

#include <visualization_msgs/Marker.h>


// class Agent;
// class Grid;
// class Obstacle;
// class Waypoint;


class Scene : public Ped::Tscene
{
public:
    Scene(const ros::NodeHandle& node);
    ~Scene() { clear(); }

    void clear();
    Grid* getGrid() { return grid_; }
    std::set<const Ped::Tagent*> getNeighbors(double x, double y, double maxDist);

    // overriding methods
    void addAgent(Ped::Tagent *a) { Ped::Tscene::addAgent(a); }
    void addObstacle(Ped::Tobstacle *o) { Ped::Tscene::addObstacle(o); }
    void cleanup() { Ped::Tscene::cleanup(); }
    void moveAgents(double h) { Ped::Tscene::moveAgents(h); }

    /// service handler for moving agents
    bool srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request&, pedsim_srvs::SetAgentState::Response& );
    void publicAgentStatus();
    void publishAgentVisuals();

    void moveAllAgents();
    void cleanupSlot();

    inline bool readFromFile(const QString& filename);
    inline void processData(QByteArray& data);

    void runSimulation();

    QList<Agent*> agents;
    QList<Obstacle*> obstacles;
    QMap<QString, Waypoint*> waypoints;
    Grid* grid_;
    size_t timestep;


private:
    // robot and agents
    Ped::Tagent* robot_;
    std::vector<Ped::Tagent*> all_agents_;

    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_all_agents_;
    ros::Publisher pub_agent_visuals_;

    // service servers
    ros::ServiceServer srv_move_agent_;

    QXmlStreamReader xmlReader;
    QList<Agent*> currentAgents;


    double eDist(double x1, double y1, double x2, double y2)
    {
        double dx = (x2-x1) * (1/20.0);
        double dy = (y2-y1) * (1/20.0);

        return sqrt( pow(dx, 2.0) + pow(dy, 2.0) );    // distance in metres
    }
};



/// helpful typedefs
typedef boost::shared_ptr<Scene> ScenePtr;
typedef boost::shared_ptr<Scene const> SceneConstPtr;




#endif // SCENE_ICRA14_H
