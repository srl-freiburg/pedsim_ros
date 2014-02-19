#ifndef SCENE_H
#define SCENE_H

#include <cstdlib>

#include <ped_scene.h>
#include <ped_tree.h>

#include <QMap>
#include <QTimer>
#include <QKeyEvent>
#include <QFile>
#include <QXmlStreamReader>
#include <QRect>

#include "config.h"
#include "agent.h"
#include "obstacle.h"
#include "waypoint.h"

// #include <boost/signal.hpp>
// #include <tf/transform_broadcaster.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/shared_ptr.hpp>

// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

// messages and services
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AllAgentsState.h>

#include <pedsim_srvs/SetAgentState.h>
#include <pedsim_srvs/SetAllAgentsState.h>
#include <pedsim_srvs/GetAgentState.h>
#include <pedsim_srvs/GetAllAgentsState.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
// #include <geometry_msgs/Point32.h>
// #include <geometry_msgs/Polygon.h>
// #include <geometry_msgs/PolygonStamped.h>


class Scene : public Ped::Tscene
{
public:
    Scene(const ros::NodeHandle& node);
    Scene( double left, double up, double width, double height, const ros::NodeHandle& node );
    ~Scene() { clear(); }

    void clear();
    std::set<const Ped::Tagent*> getNeighbors(double x, double y, double maxDist);

    // overriding methods
    void addAgent(Ped::Tagent *a) { Ped::Tscene::addAgent(a); }
    void addObstacle(Ped::Tobstacle *o) { Ped::Tscene::addObstacle(o); }
    void cleanup() { Ped::Tscene::cleanup(); }
    void moveAgents(double h) { Ped::Tscene::moveAgents(h); }
    void spawnKillAgents();

    /// service handler for moving agents
    bool srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request&, pedsim_srvs::SetAgentState::Response& );

    /// publisher helpers
    void publishAgentStatus();
    void publishAgentVisuals();
    void publishObstacles();
    void publishSensorRange();

    /// subscriber helpers
    void callbackRobotState(const pedsim_msgs::AgentState::ConstPtr& msg);

    /// helpers related to parsing scene files
    inline bool readFromFile(const QString& filename);
    inline void processData(QByteArray& data);
    inline void drawObstacles(float x1, float y1, float x2, float y2);

    /// simulaition management
    bool initialize();
    void runSimulation();
    void moveAllAgents();
    void cleanupItems();

private:
    // robot and agents
    Ped::Tagent* robot_;
    std::vector<Ped::Tagent*> all_agents_;

    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_all_agents_;
    ros::Publisher pub_agent_visuals_;
    ros::Publisher pub_obstacles_;
    // ros::Publisher pub_sensor_range_;

    // subscribers
    ros::Subscriber sub_robot_state_;

    // service servers
    ros::ServiceServer srv_move_agent_;

    QXmlStreamReader xmlReader;
    QList<Agent*> currentAgents;
    QList<Obstacle*> obstacles;
    QMap<QString, Waypoint*> waypoints;
    size_t timestep_;

    // obstacle cell locations
    std::vector<TLoc> obstacle_cells_;


    inline double* angleToQuaternion(double theta);
};


/// helpful typedefs
typedef boost::shared_ptr<Scene> ScenePtr;
typedef boost::shared_ptr<Scene const> SceneConstPtr;


#endif // SCENE_ICRA14_H
