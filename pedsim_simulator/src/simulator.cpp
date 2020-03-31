/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.hpp>
#include <pedsim_simulator/scene.hpp>
#include <pedsim_simulator/simulator.hpp>

#include <pedsim_utils/geometry.hpp>

using namespace pedsim;
using namespace pedsim_msgs::msg;
using std::placeholders::_1;
using std::placeholders::_2;

Simulator::Simulator(const std::string & name) : Node(name)
{
  //dynamic_reconfigure::Server<SimConfig>::CallbackType f;
  //f = boost::bind(&Simulator::reconfigureCB, this, _1, _2);
  //server_.setCallback(f);
}

Simulator::~Simulator() {
  // shutdown service servers
 
  //srv_pause_simulation_.shutdown();
  //srv_unpause_simulation_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation() {
  int queue_size = 0;
  std::string scene_file_param;
  int op_mode = 1;

  paused_ = false;
  robot_ = nullptr;

  declare_parameter("scene_file", rclcpp::ParameterValue(""));
  get_parameter("scene_file", scene_file_param);  
  if (scene_file_param == "") {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid scene file: " << scene_file_param);
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), 
    "Loading scene [" << scene_file_param << "] for simulation");
    
  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false)
  {
    RCLCPP_ERROR_STREAM(get_logger(), 
      "Could not load the scene file, please check the paths and param "
      "names : "
      << scene_file_param);
    return false;
  }
  // load additional parameters
  declare_parameter("enable_groups", rclcpp::ParameterValue(true));
  get_parameter("enable_groups", CONFIG.groups_enabled); 
  declare_parameter("max_robot_speed", rclcpp::ParameterValue(1.5));
  get_parameter("max_robot_speed", CONFIG.max_robot_speed); 
  declare_parameter("update_rate", rclcpp::ParameterValue(25.0));
  get_parameter("update_rate", CONFIG.updateRate); 
  declare_parameter("simulation_factor", rclcpp::ParameterValue(1.0));
  get_parameter("simulation_factor", CONFIG.simulationFactor); 
  declare_parameter("robot_mode", rclcpp::ParameterValue(1));
  get_parameter("robot_mode", op_mode);
  declare_parameter("default_queue_size", rclcpp::ParameterValue(1));
  get_parameter("default_queue_size", queue_size);
  declare_parameter("robot_radius", rclcpp::ParameterValue(0.35));
  get_parameter("robot_radius", robot_radius_);
  declare_parameter("agent_radius", rclcpp::ParameterValue(0.35));
  get_parameter("agent_radius", agent_radius_);
  declare_parameter("force_factor_social", rclcpp::ParameterValue(10.0));
  get_parameter("force_factor_social", force_factor_social_);

  CONFIG.robot_mode = static_cast<RobotMode>(op_mode);
  RCLCPP_INFO_STREAM(get_logger(), 
  "Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0
                          ? "NOTE: This means the queues are of infinite size!"
                          : ""));

  // setup ros2 publishers
  pub_obstacles_ = create_publisher<LineObstacles>("pedsim_simulator/simulated_walls", queue_size);
  pub_agent_states_ = create_publisher<AgentStates>("pedsim_simulator/simulated_agents", queue_size);
  pub_agent_groups_ = create_publisher<AgentGroups>("pedsim_simulator/simulated_groups", queue_size);
  pub_robot_position_ = create_publisher<nav_msgs::msg::Odometry>(
    "pedsim_simulator/robot_position", queue_size);
  
  // services
  srv_pause_simulation_ = create_service<std_srvs::srv::Empty>(
    "pedsim_simulator/pause_simulation", std::bind(&Simulator::onPauseSimulation, this, _1, _2));
  srv_unpause_simulation_ = create_service<std_srvs::srv::Empty>(
    "pedsim_simulator/unpause_simulation", std::bind(&Simulator::onUnpauseSimulation, this, _1, _2));
  // setup TF2 listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  spawn_timer_ = create_wall_timer(
    1000ms, std::bind(&Simulator::spawnCallback, this));
  return true;
}

void Simulator::runSimulation() 
{
  rclcpp::Rate r(CONFIG.updateRate);
  while (rclcpp::ok())
  {
    if (!robot_) 
    {
      // setup the robot
      for (Agent* agent : SCENE.getAgents()) 
      {
        agent->setForceFactorSocial(force_factor_social_);
        if (agent->getType() == Ped::Tagent::ROBOT)
        {
          robot_ = agent;
          agent->SetRadius(robot_radius_);
          last_robot_orientation_ =
              poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
        }
        else
          agent->SetRadius(agent_radius_);
      }
    }
    if (!paused_) 
    {
      updateRobotPositionFromTF();
      SCENE.moveAllAgents();
      publishAgents();
      publishGroups();
      publishRobotPosition();
      publishObstacles();  // TODO - no need to do this all the time.
    }

    rclcpp::spin_some(shared_from_this());
    r.sleep();
  }
}

/*
void Simulator::reconfigureCB(pedsim_simulator::PedsimSimulatorConfig& config,
                              uint32_t level) {
  CONFIG.updateRate = config.update_rate;
  CONFIG.simulationFactor = config.simulation_factor;

  // update force scaling factors
  CONFIG.setObstacleForce(config.force_obstacle);
  CONFIG.setObstacleSigma(config.sigma_obstacle);
  CONFIG.setSocialForce(config.force_social);
  CONFIG.setGroupGazeForce(config.force_group_gaze);
  CONFIG.setGroupCoherenceForce(config.force_group_coherence);
  CONFIG.setGroupRepulsionForce(config.force_group_repulsion);
  CONFIG.setRandomForce(config.force_random);
  CONFIG.setAlongWallForce(config.force_wall);

  // puase or unpause the simulation
  if (paused_ != config.paused) {
    paused_ = config.paused;
  }

  ROS_INFO_STREAM("Updated sim with live config: Rate=" << CONFIG.updateRate
                                                        << " incoming rate="
                                                        << config.update_rate);
}*/
void Simulator::onPauseSimulation(
  const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  paused_ = true;
}

void Simulator::onUnpauseSimulation(
  const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  paused_ = false;
}

void Simulator::spawnCallback()
{
  RCLCPP_DEBUG(get_logger(), "Spawning new agents.");

  for (const auto& sa : SCENE.getSpawnAreas()) {
    AgentCluster* agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));

    for (const auto& wp_name : sa->waypoints) 
    {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::updateRobotPositionFromTF() 
{
  if (!robot_) return;

  if (CONFIG.robot_mode == RobotMode::TELEOPERATION ||
      CONFIG.robot_mode == RobotMode::CONTROLLED) {
    robot_->setTeleop(true);
    robot_->setVmax(CONFIG.max_robot_speed);
    geometry_msgs::msg::TransformStamped tf_msg;
    try 
    {
      // Check if the transform is available
      tf_msg = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } 
    catch (tf2::TransformException &e) 
    {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return;
    }
    tf2::Transform tfTransform;
    tf2::impl::Converter<true, false>::convert(tf_msg.transform, tfTransform);
    
    const double x = tfTransform.getOrigin().x();
    const double y = tfTransform.getOrigin().y();
    const double dx = x - last_robot_pose_.transform.translation.x,
                 dy = y - last_robot_pose_.transform.translation.y;
    const double dt =
        tf_msg.header.stamp.sec - last_robot_pose_.header.stamp.sec;
    double vx = dx / dt, vy = dy / dt;

    if (!std::isfinite(vx)) vx = 0;
    if (!std::isfinite(vy)) vy = 0;

    robot_->setX(x);
    robot_->setY(y);
    robot_->setvx(vx);
    robot_->setvy(vy);

    last_robot_pose_ = tf_msg;
  }
}

void Simulator::publishRobotPosition() 
{
  if (robot_ == nullptr) return;

  nav_msgs::msg::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = "odom";

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  if (hypot(robot_->getvx(), robot_->getvy()) < 0.05) {
    robot_location.pose.pose.orientation = last_robot_orientation_;
  } else {
    robot_location.pose.pose.orientation =
        poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
    last_robot_orientation_ = robot_location.pose.pose.orientation;
  }

  robot_location.twist.twist.linear.x = robot_->getvx();
  robot_location.twist.twist.linear.y = robot_->getvy();

  pub_robot_position_->publish(robot_location);
}

void Simulator::publishAgents() 
{
  if (SCENE.getAgents().size() < 2) {
    return;
  }

  AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::msg::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents()) {
    AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER) {
      state.social_state = AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT) {
      continue;
    }

    // Forces.
    AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    all_status.agent_states.push_back(state);
  }

  pub_agent_states_->publish(all_status);
}

void Simulator::publishGroups() 
{
  if (!CONFIG.groups_enabled) {
    RCLCPP_DEBUG_STREAM(get_logger(),
      "Groups are disabled, no group data published: flag=" 
      << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1) {
    return;
  }

  AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto& ped_group : SCENE.getGroups()) {
    if (ped_group->memberCount() <= 1) continue;

    pedsim_msgs::msg::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto& member : ped_group->getMembers()) {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_->publish(sim_groups);
}

void Simulator::publishObstacles() 
{
  LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto& obstacle : SCENE.getObstacles()) {
    LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle->getax();
    line_obstacle.start.y = obstacle->getay();
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle->getbx();
    line_obstacle.end.y = obstacle->getby();
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_->publish(sim_obstacles);
}

std::string Simulator::agentStateToActivity(
    const AgentStateMachine::AgentState& state) const 
{
  std::string activity = "Unknown";
  switch (state) {
    case AgentStateMachine::AgentState::StateWalking:
      activity = AgentState::TYPE_INDIVIDUAL_MOVING;
      break;
    case AgentStateMachine::AgentState::StateGroupWalking:
      activity = AgentState::TYPE_GROUP_MOVING;
      break;
    case AgentStateMachine::AgentState::StateQueueing:
      activity = AgentState::TYPE_WAITING_IN_QUEUE;
      break;
    case AgentStateMachine::AgentState::StateShopping:
      break;
    case AgentStateMachine::AgentState::StateNone:
      break;
    case AgentStateMachine::AgentState::StateWaiting:
      break;
  }
  return activity;
}

std_msgs::msg::Header Simulator::createMsgHeader() const 
{
  std_msgs::msg::Header msg_header;
  msg_header.stamp = rclcpp::Clock().now();;
  msg_header.frame_id = "odom";
  return msg_header;
}
