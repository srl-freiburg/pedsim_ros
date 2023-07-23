#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <vector>

class AgentSpawner : public rclcpp::Node
{
public:
    AgentSpawner()
        : Node("agent_spawner")
    {
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
            "pedsim_simulator/simulated_agents", qos,
            std::bind(&AgentSpawner::actor_poses_callback, this, std::placeholders::_1));

        std::string pedsim_dir = ament_index_cpp::get_package_share_directory("pedsim_gazebo_plugin");
        std::ifstream file_xml(pedsim_dir + "/models/person_standing/model.sdf");
        if (file_xml.is_open()) {
        xml_string_ = std::string((std::istreambuf_iterator<char>(file_xml)),
                                std::istreambuf_iterator<char>());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to open file at path: %s", pedsim_dir.c_str());
        }


        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
        set_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo_spawner/set_entity_state");

        while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
        }

        is_spawned_.resize(max_ped_, false);
    }

private:
    void actor_poses_callback(const pedsim_msgs::msg::AgentStates::SharedPtr actors)
    {
        for (size_t idx = 0; idx < actors->agent_states.size(); ++idx)
        {
            auto& actor = actors->agent_states[idx];
            std::string actor_id = std::to_string(actor.id);

            if (!is_spawned_[idx])
            {
                auto spawn_future = spawn_entity(actor_id, actor.pose);
                futures_.push_back(spawn_future); // Save the future
                size_t future_idx = futures_.size() - 1; // Get the index of the saved future
    
                // Start a new thread to wait for the future
                std::thread([this, actor_id, model_pose=actor.pose, idx, future_idx]() {
                    auto& future = futures_[future_idx]; // Access the future by index
                    future.wait(); // Wait for the future to be ready
                    callback(actor_id, model_pose, idx, future_idx); // Call the callback
                }).detach();
            } else {
                set_entity_state(actor_id, actor.pose);
            }
        }
    }

    void callback(std::string actor_id, geometry_msgs::msg::Pose model_pose, int idx, size_t future_idx)
    {
        auto& future = futures_[future_idx]; // Access the future by index
        auto response = future.get();
        if (response && response->success) {
            is_spawned_[idx] = true;
            set_entity_state(actor_id, model_pose);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to spawn entity: %s", actor_id.c_str());
        }
    }

    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture spawn_entity(
        std::string actor_id, geometry_msgs::msg::Pose model_pose)
    {
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = actor_id;
        request->xml = xml_string_;
        request->robot_namespace = "";
        request->initial_pose = model_pose;
        request->reference_frame = "world";
        auto future = spawn_client_->async_send_request(request);
        return future;
    }

    void set_entity_state(std::string actor_id, geometry_msgs::msg::Pose model_pose)
    {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = actor_id;
        request->state.pose = model_pose;
        set_state_client_->async_send_request(request);
    }

    std::vector<rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture> futures_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr subscription_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;
    std::string xml_string_;
    std::vector<bool> is_spawned_;
    const int max_ped_ = 200;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentSpawner>());
    rclcpp::shutdown();
    return 0;
}