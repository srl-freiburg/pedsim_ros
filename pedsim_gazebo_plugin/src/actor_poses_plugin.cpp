/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/AgentStates.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/SpawnModel.h>


namespace gazebo
{
    class ActorPosesPlugin : public WorldPlugin{
        public:
            ActorPosesPlugin() : WorldPlugin(), spawn_models_(false) {
                world_offset_.transform.rotation.w = 1.0;
            }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            this->world_ = _world;
            if (!ros::isInitialized()) {
                ROS_ERROR("ROS not initialized");
                return;
            }

            if (_sdf->HasElement("world_offset")) {
                auto pose = _sdf->Get<ignition::math::Pose3d>("world_offset");
                world_offset_.transform.translation.x = pose.Pos().X();
                world_offset_.transform.translation.y = pose.Pos().Y();
                world_offset_.transform.translation.z = pose.Pos().Z();
                world_offset_.transform.rotation.x = pose.Rot().X();
                world_offset_.transform.rotation.y = pose.Rot().Y();
                world_offset_.transform.rotation.z = pose.Rot().Z();
                world_offset_.transform.rotation.w = pose.Rot().W();
            }

            rosNode.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
                "/pedsim_simulator/simulated_agents", 1, boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &rosQueue);
            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));

            if (_sdf->HasElement("spawn_models")) {
                spawn_models_ = _sdf->Get<bool>("spawn_models");
                if (spawn_models_) {
                    gazebo_spawn_ = rosNode->serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
                    // using ROS parameter server for conveniency of resolving relative path to ROS packages
                    rosNode->param<std::string>("/pedsim_simulator/actor_model_file", actor_model_file_, "");
                }
            }
            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate, this));
        }

        // call back function when receive rosmsg
        void OnRosMsg( const pedsim_msgs::AgentStatesConstPtr msg) {
            if (spawn_models_) {
                if (!msg->agent_states.empty()) {
                    SpawnModels(actor_model_file_, msg);
                    spawn_models_ = false;
                }
                return;
            }

            std::string model_name;
            geometry_msgs::Pose tmp_pose;
#if GAZEBO_MAJOR_VERSION < 9
            for (unsigned int mdl = 0; mdl < world_->GetModelCount(); mdl++) {
#else
            for (unsigned int mdl = 0; mdl < world_->ModelCount(); mdl++) {
#endif
                physics::ModelPtr  tmp_model;
#if GAZEBO_MAJOR_VERSION < 9
                tmp_model = world_->GetModel(mdl);
#else
                tmp_model = world_->ModelByIndex(mdl);
#endif
                std::string frame_id;
                frame_id = tmp_model->GetName();

                for (uint actor =0; actor< msg->agent_states.size() ; actor++) {
                    if (frame_id == std::to_string( msg->agent_states[actor].id)  ){
                        tf2::doTransform(msg->agent_states[actor].pose, tmp_pose, world_offset_);

                        ignition::math::Pose3d gzb_pose;
                        gzb_pose.Pos().Set(tmp_pose.position.x,
                                           tmp_pose.position.y,
                                           tmp_pose.position.z + MODEL_OFFSET);
                        gzb_pose.Rot().Set(tmp_pose.orientation.w,
                                           tmp_pose.orientation.x,
                                           tmp_pose.orientation.y,
                                           tmp_pose.orientation.z);

                        try {
                            tmp_model->SetWorldPose(gzb_pose);
                        } catch(gazebo::common::Exception gz_ex) {
                            ROS_ERROR("Error setting pose %s - %s", frame_id.c_str(), gz_ex.GetErrorStr().c_str());
                        }

                    }
                }
            }
        }

        // ROS helper function that processes messages
        private: void QueueThread() {
            static const double timeout = 0.1;
            while (rosNode->ok()) {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        void SpawnModels(const std::string& filename, const pedsim_msgs::AgentStatesConstPtr actors) {
            gazebo_msgs::SpawnModel model;
            std::ifstream file(filename);
            if (file.fail()) {
                gzerr << "Could not open " << filename << std::endl;
                return;
            }

            std::string line;
            while (!file.eof()) {
                std::getline(file, line);
                model.request.model_xml += line;
            }
            file.close();

            geometry_msgs::Pose tmp_pose;
            for (auto& actor : actors->agent_states) {
                model.request.model_name = std::to_string(actor.id);
                model.request.reference_frame = "world";
                tf2::doTransform(actor.pose, tmp_pose, world_offset_);
                model.request.initial_pose = tmp_pose;
                gazebo_spawn_.call(model);
            }
        }

    private:
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        physics::WorldPtr world_;
        event::ConnectionPtr updateConnection_;
        const float MODEL_OFFSET = 0.75;
        geometry_msgs::TransformStamped world_offset_;
        ros::ServiceClient gazebo_spawn_;
        std::string actor_model_file_;
        bool spawn_models_;
    };
    GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}


