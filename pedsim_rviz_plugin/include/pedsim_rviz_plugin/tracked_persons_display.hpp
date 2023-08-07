/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TRACKED_PERSONS_DISPLAY_H
#define TRACKED_PERSONS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <pedsim_msgs/msg/tracked_persons.hpp>
#include <pedsim_rviz_plugin/person_display_common.hpp>
#endif

namespace pedsim_rviz_plugin
{
    typedef unsigned int track_id;

    /// A single entry in the history of a tracked person.
    struct TrackedPersonHistoryEntry
    {
        Ogre::Vector3 position;
        std::shared_ptr<rviz_rendering::Shape> shape;
        bool wasOccluded;
        track_id trackId;
    };

    /// History of a tracked person.
    typedef circular_buffer<std::shared_ptr<TrackedPersonHistoryEntry> > TrackedPersonHistory;

    /// The visual of a tracked person.
    struct TrackedPersonVisual
    {
        TrackedPersonHistory history;
        std::shared_ptr<rviz_rendering::BillboardLine> historyLine;
        Ogre::Vector3 positionOfLastHistoryEntry;

        std::shared_ptr<Ogre::SceneNode> sceneNode, historySceneNode, historyLineSceneNode;

        std::shared_ptr<PersonVisual> personVisual;
        std::shared_ptr<TextNode> idText, detectionIdText, stateText;
        std::shared_ptr<rviz_rendering::Arrow> velocityArrow;
        std::shared_ptr<CovarianceVisual> covarianceVisual;

        Ogre::Matrix4 lastObservedPose;

        bool isOccluded, isDeleted, isMissed, hasZeroVelocity;
        int numCyclesNotSeen;
    };

    // The TrackedPersonsDisplay class itself just implements a circular buffer,
    // editable parameters, and Display subclass machinery.
    class TrackedPersonsDisplay: public PersonDisplayCommon<pedsim_msgs::msg::TrackedPersons>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        TrackedPersonsDisplay() {};
        virtual ~TrackedPersonsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.

        // Called after the constructors have run
        virtual void onInitialize();

        // Called periodically by the visualization manager
        virtual void update(float wall_dt, float ros_dt);

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz_common::DisplayContext* getContext() {
            return context_;
        }

    private Q_SLOTS:
        void personVisualTypeChanged();

        // Called whenever one of the properties in PersonDisplayCommonProperties has been changed
        virtual void stylesChanged();

    private:
        // Function to handle an incoming ROS message.
        void processMessage(pedsim_msgs::msg::TrackedPersons::ConstSharedPtr msg) override;
       
        // All currently active tracks, with unique track ID as map key
        typedef map<track_id, std::shared_ptr<TrackedPersonVisual> > track_map;
        track_map m_cachedTracks;

        // Scene node for track history visualization
        std::shared_ptr<Ogre::SceneNode> m_trackHistorySceneNode;

        // User-editable property variables.
        rviz_common::properties::FloatProperty*  m_occlusion_alpha_property;
        rviz_common::properties::FloatProperty*  m_missed_alpha_property;
        rviz_common::properties::StringProperty* m_tracking_frame_property;
        rviz_common::properties::IntProperty*    m_history_length_property;
        rviz_common::properties::IntProperty*    m_delete_after_ncycles_property;

        rviz_common::properties::BoolProperty* m_show_deleted_property;
        rviz_common::properties::BoolProperty* m_show_occluded_property;
        rviz_common::properties::BoolProperty* m_show_missed_property;
        rviz_common::properties::BoolProperty* m_show_matched_property;
    
        rviz_common::properties::BoolProperty* m_render_person_property;
        rviz_common::properties::BoolProperty* m_render_history_property;
        rviz_common::properties::BoolProperty* m_render_history_as_line_property;
        rviz_common::properties::BoolProperty* m_render_covariances_property;
        rviz_common::properties::BoolProperty* m_render_state_prediction_property;
        rviz_common::properties::BoolProperty* m_render_velocities_property;
        rviz_common::properties::BoolProperty* m_render_ids_property;
        rviz_common::properties::BoolProperty* m_render_detection_ids_property;
        rviz_common::properties::BoolProperty* m_render_track_state_property;

        rviz_common::properties::FloatProperty* m_history_line_width_property;
        rviz_common::properties::FloatProperty* m_history_min_point_distance_property;
        rviz_common::properties::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace pedsim_rviz_plugin

#endif // TRACKED_PERSONS_DISPLAY_H
