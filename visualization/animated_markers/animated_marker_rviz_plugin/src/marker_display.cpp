/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

#include <sstream>

#include <tf/transform_listener.h>

#include "markers/mesh_resource_marker.h"

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"

#include "marker_display.h"

using namespace rviz;

namespace animated_marker_rviz_plugin
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerDisplay::MarkerDisplay()
  : Display()
{
  marker_topic_property_ = new RosTopicProperty( "Marker Topic", "animated_marker",
                                                 QString::fromStdString( ros::message_traits::datatype<animated_marker_msgs::AnimatedMarker>() ),
                                                 "animated_marker_msgs::AnimatedMarker topic to subscribe to.  <topic>_array will also"
                                                 " automatically be subscribed with type animated_marker_msgs::AnimatedMarkerArray.",
                                                 this, SLOT( updateTopic() ));

  queue_size_property_ = new IntProperty( "Queue Size", 100,
                                          "Advanced: set the size of the incoming Marker message queue.  Increasing this is"
                                          " useful if your incoming TF data is delayed significantly from your Marker data, "
                                          "but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 0 );

  namespaces_category_ = new Property( "Namespaces", QVariant(), "", this );
}

void MarkerDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<animated_marker_msgs::AnimatedMarker>( *context_->getTFClient(),
                                                                  fixed_frame_.toStdString(),
                                                                  queue_size_property_->getInt(),
                                                                  update_nh_ );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));
  tf_filter_->registerFailureCallback(boost::bind(&MarkerDisplay::failedMarker, this, _1, _2));
}

MarkerDisplay::~MarkerDisplay()
{
  if ( initialized() )
  {
    unsubscribe();

    clearMarkers();

    delete tf_filter_;
  }
}

void MarkerDisplay::clearMarkers()
{
  markers_.clear();
  markers_with_expiration_.clear();
  frame_locked_markers_.clear();
  tf_filter_->clear();
  namespaces_category_->removeChildren();
  namespaces_.clear();
}

void MarkerDisplay::onEnable()
{
  subscribe();
}

void MarkerDisplay::onDisable()
{
  unsubscribe();
  tf_filter_->clear();

  clearMarkers();
}

void MarkerDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void MarkerDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void MarkerDisplay::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  std::string marker_topic = marker_topic_property_->getTopicStd();
  if( !marker_topic.empty() )
  {
    array_sub_.shutdown();
    sub_.unsubscribe();

    try
    {
      sub_.subscribe( update_nh_, marker_topic, queue_size_property_->getInt() );
      array_sub_ = update_nh_.subscribe( marker_topic + "_array", queue_size_property_->getInt(), &MarkerDisplay::incomingMarkerArray, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what() );
    }
  }
}

void MarkerDisplay::unsubscribe()
{
  sub_.unsubscribe();
  array_sub_.shutdown();
}

void MarkerDisplay::deleteMarker(MarkerID id)
{
  deleteMarkerStatus( id );

  M_IDToMarker::iterator it = markers_.find( id );
  if( it != markers_.end() )
  {
    markers_with_expiration_.erase(it->second);
    frame_locked_markers_.erase(it->second);
    markers_.erase(it);
  }
}

void MarkerDisplay::deleteMarkersInNamespace( const std::string& ns )
{
  std::vector<MarkerID> to_delete;

  // TODO: this is inefficient, should store every in-use id per namespace and lookup by that
  M_IDToMarker::iterator marker_it = markers_.begin();
  M_IDToMarker::iterator marker_end = markers_.end();
  for (; marker_it != marker_end; ++marker_it)
  {
    if (marker_it->first.first == ns)
    {
      to_delete.push_back(marker_it->first);
    }
  }

  {
    std::vector<MarkerID>::iterator it = to_delete.begin();
    std::vector<MarkerID>::iterator end = to_delete.end();
    for (; it != end; ++it)
    {
      deleteMarker(*it);
    }
  }
}

void MarkerDisplay::setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text)
{
  std::stringstream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  setStatusStd(level, marker_name, text);
}

void MarkerDisplay::deleteMarkerStatus(MarkerID id)
{
  std::stringstream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  deleteStatusStd(marker_name);
}

void MarkerDisplay::incomingMarkerArray(const animated_marker_msgs::AnimatedMarkerArray::ConstPtr& array)
{
  std::vector<animated_marker_msgs::AnimatedMarker>::const_iterator it = array->markers.begin();
  std::vector<animated_marker_msgs::AnimatedMarker>::const_iterator end = array->markers.end();
  for (; it != end; ++it)
  {
    const animated_marker_msgs::AnimatedMarker& marker = *it;
    tf_filter_->add(animated_marker_msgs::AnimatedMarker::Ptr(new animated_marker_msgs::AnimatedMarker(marker)));
  }
}

void MarkerDisplay::incomingMarker( const animated_marker_msgs::AnimatedMarker::ConstPtr& marker )
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  message_queue_.push_back(marker);
}

void MarkerDisplay::failedMarker(const ros::MessageEvent<animated_marker_msgs::AnimatedMarker>& marker_evt, tf::FilterFailureReason reason)
{
  animated_marker_msgs::AnimatedMarker::ConstPtr marker = marker_evt.getConstMessage();
  std::string authority = marker_evt.getPublisherName();
  std::string error = context_->getFrameManager()->discoverFailureReason(marker->header.frame_id, marker->header.stamp, authority, reason);
  setMarkerStatus(MarkerID(marker->ns, marker->id), StatusProperty::Error, error);
}

bool validateFloats(const animated_marker_msgs::AnimatedMarker& msg)
{
  bool valid = true;
  valid = valid && rviz::validateFloats(msg.pose);
  valid = valid && rviz::validateFloats(msg.scale);
  valid = valid && rviz::validateFloats(msg.color);
  return valid;
}

void MarkerDisplay::processMessage( const animated_marker_msgs::AnimatedMarker::ConstPtr& message )
{
  if (!validateFloats(*message))
  {
    setMarkerStatus(MarkerID(message->ns, message->id), StatusProperty::Error, "Contains invalid floating point values (nans or infs)");
    return;
  }

  switch ( message->action )
  {
  case animated_marker_msgs::AnimatedMarker::ADD:
    processAdd( message );
    break;

  case animated_marker_msgs::AnimatedMarker::DELETE:
    processDelete( message );
    break;

  default:
    ROS_ERROR( "Unknown marker action: %d\n", message->action );
  }
}

void MarkerDisplay::processAdd( const animated_marker_msgs::AnimatedMarker::ConstPtr& message )
{
  QString namespace_name = QString::fromStdString( message->ns );
  M_Namespace::iterator ns_it = namespaces_.find( namespace_name );
  if( ns_it == namespaces_.end() )
  {
    ns_it = namespaces_.insert( namespace_name, new MarkerNamespace( namespace_name, namespaces_category_, this ));
  }

  if( !ns_it.value()->isEnabled() )
  {
    return;
  }

  deleteMarkerStatus( MarkerID( message->ns, message->id ));

  bool create = true;
  MarkerBasePtr marker;

  M_IDToMarker::iterator it = markers_.find( MarkerID(message->ns, message->id) );
  if ( it != markers_.end() )
  {
    marker = it->second;
    markers_with_expiration_.erase(marker);
    if ( message->type == marker->getMessage()->type )
    {
      create = false;
    }
    else
    {
      markers_.erase( it );
    }
  }

  if ( create )
  {
    switch ( message->type )
    {
    case animated_marker_msgs::AnimatedMarker::MESH_RESOURCE:
      {
        marker.reset(new MeshResourceMarker(this, context_, scene_node_));
      }
      break;

    default:
      ROS_ERROR( "Unknown marker type: %d", message->type );
    }

    markers_.insert(std::make_pair(MarkerID(message->ns, message->id), marker));
  }

  if (marker)
  {
    marker->setMessage(message);

    if (message->lifetime.toSec() > 0.0001f)
    {
      markers_with_expiration_.insert(marker);
    }

    if (message->frame_locked)
    {
      frame_locked_markers_.insert(marker);
    }

    context_->queueRender();
  }
}

void MarkerDisplay::processDelete( const animated_marker_msgs::AnimatedMarker::ConstPtr& message )
{
  deleteMarker(MarkerID(message->ns, message->id));
  context_->queueRender();
}

void MarkerDisplay::update(float wall_dt, float ros_dt)
{
  V_MarkerMessage local_queue;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    local_queue.swap( message_queue_ );
  }

  if ( !local_queue.empty() )
  {
    V_MarkerMessage::iterator message_it = local_queue.begin();
    V_MarkerMessage::iterator message_end = local_queue.end();
    for ( ; message_it != message_end; ++message_it )
    {
      animated_marker_msgs::AnimatedMarker::ConstPtr& marker = *message_it;

      processMessage( marker );
    }
  }

  {
    S_MarkerBase::iterator it = markers_with_expiration_.begin();
    S_MarkerBase::iterator end = markers_with_expiration_.end();
    for (; it != end;)
    {
      MarkerBasePtr marker = *it;
      if (marker->expired())
      {
        ++it;
        deleteMarker(marker->getID());
      }
      else
      {
        ++it;
      }
    }
  }

  {
    S_MarkerBase::iterator it = frame_locked_markers_.begin();
    S_MarkerBase::iterator end = frame_locked_markers_.end();
    for (; it != end; ++it)
    {
      MarkerBasePtr marker = *it;
      marker->updateFrameLocked();
    }
  }

  // Update animation states
  {
    for(M_IDToMarker::iterator it = markers_.begin(); it != markers_.end(); ++it) {
      it->second->update(ros_dt);
    }
  }
}

void MarkerDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );

  clearMarkers();
}

void MarkerDisplay::reset()
{
  Display::reset();
  clearMarkers();
}

/////////////////////////////////////////////////////////////////////////////////
// MarkerNamespace

MarkerNamespace::MarkerNamespace( const QString& name, Property* parent_property, MarkerDisplay* owner )
  : BoolProperty( name, true,
                  "Enable/disable all markers in this namespace.",
                  parent_property )
  , owner_( owner )
{
  // Can't do this connect in chained constructor above because at
  // that point it doesn't really know that "this" is a
  // MarkerNamespace*, so the signal doesn't get connected.
  connect( this, SIGNAL( changed() ), this, SLOT( onEnableChanged() ));
}

void MarkerNamespace::onEnableChanged()
{
  if( !isEnabled() )
  {
    owner_->deleteMarkersInNamespace( getName().toStdString() );
  }
}

} // namespace animated_marker_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( animated_marker_rviz_plugin::MarkerDisplay, rviz::Display )
