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

#ifndef FPS_MOTION_VIEW_CONTROLLER
#define FPS_MOTION_VIEW_CONTROLLER

//#include </opt/ros/humble/opt/rviz_ogre_vendor/include/OGRE/OgreVector3.h>
//#include </opt/ros/humble/opt/rviz_ogre_vendor/include/OGRE/OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

//#include </opt/ros/humble/include/rviz_common/rviz_common/frame_position_tracking_view_controller.hpp>
#include <rviz_common/frame_position_tracking_view_controller.hpp>
//#include "rviz/frame_position_tracking_view_controller.h"

//using namespace rviz;
using namespace rviz_common;

namespace rviz_common
{
using rviz_common::properties::FloatProperty;
using rviz_common::properties::BoolProperty;
class SceneNode;
class Shape;
using rviz_common::properties::VectorProperty;
using rviz_common::properties::EnumProperty;

/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class RVIZ_COMMON_PUBLIC FPSMotionViewController : public FramePositionTrackingViewController
{
Q_OBJECT
public:
  FPSMotionViewController();
  virtual ~FPSMotionViewController();

  virtual void onInitialize();

  void yaw( float angle );
  void pitch( float angle );
  void move( float x, float y, float z );
  void fly( float x, float y, float z );
  void changeZ(float z );

  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  virtual void lookAt( const Ogre::Vector3& point );

  virtual void reset();

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic( ViewController* source_view );

  virtual void update(float dt, float ros_dt);

  void setCamera( Ogre::Camera* source_camera ) { setPropertiesFromCamera(source_camera );};

protected:
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  void updateCamera();
  void updateCamera(Ogre::Vector3& position, Ogre::Quaternion& orientation);
  Ogre::Quaternion getOrientation(); ///< Return a Quaternion based on the yaw and pitch properties.

  rviz_common::properties::FloatProperty* yaw_property_;                         ///< The camera's yaw (rotation around the y-axis), in radians
  rviz_common::properties::FloatProperty* pitch_property_;                       ///< The camera's pitch (rotation around the x-axis), in radians
  rviz_common::properties::VectorProperty* position_property_;
  rviz_common::properties::BoolProperty* fly_property_;
};

} // end namespace rviz

#endif // FPS_MOTION_VIEW_CONTROLLER
