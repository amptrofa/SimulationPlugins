/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef __GAZEBO_DEPTH_CAMERA_PLUGIN_HH__
#define __GAZEBO_DEPTH_CAMERA_PLUGIN_HH__

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "boost/algorithm/string/replace.hpp"

namespace gazebo
{
  class DepthCameraPlugin : public SensorPlugin
  {
    public: DepthCameraPlugin();
    
    private: std::string GetTopicName() const;
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewDepthFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    public: virtual void OnNewRGBPointCloud(const float *_pcd,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    public: virtual void OnNewImageFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::DepthCameraSensorPtr parentSensor;
    protected: rendering::DepthCameraPtr depthCamera;
    protected: transport::NodePtr node;
    protected: transport::PublisherPtr depthMapPub;

    private: event::ConnectionPtr newDepthFrameConnection;
    private: event::ConnectionPtr newRGBPointCloudConnection;
    private: event::ConnectionPtr newImageFrameConnection;
  };
}
#endif
