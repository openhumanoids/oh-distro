/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: John Hsu */

/* example

 <sensor name="my_camera_sensor" type="camera" update_rate="20">
   <origin xyz="0 0 0" rpy="0 0 0"/>
   <camera>
     <horizontal_hov>1.5708</horizontal_hov>
     <image width="640" height="480" format="R8G8B8"/>
     <clip near="0.01" far="50.0"/>
   </camera>
 </sensor>
 <sensor name="my_ray_sensor" type="ray" update_rate="20">
   <origin xyz="0 0 0" rpy="0 0 0"/>
   <ray>
     <scan>
       <horizontal samples="100" resolution="1" min_angle="-1.5708" max_angle="1.5708"/>
       <vertical samples="1" resolution="1" min_angle="0" max_angle="0"/>
     </scan>
   </ray>
 </sensor>

*/



#ifndef URDF_SENSOR_H
#define URDF_SENSOR_H

#include <string>
#include <vector>
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace urdf{

class VisualSensor
{
public:
  enum {CAMERA, RAY} type;
  virtual void initXml(TiXmlElement *) = 0;
};

class Camera : public VisualSensor
{
public:
  Camera() { this->clear(); };
  double horizontal_fov;
  double width, height;
  /// format is optional: defaults to R8G8B8), but can be
  /// (L8|R8G8B8|B8G8R8|BAYER_RGGB8|BAYER_BGGR8|BAYER_GBRG8|BAYER_GRBG8)
  std::string format;

  double near;
  double far;

  void clear()
  {
    horizontal_fov = 0;
    width = 0;
    height = 0;
    format.clear();
    near = 0;
    far = 0;
  };
  void initXml(TiXmlElement *);
};

class Ray : public VisualSensor
{
public:
  Ray() { this->clear(); };
  double horizontal_samples;
  double horitontal_resolution;
  double horitontal_min_angle;
  double horitontal_max_angle;
  double vertical_samples;
  double vertical_resolution;
  double vertical_min_angle;
  double vertical_max_angle;

  void clear()
  {
    horizontal_samples = 0;
    horitontal_resolution = 0;
    horitontal_min_angle = 0;
    horitontal_max_angle = 0;
    vertical_samples = 0;
    vertical_resolution = 0;
    vertical_min_angle = 0;
    vertical_max_angle = 0;
  };
  void initXml(TiXmlElement *);
};


class Sensor
{
public:
  Sensor() { this->clear(); };

  /// sensor name must be unique
  std::string name;

  /// type can be "camera" or "ray"
  std::string type;

  /// update rate in Hz
  double update_rate;

  /// transform from parent frame to optical center
  ///   with z-forward and x-right, y-down
  Pose origin;

  /// sensor type
  boost::shared_ptr<VisualSensor> sensor;


  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  boost::shared_ptr<Joint> parent_joint;

  void initXml(TiXmlElement* config);

  boost::shared_ptr<Link> getParent() const
  {return parent_link_.lock();};

  void setParent(boost::shared_ptr<Link> parent);

  void clear()
  {
    this->name.clear();
    this->sensor.reset();
    this->parent_link_.reset();
    this->parent_joint.reset();
    this->collision_groups.clear();
  };

private:
  boost::weak_ptr<Sensor> parent_link_;

};
}
#endif
