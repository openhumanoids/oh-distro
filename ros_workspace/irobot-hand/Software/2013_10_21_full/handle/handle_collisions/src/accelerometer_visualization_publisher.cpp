
#include "ros/ros.h"

#include <string>
#include <sstream>

// #include <handle_msgs/CableTension.h>
// #include <handle_msgs/Collision.h>
#include <handle_msgs/Finger.h>
// #include <handle_msgs/HandleCollisions.h>
// #include <handle_msgs/HandleControl.h>
// #include <handle_msgs/HandleSensorsCalibrated.h>
#include <handle_msgs/HandleSensors.h>

// #include <sensor_msgs/JointState.h>

#include <geometry_msgs/Point.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;

#define ARROW_LENGTH 0.2
#define ARROW_DIAMETER 0.01
#define ARROW_HEAD_DIAMETER 0.025

void callback(const handle_msgs::HandleSensorsConstPtr& data)
{
    visualization_msgs::MarkerArray marker_array;
    
    for (unsigned int i=0; i<3; i++)
    {
        //
        // distal accel
        //
        visualization_msgs::Marker dmarker;
        dmarker.header.stamp = ros::Time::now();
        
        dmarker.color.r = 0.0;
        dmarker.color.g = 0.0;
        dmarker.color.b = 1.0;
        dmarker.color.a = 1.0;
        
        dmarker.ns = "accelerometer_display";
        dmarker.id = i;
        
        std::stringstream dss;
        dss << "/finger[" << i << "]/distal_accelerometer";
		dmarker.header.frame_id = dss.str();
		dmarker.type = visualization_msgs::Marker::ARROW;
		dmarker.action = visualization_msgs::Marker::ADD;
        dmarker.lifetime = ros::Duration(0.1);
		dmarker.scale.x = ARROW_DIAMETER;
		dmarker.scale.y = ARROW_HEAD_DIAMETER;
		dmarker.scale.z = 0;
        
        dmarker.points.resize(2);
        dmarker.points[0] = geometry_msgs::Point();
        dmarker.points[1] = geometry_msgs::Point();
        dmarker.points[1].x = ARROW_LENGTH * data->distalAcceleration[i].x;
        dmarker.points[1].y = ARROW_LENGTH * data->distalAcceleration[i].y;
        dmarker.points[1].z = ARROW_LENGTH * data->distalAcceleration[i].z;
        
        marker_array.markers.push_back(dmarker);

        //
        // proximal accel
        //
        visualization_msgs::Marker pmarker;
        pmarker.header.stamp = ros::Time::now();
 
        pmarker.color.r = 0.0;
        pmarker.color.g = 0.0;
        pmarker.color.b = 1.0;
        pmarker.color.a = 1.0;
        
        pmarker.ns = "accelerometer_display";
        pmarker.id = 10+i;
        
        std::stringstream pss;
        pss << "/finger[" << i << "]/proximal_accelerometer";
		pmarker.header.frame_id = pss.str();
		pmarker.type = visualization_msgs::Marker::ARROW;
		pmarker.action = visualization_msgs::Marker::ADD;
        pmarker.lifetime = ros::Duration(0.1);
		pmarker.scale.x = ARROW_DIAMETER;
		pmarker.scale.y = ARROW_HEAD_DIAMETER;
		pmarker.scale.z = 0;
        
        pmarker.points.resize(2);
        pmarker.points[0] = geometry_msgs::Point();
        pmarker.points[1] = geometry_msgs::Point();
        pmarker.points[1].x = ARROW_LENGTH * data->proximalAcceleration[i].x;
        pmarker.points[1].y = ARROW_LENGTH * data->proximalAcceleration[i].y;
        pmarker.points[1].z = ARROW_LENGTH * data->proximalAcceleration[i].z;
        
        marker_array.markers.push_back(pmarker);
    }
    
    pub.publish(marker_array);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accelerometer_visualization_publisher");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/handle/sensors/raw", 1, callback);
  pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
  
  ros::spin();
  
  return 0;
}
