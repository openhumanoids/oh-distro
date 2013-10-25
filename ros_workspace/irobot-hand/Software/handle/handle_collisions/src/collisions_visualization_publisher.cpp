
#include "ros/ros.h"

#include <string>
//#include <iostream>
#include <ncurses.h>

// #include <handle_msgs/CableTension.h>
// #include <handle_msgs/Collision.h>
#include <handle_msgs/Finger.h>
// #include <handle_msgs/HandleCollisions.h>
// #include <handle_msgs/HandleControl.h>
// #include <handle_msgs/HandleSensorsCalibrated.h>
#include <handle_msgs/HandleSensors.h>

// #include <sensor_msgs/JointState.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;

float min_val;
float max_val;

float cal[114];
bool calibrate = true;

#define THRESHOLD 30.0

void proximalPos(int sensor, double& x, double& y, double& z)
{
    int rows = 2;
    int cols = 6;
    int mapping[12] = {11,10,9,8,7,6,5,4,3,2,1,0};
    float z_init = 0.015;
    float x_init = -0.003;
    float z_spacing = 0.007;
    float x_spacing = 0.007;
    x = x_init + (mapping[sensor] % rows) * x_spacing;
    y = 0.01; // offset from surface
    z = z_init + ((mapping[sensor] / rows) % cols) * z_spacing;
}

void distalPos(int sensor, double& x, double& y, double& z)
{
    int rows = 2;
    int cols = 5;
    int mapping[10] = {7,6,5,4,3,2,1,0,8,9};
    float z_init = 0.004;	
    float x_init = -0.003;
    float z_spacing = 0.007;
    float x_spacing = 0.006;
    x = x_init + (mapping[sensor] % rows) * x_spacing;
    y = 0.01; // offset from surface
    z = z_init + ((mapping[sensor] / rows) % cols) * z_spacing;
}

void palmPos(int sensor, double& x, double& y, double& z)
{
    //self.rows = 10
    //self.columns = 11
    float x_init = 0;
    float y_init = -0.01;
    float x_spacing = 0.045/335.8;
    float y_spacing = 0.045/335.8;
    float tact_x[48] = {335.8246,289.5579,328.5193,320.4023,311.4737,302.545,296.0515,283.876,240.8561,198.648,216.5053,228.6807,223.8105,144.2643,147.5111,66.3415,62.283,60.6596,60.6596,32.2503,1.4058,1.4058,1.4058,-1.0292,-29.4386,1.4058,-56.2246,0.594200000000001,-57.848,0.594200000000001,-57.0363,3.0292,-64.3415,-145.5111,-139.0175,-219.3754,-225.869,-215.317,-193.4012,-235.6094,-277.8175,-290.8047,-298.9216,-307.0386,-315.9673,-324.8959,-287.5579,-333.0129};
    float tact_y[48] = {-16.5456,13.4871,38.6497,93.0333,144.9819,199.3655,251.314,304.886,325.9901,300.8275,220.4696,140.1117,57.3187,64.624,147.417,143.3585,96.2801,28.0977,-43.3316,-98.5269,-147.2287,-204.0474,-261.6778,-320.1199,-97.7152,-62.0006,-44.1433,-5.18189999999998,29.7211,63.0006,94.6567,130.3713,143.3585,147.417,65.4357,58.9421,141.7351,221.2813,302.4509,327.6135,305.6977,252.1257,199.3655,145.7936,92.2216,39.4614,15.1105,-13.2988};
    x = x_spacing * tact_x[sensor] + x_init;
    y = -(y_spacing * tact_y[sensor] + y_init);
    z = 0.08; // offset from surface
}

void set_max_min(const handle_msgs::HandleSensorsConstPtr& data)
{
    min_val = data->fingerTactile[0].proximal[0] - cal[0];
    max_val = data->fingerTactile[0].proximal[0] - cal[0];
    
    int count = 0;
    for (int finger=0; finger<3; finger++)
    {
        for (int i=0; i<12; i++)
        {
            if (data->fingerTactile[finger].proximal[i]-cal[count] > max_val)
                max_val = data->fingerTactile[finger].proximal[i]-cal[count];
            else if (data->fingerTactile[finger].proximal[i]-cal[count] < min_val)
                min_val = data->fingerTactile[finger].proximal[i]-cal[count];
            count++;
        }
        
        for (int i=0; i<10; i++)
        {
            if (data->fingerTactile[finger].distal[i]-cal[count] > max_val)
                max_val = data->fingerTactile[finger].distal[i]-cal[count];
            else if (data->fingerTactile[finger].distal[i]-cal[count] < min_val)
                min_val = data->fingerTactile[finger].distal[i]-cal[count];
            count++;
        }
    }
    for (int i=0; i<48; i++)
    {
        if (data->palmTactile[i]-cal[count] > max_val)
            max_val = data->palmTactile[i]-cal[count];
        if (data->palmTactile[i]-cal[count] < min_val)
            min_val = data->palmTactile[i]-cal[count];
        count++;
    }
};


visualization_msgs::Marker makeMarker(float val)
{
    visualization_msgs::Marker marker;
    
    if (val > 0)
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }
    else
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }
    // marker.color.r = (val - min_val) / (max_val - min_val);
    // marker.color.g = (max_val - val) / (max_val - min_val);
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;
    
    marker.ns = "tactile_display";
    //marker.id set later
    //marker.header.frame_id set later
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.0002 * fabs(val) + 0.001;
    //marker.pose.position.x set later
    //marker.pose.position.y set later
    //marker.pose.position.z set later
    marker.pose.orientation.x = 0.0;        
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.707106781187;
    marker.pose.orientation.w = 0.707106781187;

    // marker.type = visualization_msgs::Marker::CUBE;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.lifetime = ros::Duration(0.1);
    // marker.scale.x = 0.0051;
    // marker.scale.y = 0.0051;
    // marker.scale.z = 0.0051;
    // //marker.pose.position.x set later
    // //marker.pose.position.y set later
    // //marker.pose.position.z set later
    // marker.pose.orientation.x = 0.0;        
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    
    return marker;
};

void callback(const handle_msgs::HandleSensorsConstPtr& data)
{
    if (calibrate)
    {
        printf("calibrating\n\r");
        int count = 0;
        for (int finger=0; finger<3; finger++)
        {
            for (int i=0; i<12; i++)
                cal[count++] = data->fingerTactile[finger].proximal[i];
            for (int i=0; i<10; i++)
                cal[count++] = data->fingerTactile[finger].distal[i];
        }
        for (int i=0; i<48; i++)
            cal[count++] = data->palmTactile[i];
        calibrate = false;
        set_max_min(data);
    }
    
    visualization_msgs::MarkerArray marker_array;
    int count = 0;
    
    for (int finger=0; finger<3; finger++)
    {
        char pframeid[50];
        sprintf(pframeid, "/finger[%d]/proximal_link", finger);
        char dframeid[50];
        sprintf(dframeid, "/finger[%d]/distal_link", finger);

        for (int i=0; i<12; i++)
        {
            float val = data->fingerTactile[finger].proximal[i]-cal[count];
            
            if (fabs(val) > THRESHOLD)
            {
                visualization_msgs::Marker marker = makeMarker(val);
                marker.id = count;
                marker.header.frame_id = pframeid;
                proximalPos(i, 
                            marker.pose.position.x,
                            marker.pose.position.y,
                            marker.pose.position.z);
                marker_array.markers.push_back(marker);
            }
            count++;
        }
        
        for (int i=0; i<10; i++)
        {
            float val = data->fingerTactile[finger].distal[i]-cal[count];
            if (fabs(val) > THRESHOLD)
            {
                visualization_msgs::Marker marker = makeMarker(val);
                marker.id = count;
                marker.header.frame_id = dframeid;
                distalPos(i, 
                          marker.pose.position.x,
                          marker.pose.position.y,
                          marker.pose.position.z);
                marker_array.markers.push_back(marker);
            }
            count++;
        }
    }
    
    for (int i=0; i<48; i++)
    {
        float val = data->palmTactile[i]-cal[count];
        if (fabs(val) > THRESHOLD)
        {
            visualization_msgs::Marker marker = makeMarker(val);
            marker.id = count;
            marker.header.frame_id = "base_link";
            palmPos(i, 
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = -0.707106781187;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.707106781187;
            marker_array.markers.push_back(marker);
        }
        count++;
    }
    
    pub.publish(marker_array);
};

int main(int argc, char **argv)
{
  for (int i=0; i<114; i++)
      cal[i] = 0;

  ros::init(argc, argv, "collisions_visualization_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/handle/sensors/raw", 1, callback);
  pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

  //ros::AsyncSpinner spinner(1); // Use 4 threads
  //spinner.start();

  //while (!ros::isShuttingDown()) //ros::ok())
  
  printf("Press 'c' to calibrate tactile sensors:\n\r");

  SCREEN *s = newterm(NULL, stdin, stdout);
  //initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  //refresh();

  ros::Rate r(200);
  while (ros::ok())
  {
      char c = getch();
      //printw("%c\n", c);
      //std::cin >> c;
      if (c == 'c')
      {
          printf("ok\n\r");
          calibrate = true;
      }
      //refresh();
      ros::spinOnce();
      r.sleep();
  }
  
  printf("exited while loop\n\r");
  //spinner.stop();
  endwin();
  delscreen(s);
  
  return 0;
}
