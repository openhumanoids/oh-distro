#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

Eigen::Matrix4d bTf; // from frame vicon to base link


// temp data structures for data conversion
tf::Transform temp_tf_transf;
tf::Vector3 temp_tf_vector;
tf::Quaternion temp_tf_quaternion;
tf::Matrix3x3 temp_tf_matrix;
Eigen::Matrix3d temp_eigen_matrix;
Eigen::Vector3d temp_eigen_vector;

// we send the transform between markers on the frame and base link through tf
// this is temporary!
tf::TransformBroadcaster *tb;

void fillBtF() {
    bTf <<        0.00000,   0.00000,  1.00000,  -0.26100,
        0.00000,   -1.00000,   0.00000,   0.19100,
        1.00000,   0.00000,   0.00000,  -0.44300,
        0.00000,  0.00000,   0.00000,   1.00000;
}


// get the transforms from vicon and send the transform bTc
void transformCallBack(const geometry_msgs::TransformStampedConstPtr& wf_in) {
    ros::Time now = wf_in->header.stamp;

    // Send the transform between frame and base link to tf
    temp_eigen_matrix = bTf.block(0, 0, 3, 3);
    tf::matrixEigenToTF(temp_eigen_matrix, temp_tf_matrix);
    temp_tf_matrix.getRotation(temp_tf_quaternion);
    temp_tf_transf.setRotation(temp_tf_quaternion);

    temp_eigen_vector = bTf.block(0, 3, 3, 1);
    tf::vectorEigenToTF(temp_eigen_vector, temp_tf_vector);
    temp_tf_transf.setOrigin(temp_tf_vector);

    tb->sendTransform(tf::StampedTransform(temp_tf_transf,
                                           now,
                                           "/vicon/hyq/body",
                                           "base_link"));
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "frame_to_body_node");
    ros::NodeHandle nh;


    fillBtF();

    // Listens to the vicon message
    ros::Subscriber wf_sub = nh.subscribe("/vicon/hyq/body", 1000, transformCallBack);

    tb = new tf::TransformBroadcaster();

    ros::spin();

    return 0;
}



