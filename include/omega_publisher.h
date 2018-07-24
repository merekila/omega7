#ifndef __OMEGA_PUBLISHER_H__
#define __OMEGA_PUBLISHER_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include "dhdc.h"
#include "tf/tf.h"

#include "std_msgs/Int64.h"
#include <sstream>

#define K 50.0

//typedef tfScalar 

class CE_OO{
public:    
    CE_OO(ros::NodeHandle*);
    bool run();
    void setScalingFactors(double scalefactor_trans_x, double scalefactor_trans_y, double scalefactor_trans_z, double scalefactor_rot_x, double scalefactor_rot_y, double scalefactor_rot_z);

protected:
    ros::NodeHandle *n;
    ros::Publisher omega_pub;
    ros::Subscriber sub_set_omega;
    std_msgs::Int64 flag_collision;
    std::stringstream ss;
    geometry_msgs::PoseStamped pose;
    void setOmegaCallback(const std_msgs::Int64::ConstPtr& msg);
    void initialize_stuff();
    int enc[DHD_MAX_DOF];
    int encCount;
    double px, py, pz;
    double ox, oy, oz;
    double scaled_ox, scaled_oy, scaled_oz;
    tf::Quaternion q;
    double gr;
    double scale_trans_x, scale_trans_y, scale_trans_z;
    double scale_rot_x, scale_rot_y, scale_rot_z;
};
#endif
