#include "../include/omega_publisher.h"

CE_OO::CE_OO(ros::NodeHandle *nh){
    n=nh;

    omega_pub = n->advertise<geometry_msgs::PoseStamped>("poseFromFile/PoseStampedRelative", 1000);
    sub_set_omega = n->subscribe("force_feedback_data", 1000, &CE_OO::setOmegaCallback, this);

    initialize_stuff();
}

void CE_OO::initialize_stuff(){

  //OMEGA open the first available device
  if (dhdOpen () < 0) {
    //printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    //return -1;
  }

  //OMEGA identify number of encoders to report based on device type
  switch (dhdGetSystemType ()) {
    case DHD_DEVICE_DELTA3:
    case DHD_DEVICE_OMEGA3:
    case DHD_DEVICE_FALCON:
        encCount = 3;
        break;
    case DHD_DEVICE_DELTA6:
    case DHD_DEVICE_OMEGA33:
    case DHD_DEVICE_OMEGA33_LEFT:
        encCount = 6;
        break;
    case DHD_DEVICE_OMEGA331:
    case DHD_DEVICE_OMEGA331_LEFT:
        encCount = 7;
        break;
    case DHD_DEVICE_CONTROLLER:
    case DHD_DEVICE_CONTROLLER_HR:
    default:
        encCount = 8;
        break;
    }

    // enable force
    dhdEnableForce (DHD_ON);
}

bool CE_OO::run(){

    ros::Rate r(100);

    //TEST SECTOR
    //------------------------------------------------

    //Accessing the variable by ROS_INFO_STREAM(q) doesn't work. 
    //It just gives the address of the variable q. 
    //Use the format of access to the variables below! Brackets after  
    //variables are used because they are protected.
    /*
    q = tf::createQuaternionFromRPY(0, 0, 0);
    ROS_INFO_STREAM(q.x());
    ROS_INFO_STREAM(q.y());
    ROS_INFO_STREAM(q.z());
    ROS_INFO_STREAM(q.w());
    */

    

    //-----------------------------------------------


    while (ros::ok())
    {
        ros::spinOnce();

        //apply zero force for gravity compensation in argument of if. if is for error handling but not necessary
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr() );
        }

        // update end-effector position and orientation variables
        dhdGetPositionAndOrientationDeg (&px, &py, &pz, &ox, &oy, &oz);

        // print out encoders according to system type
        // ROS_INFO_STREAM("Positions(" <<  px << "," << py << "," << pz << ")");

        // set variables of position in m
        pose.pose.position.x = px;
        pose.pose.position.y = py;
        pose.pose.position.z = pz;
        
        //commented lines from omega_publisher.cpp
        //----------------------------------------------
        /*
        pose.pose.orientation.x = ox;
        pose.pose.orientation.y = oy;
        pose.pose.orientation.z = oz;

        // get angle of gripper (range: 0° - 28°)
        dhdGetGripperAngleDeg(&gr);

        // set w of pose variable as angle of gripper
        pose.pose.orientation.w = gr;
        */
        //----------------------------------------------

        //convert Euler to Quaternion
        q = tf::createQuaternionFromRPY(ox, oy, oz);

        //set variables of rotation
        
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        

        omega_pub.publish(pose);


        // to hold the frequency determined before while loop
        r.sleep();
    }
}

void CE_OO::setOmegaCallback(const std_msgs::Int64::ConstPtr& msg)
{

    if(msg->data ==1){
        ROS_INFO_STREAM("Collision in progress");
        dhdSetForce (K*px, K*py,K*pz);
    }
    else{
        ROS_INFO_STREAM("Clear path");
        dhdSetForce (0.0,0.0,0.0);
    }

}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "omega7_qt");
    ros::NodeHandle n;
    CE_OO instance(&n);
    instance.run();
}

