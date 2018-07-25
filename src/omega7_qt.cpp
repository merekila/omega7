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

void CE_OO::setScalingFactors(double scalefactor_trans_x, double scalefactor_trans_y, double scalefactor_trans_z, double scalefactor_rot_x, double scalefactor_rot_y, double scalefactor_rot_z){
    scale_trans_x = scalefactor_trans_x;
    scale_trans_y = scalefactor_trans_y;
    scale_trans_z = scalefactor_trans_z;
    scale_rot_x = scalefactor_rot_x;
    scale_rot_y = scalefactor_rot_y;
    scale_rot_z = scalefactor_rot_z;
}
/*not completed...
    void computeQuaternionFromSpatialRotation(double x_rot, double y_rot, double z_rot, tf::Quaternion quat){
        quat.x()= cos(x_rot/2);
        quat.y()= sin(x_rot/2)
        quat.z()= sin(x_rot/2)
        quat.w()= sin(x_rot/2)
    }
*/

bool CE_OO::run(){

    ros::Rate r(20);

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

        // set variables of scaled position in m
        pose.pose.position.x = py * -scale_trans_y;
        pose.pose.position.y = pz * scale_trans_z;
        pose.pose.position.z = px * -scale_trans_x;
        
        //scale the rotation
      

        scaled_ox = oy * -scale_rot_y;
        scaled_oy = oz * scale_rot_z;
        scaled_oz = ox * -scale_rot_x;
/*
        scaled_ox = ox * scale_rot_x;
        scaled_oy = oy * scale_rot_y;
        scaled_oz = oz * scale_rot_z;

        
        std::cout<< "scaled ox is "<< scaled_ox<<std::endl;
        std::cout<< "scaled oy is "<< scaled_oy<<std::endl;
        std::cout<< "scaled oz is "<< scaled_oz<<std::endl;
 */
        //Transformation
        rollMatrix.setEulerYPR(0, 0, scaled_ox);
        pitchMatrix.setEulerYPR(0, scaled_oy, 0);
        yawMatrix.setEulerYPR(scaled_oz, 0, 0);

        rotMatrix *= rollMatrix;
        rotMatrix = pitchMatrix * rotMatrix;
        rotMatrix = yawMatrix * rotMatrix;

        rotMatrix.getRotation(q);
 
 
 
 /*       
        //convert Euler to Quaternion
        q = tf::createQuaternionFromRPY(scaled_ox, scaled_oy, scaled_oz);
        //std::cout<< "x of quaternion q_raw is "<< q_raw.x()<<std::endl;

/*
        rotMatrix.setRPY(ox,oy,oz);
        //rotMatrix.setEulerYPR(scaled_oz, scaled_oy, scaled_ox);
        euler_x_omega_new = -euler_y_omega * scale_rot_y;
        euler_y_omega_new = euler_z_omega * scale_rot_z;
        euler_z_omega_new = -euler_x_omega * scale_rot_x;
        std::cout<< "euler_x_omega_new is" << euler_x_omega_new<< std::endl;
        rotMatrix.setEulerYPR(euler_z_omega_new, euler_y_omega_new, euler_x_omega_new);
        rotMatrix.getRotation(q);
        std::cout<< "x of quaternion is q"<< q.x()<<std::endl;
*/



        //set rotation variables of PoseStamped message
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
    instance.setScalingFactors(3, 3, 3, 0.1, 0.03, 0.03);
    instance.run();
}

