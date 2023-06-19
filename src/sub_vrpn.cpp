#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>


geometry_msgs::PoseStamped pose, current_object_pose;

bool target_1_reached = false;
bool target_2_reached = false; 

//Parameters of gripper
int open_pwm = 1050, close_pwm = 1950;
mavros_msgs::OverrideRCIn override_rc_in;
int counter = 0, min_count = 200;

void object_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& object_pose){
    current_object_pose.pose.position.x = object_pose->pose.position.x;
    current_object_pose.pose.position.y = object_pose->pose.position.y;
    current_object_pose.pose.position.z = object_pose->pose.position.z;   
    //object_pose = *msg;  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber object_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_sav_cylinder/pose", 100, object_pose_cb);
    //ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100,pose_cb);
    //ros::Publisher override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",20);
    
    
    


    while(ros::ok()){
        //Get current object pose as initial object pose
        pose.pose.position.x = current_object_pose.pose.position.x;
        pose.pose.position.y = current_object_pose.pose.position.y + 0.1;
        pose.pose.position.z = current_object_pose.pose.position.z;

        std::cout<<"x_current_object_pose: "<< current_object_pose.pose.position.x<<std::endl;
        std::cout<<"y_current_object_pose: "<< current_object_pose.pose.position.y<<std::endl;
        std::cout<<"x_object_pose: "<< pose.pose.position.x<<std::endl;
        std::cout<<"y_object_pose: "<< pose.pose.position.y<<std::endl;

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}