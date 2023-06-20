#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>
#include <airo_px4/Reference.h>
#include <mavros_msgs/OverrideRCIn.h>


geometry_msgs::PoseStamped local_pose, object_pose, current_object_pose;
airo_px4::Reference target_pose_1;
airo_px4::Reference target_pose_2;
airo_px4::Reference target_pose_3;
airo_px4::FSMInfo fsm_info;
airo_px4::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;
bool target_2_reached = false; 

//Parameters of gripper
int open_pwm = 1050, close_pwm = 1950;
mavros_msgs::OverrideRCIn override_rc_in;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_px4::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void object_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& object_pose){
    current_object_pose.pose.position.x = object_pose->pose.position.x;
    current_object_pose.pose.position.y = object_pose->pose.position.y;
    current_object_pose.pose.position.z = object_pose->pose.position.z;   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    //ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber object_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_sav_cylinder/pose", 10, object_pose_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_px4::FSMInfo>("/airo_px4/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_px4::Reference>("/airo_px4/setpoint",10);
    // ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_px4/position_setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_px4::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);
    ros::Publisher override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",20);

    target_pose_1.ref_pose.resize(41);
    target_pose_1.ref_twist.resize(41);
    target_pose_2.ref_pose.resize(41);
    target_pose_2.ref_twist.resize(41);
    target_pose_3.ref_pose.resize(41);
    target_pose_3.ref_twist.resize(41);

    while(ros::ok()){
        //Get current object pose as initial object pose
        object_pose.pose.position.x = current_object_pose.pose.position.x;
        object_pose.pose.position.y = current_object_pose.pose.position.y;
        object_pose.pose.position.z = current_object_pose.pose.position.z;

        for (int i = 0; i < 41; i++){
            target_pose_1.ref_pose[i].position.x = -1.53;
            target_pose_1.ref_pose[i].position.y = 1.4;
            target_pose_1.ref_pose[i].position.z = 1;
            target_pose_1.ref_pose[i].orientation.w = 1;
            target_pose_1.ref_pose[i].orientation.x = 0.0;
            target_pose_1.ref_pose[i].orientation.y = 0.0;
            target_pose_1.ref_pose[i].orientation.z = 0.0;
        }

        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
                    override_rc_in.channels[9] = open_pwm; 
                    override_pub.publish(override_rc_in);
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(fsm_info.is_waiting_for_command){
                            state = COMMAND;
                            break;
                        }
                    }
                }
                break;
            }

            case COMMAND:{
                if(fsm_info.is_waiting_for_command){
                    if(!target_1_reached){
                        target_pose_1.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_1);
                        override_rc_in.channels[9] = open_pwm; 
                        override_pub.publish(override_rc_in);
                        std::cout<<"Pose 1"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_1.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.ref_pose[0].position.z) < 2){
                            //target_1_reached = true;
                            std::cout<<"target 1 is finished"<<std::endl;
                            std::cout<<"x tracking error: "<< (local_pose.pose.position.x - target_pose_1.ref_pose[0].position.x)<<std::endl;
                            std::cout<<"y tracking error: "<< (local_pose.pose.position.y - target_pose_1.ref_pose[0].position.y)<<std::endl;
                            std::cout<<"z tracking error: "<< (local_pose.pose.position.z - target_pose_1.ref_pose[0].position.z)<<std::endl;
                        }
                    }
                }   
            }
                break;
            

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                    override_rc_in.channels[9] = open_pwm; 
                    override_pub.publish(override_rc_in);
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}