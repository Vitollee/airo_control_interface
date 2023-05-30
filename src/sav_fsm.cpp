#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>
#include <airo_px4/Reference.h>

geometry_msgs::PoseStamped local_pose;
airo_px4::Reference target_pose_1;
airo_px4::Reference target_pose_2;
airo_px4::Reference target_pose_3;
airo_px4::Reference target_pose_4;
airo_px4::Reference target_pose_5;
airo_px4::FSMInfo fsm_info;
airo_px4::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;
bool target_2_reached = false;
bool target_3_reached = false;
bool target_4_reached = false;
bool target_5_reached = false;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    //ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_px4::FSMInfo>("/airo_px4/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_px4::Reference>("/airo_px4/setpoint",10);
    // ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_px4/position_setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_px4::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);

    target_pose_1.ref_pose.resize(41);
    target_pose_1.ref_twist.resize(41);
    target_pose_2.ref_pose.resize(41);
    target_pose_2.ref_twist.resize(42);
    target_pose_3.ref_pose.resize(41);
    target_pose_3.ref_twist.resize(41);
    target_pose_4.ref_pose.resize(41);
    target_pose_4.ref_twist.resize(42);
    target_pose_5.ref_pose.resize(41);
    target_pose_5.ref_twist.resize(42);

    for (int i = 0; i < 41; i++){
        target_pose_1.ref_pose[i].position.x = -1;
        target_pose_1.ref_pose[i].position.y = 1;
        target_pose_1.ref_pose[i].position.z = 1;
        target_pose_1.ref_pose[i].orientation.w = 1;
        target_pose_1.ref_pose[i].orientation.x = 0.0;
        target_pose_1.ref_pose[i].orientation.y = 0.0;
        target_pose_1.ref_pose[i].orientation.z = 0.0;
    }

    for (int i = 0; i < 41; i++){
        target_pose_2.ref_pose[i].position.x = 0;
        target_pose_2.ref_pose[i].position.y = -1;
        target_pose_2.ref_pose[i].position.z = 0.5;
        target_pose_2.ref_pose[i].orientation.w = 1;
        target_pose_2.ref_pose[i].orientation.x = 0.0;
        target_pose_2.ref_pose[i].orientation.y = 0;
        target_pose_2.ref_pose[i].orientation.z = 0.;
    }

    for (int i = 0; i < 41; i++){
        target_pose_3.ref_pose[i].position.x = 1;
        target_pose_3.ref_pose[i].position.y = -1;
        target_pose_3.ref_pose[i].position.z = 1;
        target_pose_3.ref_pose[i].orientation.w = 1;
        target_pose_3.ref_pose[i].orientation.x = 0.0;
        target_pose_3.ref_pose[i].orientation.y = 0;
        target_pose_3.ref_pose[i].orientation.z = 0.;
    }

    for (int i = 0; i < 41; i++){
        target_pose_4.ref_pose[i].position.x = 1;
        target_pose_4.ref_pose[i].position.y = 0;
        target_pose_4.ref_pose[i].position.z = 0.5;
        target_pose_4.ref_pose[i].orientation.w = 1;
        target_pose_4.ref_pose[i].orientation.x = 0.0;
        target_pose_4.ref_pose[i].orientation.y = 0;
        target_pose_4.ref_pose[i].orientation.z = 0.;
    }

    for (int i = 0; i < 41; i++){
        target_pose_5.ref_pose[i].position.x = 0;
        target_pose_5.ref_pose[i].position.y = 0;
        target_pose_5.ref_pose[i].position.z = 1;
        target_pose_5.ref_pose[i].orientation.w = 1;
        target_pose_5.ref_pose[i].orientation.x = 0.0;
        target_pose_5.ref_pose[i].orientation.y = 0;
        target_pose_5.ref_pose[i].orientation.z = 0.;
    }


    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
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
                        std::cout<<"pose 1"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_1.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.ref_pose[0].position.z) < 0.5){
                            target_1_reached = true;
                        }
                    }
                    if(target_1_reached && !target_2_reached){
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        std::cout<<"pose 2"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_2.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.ref_pose[0].position.z) < 0.5){
                            target_2_reached = true;
                        }
                    }
                    if(target_2_reached && !target_3_reached){
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);
                        std::cout<<"pose 3"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_3.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_3.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_3.ref_pose[0].position.z) < 0.5){
                            target_3_reached = true;
                        }
                    }
                    if(target_3_reached && !target_4_reached){
                        target_pose_4.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_4);
                        std::cout<<"pose 4"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_4.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_4.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_4.ref_pose[0].position.z) < 0.5){
                            target_4_reached = true;
                        }
                    }
                    if(target_4_reached && !target_5_reached){
                        target_pose_5.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_5);
                        std::cout<<"pose 5"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_5.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_5.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_5.ref_pose[0].position.z) < 0.5){
                            target_5_reached = true;
                            state = LAND;
                        }
                    }
                    
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}