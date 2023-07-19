#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>
#include <airo_px4/Reference.h>

geometry_msgs::PoseStamped local_pose;
geometry_msgs::PoseStamped target_pose_1, target_pose_2, target_pose_3, target_pose_4, target_pose_5;
//geometry_msgs::PoseStamped target_pose_2;
airo_px4::FSM_Info fsm_info;
airo_px4::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;
bool target_2_reached = false;
bool target_3_reached = false;
bool target_4_reached = false;
bool target_5_reached = false;
double pose2x = -1, pose2y = 1, pose2z =1;
double pose3x = 0, pose3y = -1, pose3z = 0.5;
double pose4x = 1, pose4y = -1, pose4z = 1;
double pose5x = 1, pose5y = 0, pose5z = 0.5;
double pose6x = 0, pose6y = 0, pose6z = 1;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_px4::FSM_Info::ConstPtr& msg){
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

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_px4::FSM_Info>("/airo_px4/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_px4/position_setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_px4::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);

    target_pose_1.pose.position.x = pose2x;
    target_pose_1.pose.position.y = pose2y;
    target_pose_1.pose.position.z = pose2z;

    target_pose_2.pose.position.x = pose3x;
    target_pose_2.pose.position.y = pose3y;
    target_pose_2.pose.position.z = pose3z;

    target_pose_3.pose.position.x = pose4x;
    target_pose_3.pose.position.y = pose4y;
    target_pose_3.pose.position.z = pose4z;

    target_pose_4.pose.position.x = pose5x;
    target_pose_4.pose.position.y = pose5y;
    target_pose_4.pose.position.z = pose5z;

    target_pose_5.pose.position.x = pose6x;
    target_pose_5.pose.position.y = pose6y;
    target_pose_5.pose.position.z = pose6z;

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
                        if(abs(local_pose.pose.position.x - target_pose_1.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.pose.position.z) < 0.5){
                            target_1_reached = true;
                        }
                    }
                    if(!target_2_reached && target_1_reached){
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        if(abs(local_pose.pose.position.x - target_pose_2.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.pose.position.z) < 0.5){
                            target_2_reached = true;
                        }
                    }
                    if(!target_3_reached && target_2_reached){
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);
                        if(abs(local_pose.pose.position.x - target_pose_3.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_3.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_3.pose.position.z) < 0.5){
                            target_3_reached = true;
                        }
                    }
                    if(!target_4_reached && target_3_reached){
                        target_pose_4.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_4);
                        if(abs(local_pose.pose.position.x - target_pose_4.pose.position.x) < 0.15 &&
                           abs(local_pose.pose.position.y - target_pose_4.pose.position.y) < 0.15 &&
                           abs(local_pose.pose.position.z - target_pose_4.pose.position.z) < 0.15){
                            target_4_reached = true;
                        }
                    }
                    if(!target_5_reached && target_4_reached){
                        target_pose_5.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_5);
                        if(abs(local_pose.pose.position.x - target_pose_5.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_5.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_5.pose.position.z) < 0.5){
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