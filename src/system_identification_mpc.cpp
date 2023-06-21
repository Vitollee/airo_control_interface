#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <numeric>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>
#include <airo_px4/Reference.h>

double hover_thrust, tau_phi, tau_theta, tau_psi;
double takeoff_height = 1.0;
double yaw_ref = M_PI/4;
std::vector<double> thrust,tau_phi_diff,tau_phi_rate,tau_theta_diff,tau_theta_rate,tau_psi_diff,tau_psi_rate;
bool hover_thrust_id = false;
bool tau_phi_id = false;
bool tau_theta_id = false;
bool tau_psi_id = false;
bool local_pose_received = false;
ros::Time last_state_time;
//mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pose,takeoff_pose,x_maneuver_pose,y_maneuver_pose,yaw_maneuver_pose;
std::string package_path = ros::package::getPath("airo_px4");
std::string POSE_TOPIC, YAML_NAME;
tf::Quaternion tf_quaternion;

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
    LAND,
    FINISH
};


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
    // local_pose_received = true;
}

void fsm_info_cb(const airo_px4::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

Eigen::Vector3d q2rpy(geometry_msgs::Quaternion q){
    Eigen::Vector3d euler;
    tf::quaternionMsgToTF(q,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

geometry_msgs::Quaternion rpy2q(Eigen::Vector3d euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.x(), euler.y(), euler.z());
    return quaternion;
}

void sync_cb(const geometry_msgs::PoseStampedConstPtr& attitude_msg, const mavros_msgs::AttitudeTargetConstPtr& attitude_target_msg, const geometry_msgs::TwistStampedConstPtr& angular_rate_msg){
    Eigen::Vector3d current_target_euler = q2rpy(attitude_target_msg->orientation);
    Eigen::Vector3d current_euler = q2rpy(attitude_msg->pose.orientation);
    std::cout<<"sync_cb starts"<<std::endl;
    if (tau_phi_id){
        tau_phi_diff.push_back(current_target_euler.x() - current_euler.x());
        std::cout<<"current_target_euler.x: "<<current_target_euler.x()<<std::endl;
        std::cout<<"current_euler.x: "<<current_euler.x()<<std::endl;
        tau_phi_rate.push_back(angular_rate_msg->twist.angular.x);
        std::cout<<"angular_rate_msg: "<<angular_rate_msg<<std::endl;
        //std::cout<<"twist.angular.x: "<<twist.angular.x<<std::endl;
    }else if (tau_theta_id){
        tau_theta_diff.push_back(current_target_euler.y() - current_euler.y());
        std::cout<<"current_target_euler.y: "<<current_target_euler.y()<<std::endl;
        std::cout<<"current_euler.y: "<<current_euler.y()<<std::endl;
        tau_theta_rate.push_back(angular_rate_msg->twist.angular.y);
        std::cout<<"angular_rate_msg: "<<angular_rate_msg<<std::endl;
    }else if (tau_psi_id){
        tau_psi_diff.push_back(current_target_euler.z() - current_euler.z());
        std::cout<<"current_target_euler.z: "<<current_target_euler.z()<<std::endl;
        std::cout<<"current_euler.z: "<<current_euler.z()<<std::endl;
        tau_psi_rate.push_back(angular_rate_msg->twist.angular.z);
        std::cout<<"angular_rate_msg: "<<angular_rate_msg<<std::endl;
    }
}

void target_actuator_control_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg){
    if (hover_thrust_id){
        thrust.push_back(msg->controls[3]);
    }
}

bool target_reached(const geometry_msgs::PoseStamped& msg){
    return sqrt(pow(msg.pose.position.x - local_pose.pose.position.x,2)+pow(msg.pose.position.y - local_pose.pose.position.y,2)
    +pow(msg.pose.position.z - local_pose.pose.position.z,2)) < 0.25;
}

void update_x_maneuver(){
    x_maneuver_pose.pose.position.x = takeoff_pose.pose.position.x + 1*sin(2*(ros::Time::now().toSec() - last_state_time.toSec()));
    x_maneuver_pose.pose.position.y = takeoff_pose.pose.position.y;
    x_maneuver_pose.pose.position.z = takeoff_pose.pose.position.z;
    x_maneuver_pose.pose.orientation.w = 1;
    x_maneuver_pose.pose.orientation.x = 0;
    x_maneuver_pose.pose.orientation.y = 0;
    x_maneuver_pose.pose.orientation.z = 0;
}

void update_y_maneuver(){
    y_maneuver_pose.pose.position.x = takeoff_pose.pose.position.x;
    y_maneuver_pose.pose.position.y = takeoff_pose.pose.position.y + 1*sin(2*(ros::Time::now().toSec() - last_state_time.toSec()));
    y_maneuver_pose.pose.position.z = takeoff_pose.pose.position.z;
    y_maneuver_pose.pose.orientation.w = 1;
    y_maneuver_pose.pose.orientation.x = 0;
    y_maneuver_pose.pose.orientation.y = 0;
    y_maneuver_pose.pose.orientation.z = 0;
}

void update_yaw_maneuver(){
    Eigen::Vector3d euler = q2rpy(local_pose.pose.orientation);
    if (euler.z() > M_PI/4 - M_PI/16 && euler.z() < M_PI/4 + M_PI/16 && yaw_ref == M_PI/4){
        yaw_ref = -M_PI/4;
    }
    else if (euler.z() > -M_PI/4 - M_PI/16 && euler.z() < -M_PI/4 + M_PI/16 && yaw_ref == -M_PI/4){
        yaw_ref = M_PI/4;
    }
    Eigen::Vector3d euler_ref{0,0,yaw_ref};
    yaw_maneuver_pose.pose.orientation = rpy2q(euler_ref);
    yaw_maneuver_pose.pose.position.x = takeoff_pose.pose.position.x;
    yaw_maneuver_pose.pose.position.y = takeoff_pose.pose.position.y;
    yaw_maneuver_pose.pose.position.z = takeoff_pose.pose.position.z;
}

double linear_regression(std::vector<double> x, std::vector<double> y){
    // y = a*x
    double mean_x = std::accumulate(x.begin(),x.end(),0.0)/x.size();
    double mean_y = std::accumulate(y.begin(),y.end(),0.0)/y.size();
    std::cout<<"mean x: "<<mean_x<<std::endl;
    std::cout<<"mean y: "<<mean_y<<std::endl;
    double numerator = 0.0;
    double denominator = 0.0;
    for (int i = 0; i < x.size();++i){
        numerator += (x[i] - mean_x) * (y[i] - mean_y);
        denominator += (x[i] - mean_x) * (x[i] - mean_x);
    }

    double a = numerator / denominator;
    return a;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "system_identification_node_mpc");
    ros::NodeHandle nh;
    ros::Rate rate(40);
    State state = TAKEOFF;

    nh.getParam("system_identification_node/pose_topic",POSE_TOPIC);
    nh.getParam("system_identification_node/yaml_name",YAML_NAME);

    //std::string yaml_path = package_path + YAML_NAME;
    std::string yaml_path = package_path + "/config/vicon_param.yaml";
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_px4::FSMInfo>("/airo_px4/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_px4::Reference>("/airo_px4/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_px4::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);

    ros::Subscriber target_actuator_control_sub = nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control",100,target_actuator_control_cb);
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> attitude_sub(nh,"/mavros/vision_pose/pose",10);
    message_filters::Subscriber<mavros_msgs::AttitudeTarget> attitude_target_sub(nh,"/mavros/setpoint_raw/target_attitude",10);
    message_filters::Subscriber<geometry_msgs::TwistStamped> angular_rate_sub(nh,"/mavros/local_position/velocity_local",10);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::AttitudeTarget, geometry_msgs::TwistStamped> my_sync_policy;
    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), attitude_sub,attitude_target_sub,angular_rate_sub);
    sync.registerCallback(boost::bind(&sync_cb,_1,_2,_3));


    target_pose_1.ref_pose.resize(41);
    target_pose_1.ref_twist.resize(41);
    target_pose_2.ref_pose.resize(41);
    target_pose_2.ref_twist.resize(41);
    target_pose_3.ref_pose.resize(41);
    target_pose_3.ref_twist.resize(41);
    target_pose_4.ref_pose.resize(41);
    target_pose_4.ref_twist.resize(41);
    target_pose_5.ref_pose.resize(41);
    target_pose_5.ref_twist.resize(41);

    ros::Time last_request = ros::Time::now();
    
    takeoff_pose.pose.position.x = local_pose.pose.position.x;
    takeoff_pose.pose.position.y = local_pose.pose.position.y;
    takeoff_pose.pose.position.z = local_pose.pose.position.z + takeoff_height;
    takeoff_pose.pose.orientation.w = 1;
    takeoff_pose.pose.orientation.x = 0;
    takeoff_pose.pose.orientation.y = 0;
    takeoff_pose.pose.orientation.z = 0;

    for (int i = 0; i < 41; i++){
        target_pose_1.ref_pose[i].position.x = takeoff_pose.pose.position.x;
        target_pose_1.ref_pose[i].position.y = takeoff_pose.pose.position.y;
        target_pose_1.ref_pose[i].position.z = takeoff_pose.pose.position.z;
        target_pose_1.ref_pose[i].orientation.w = 1;
        target_pose_1.ref_pose[i].orientation.x = 0.0;
        target_pose_1.ref_pose[i].orientation.y = 0.0;
        target_pose_1.ref_pose[i].orientation.z = 0.0;
    }

    for (int i = 0; i < 41; i++){
        target_pose_5.ref_pose[i].position.x = 0;
        target_pose_5.ref_pose[i].position.y = 0;
        target_pose_5.ref_pose[i].position.z = 1;
        target_pose_5.ref_pose[i].orientation.w = 1;
        target_pose_5.ref_pose[i].orientation.x = 0.0;
        target_pose_5.ref_pose[i].orientation.y = 0.0;
        target_pose_5.ref_pose[i].orientation.z = 0.0;
    }


    while(ros::ok()){
        for (int i = 0; i < 41; i++){
            target_pose_2.ref_pose[i].position.x = x_maneuver_pose.pose.position.x;
            target_pose_2.ref_pose[i].position.y = x_maneuver_pose.pose.position.y;
            target_pose_2.ref_pose[i].position.z = x_maneuver_pose.pose.position.z;
            target_pose_2.ref_pose[i].orientation.w = 1;
            target_pose_2.ref_pose[i].orientation.x = 0.0;
            target_pose_2.ref_pose[i].orientation.y = 0.0;
            target_pose_2.ref_pose[i].orientation.z = 0.0;
        }

        for (int i = 0; i < 41; i++){
            target_pose_3.ref_pose[i].position.x = y_maneuver_pose.pose.position.x;
            target_pose_3.ref_pose[i].position.y = y_maneuver_pose.pose.position.y;
            target_pose_3.ref_pose[i].position.z = y_maneuver_pose.pose.position.z;
            target_pose_3.ref_pose[i].orientation.w = 1;
            target_pose_3.ref_pose[i].orientation.x = 0.0;
            target_pose_3.ref_pose[i].orientation.y = 0.0;
            target_pose_3.ref_pose[i].orientation.z = 0.0;
        }

        for (int i = 0; i < 41; i++){
            target_pose_4.ref_pose[i].position.x = yaw_maneuver_pose.pose.position.x;
            target_pose_4.ref_pose[i].position.y = yaw_maneuver_pose.pose.position.y;
            target_pose_4.ref_pose[i].position.z = yaw_maneuver_pose.pose.position.z;
            target_pose_4.ref_pose[i].orientation = yaw_maneuver_pose.pose.orientation;
        }

       
        switch(state){
            case TAKEOFF:{
                if( fsm_info.is_landed == true){
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(fsm_info.is_waiting_for_command){
                            state = COMMAND;
                            hover_thrust_id = true;
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
                        std::cout<<"HOVER starts"<<std::endl;
                        std::cout<<"local x: "<<local_pose.pose.position.x<<"  local y: "<<local_pose.pose.position.y<<"  local z: "<<local_pose.pose.position.z<<std::endl;
                        std::cout<<"target_pose_1 x: "<<target_pose_1.ref_pose[0].position.x<<"  target_pose_1 y: "<<target_pose_1.ref_pose[0].position.y<<"  target_pose_1 z: "<<target_pose_1.ref_pose[0].position.z<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_1.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.ref_pose[0].position.z) < 0.5
                         && ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            target_1_reached = true;
                            hover_thrust_id = false;
                            hover_thrust = std::accumulate(thrust.begin(),thrust.end(),0.0) / thrust.size();
                            std::cout<<"hover_thrust: "<<hover_thrust<<std::endl;
                            tau_theta_id = true;
                            last_state_time = ros::Time::now();
                            std::cout<<"HOVER is finished"<<std::endl;
                            // state = FINISH;
                            // break;
                        }
                    }

                    if(target_1_reached && !target_2_reached){
                        update_x_maneuver();
                        std::cout<<"hover_thrust: "<<hover_thrust<<std::endl;
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        std::cout<<"local x: "<<local_pose.pose.position.x<<"  local y: "<<local_pose.pose.position.y<<"  local z: "<<local_pose.pose.position.z<<std::endl;
                        std::cout<<"target_pose_2 x: "<<target_pose_2.ref_pose[0].position.x<<"  target_pose_2 y: "<<target_pose_2.ref_pose[0].position.y<<"  target_pose_2 z: "<<target_pose_2.ref_pose[0].position.z<<std::endl;
                        std::cout<<"X_MANEUVER starts"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_2.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.ref_pose[0].position.z) < 0.5
                          && ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_theta_id = false;
                            target_2_reached = true;
                            tau_phi_id = true;
                            last_state_time = ros::Time::now();
                            std::cout<<"X_MANEUVER is done"<<std::endl;
                            // state = FINISH;
                            // break;
                        }
                    }

                    if(target_2_reached && !target_3_reached){
                        update_y_maneuver();
                        std::cout<<"hover_thrust: "<<hover_thrust<<std::endl;
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);
                        std::cout<<"local x: "<<local_pose.pose.position.x<<"  local y: "<<local_pose.pose.position.y<<"  local z: "<<local_pose.pose.position.z<<std::endl;
                        std::cout<<"target_pose_3 x: "<<target_pose_3.ref_pose[0].position.x<<"  target_pose_3 y: "<<target_pose_3.ref_pose[0].position.y<<"  target_pose_3 z: "<<target_pose_3.ref_pose[0].position.z<<std::endl;
                        std::cout<<"Y_MANEUVER starts"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_3.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_3.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_3.ref_pose[0].position.z) < 0.5
                          && ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_phi_id = false;
                            target_3_reached = true;
                            tau_psi_id = true;
                            last_state_time = ros::Time::now();
                            std::cout<<"Y_MANEUVER is done"<<std::endl;
                            // state = FINISH;
                            // break;
                        }
                    }
                    
                    if(target_3_reached && !target_4_reached){
                        update_yaw_maneuver();
                        target_pose_4.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_4);
                        std::cout<<"local x: "<<local_pose.pose.position.x<<"  local y: "<<local_pose.pose.position.y<<"  local z: "<<local_pose.pose.position.z<<std::endl;
                        std::cout<<"target_pose_4 x: "<<target_pose_4.ref_pose[0].position.x<<"  target_pose_4 y: "<<target_pose_4.ref_pose[0].position.y<<"  target_pose_4 z: "<<target_pose_4.ref_pose[0].position.z<<std::endl;
                        std::cout<<"YAW_MANEUVER starts"<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_4.ref_pose[0].position.x)
                         + abs(local_pose.pose.position.y - target_pose_4.ref_pose[0].position.y)
                         + abs(local_pose.pose.position.z - target_pose_4.ref_pose[0].position.z) < 0.5
                          && ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_psi_id = false;
                            target_4_reached = true;
                            last_state_time = ros::Time::now();
                            //state = LAND;
                            std::cout<<"YAW_MANEUVER is done"<<std::endl;
                            state = FINISH;
                            break;
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
                    std::cout<<"land"<<std::endl;
                    if (ros::Time::now().toSec() - last_state_time.toSec() > 5.0){
                        state = FINISH;
                        last_state_time = ros::Time::now();
                    }
                    
                }
                break;
            }

            case FINISH:{
                target_pose_5.header.stamp = ros::Time::now();
                command_pub.publish(target_pose_5);
                std::cout<<"pose 5"<<std::endl;
                tau_phi = 1 / linear_regression(tau_phi_diff,tau_phi_rate);
                tau_theta = 1 / linear_regression(tau_theta_diff,tau_theta_rate);
                tau_psi = 1 / linear_regression(tau_psi_diff,tau_psi_rate);
                std::cout<<"System identification result:"<<std::endl;
                std::cout<<"hover_thrust = "<<hover_thrust<<std::endl;
                std::cout<<"tau_phi = "<<tau_phi<<std::endl;
                std::cout<<"tau_theta = "<<tau_theta<<std::endl;
                std::cout<<"tau_psi = "<<tau_psi<<std::endl;
                std::cout<<"Save the parameters to .yaml file? (y/n)"<<std::endl;
                std::string input;
                std::cin>>input;
                if (input == "y" || input == "\n"){
                    YAML::Node yaml_config =  YAML::LoadFile(yaml_path);
                    std::ofstream yaml_file(yaml_path);
                    yaml_config["hover_thrust"] = hover_thrust;
                    yaml_config["tau_phi"] = tau_phi;
                    yaml_config["tau_theta"] = tau_theta;
                    yaml_config["tau_psi"] = tau_psi;
                    yaml_file << yaml_config;
                    yaml_file.close();
                    std::cout<<"Parameters saved!"<<std::endl;
                }
                std::cout<<"Exiting!"<<std::endl;
                return 0;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}