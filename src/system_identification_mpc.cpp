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
mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pose,takeoff_pose,x_maneuver_pose,y_maneuver_pose,yaw_maneuver_pose;
std::string package_path = ros::package::getPath("airo_px4");
std::string POSE_TOPIC, YAML_NAME;
tf::Quaternion tf_quaternion;

airo_px4::Reference target_pose_1;
airo_px4::Reference target_pose_2;
airo_px4::Reference target_pose_3;
airo_px4::Reference target_pose_4;
airo_px4::FSMInfo fsm_info;
airo_px4::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;
bool target_2_reached = false; 
bool target_3_reached = false; 
bool target_4_reached = false; 

enum State{
    TAKEOFF,
    COMMAND,
    LAND,
    FINISH
};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
    local_pose_received = true;
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
    
    if (tau_phi_id){
        tau_phi_diff.push_back(current_target_euler.x() - current_euler.x());
        tau_phi_rate.push_back(angular_rate_msg->twist.angular.x);
    }else if (tau_theta_id){
        tau_theta_diff.push_back(current_target_euler.y() - current_euler.y());
        tau_theta_rate.push_back(angular_rate_msg->twist.angular.y);
    }else if (tau_psi_id){
        tau_psi_diff.push_back(current_target_euler.z() - current_euler.z());
        tau_psi_rate.push_back(angular_rate_msg->twist.angular.z);
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
    x_maneuver_pose.pose.position.x = takeoff_pose.pose.position.x + 1.5*sin(2*(ros::Time::now().toSec() - last_state_time.toSec()));
    x_maneuver_pose.pose.position.y = takeoff_pose.pose.position.y;
    x_maneuver_pose.pose.position.z = takeoff_pose.pose.position.z;
    x_maneuver_pose.pose.orientation.w = 1;
    x_maneuver_pose.pose.orientation.x = 0;
    x_maneuver_pose.pose.orientation.y = 0;
    x_maneuver_pose.pose.orientation.z = 0;
}

void update_y_maneuver(){
    y_maneuver_pose.pose.position.x = takeoff_pose.pose.position.x;
    y_maneuver_pose.pose.position.y = takeoff_pose.pose.position.y + 1.5*sin(2*(ros::Time::now().toSec() - last_state_time.toSec()));
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

    std::string yaml_path = package_path + YAML_NAME;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC,100,pose_cb);
    //ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_px4::FSMInfo>("/airo_px4/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_px4::Reference>("/airo_px4/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_px4::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);

    ros::Subscriber target_actuator_control_sub = nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control",100,target_actuator_control_cb);
    //ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    message_filters::Subscriber<geometry_msgs::PoseStamped> attitude_sub(nh,"/mavros/local_position/pose",10);
    message_filters::Subscriber<mavros_msgs::AttitudeTarget> attitude_target_sub(nh,"/mavros/setpoint_raw/target_attitude",10);
    message_filters::Subscriber<geometry_msgs::TwistStamped> angular_rate_sub(nh,"/mavros/local_position/velocity_local",10);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::AttitudeTarget, geometry_msgs::TwistStamped> my_sync_policy;
    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), attitude_sub,attitude_target_sub,angular_rate_sub);
    sync.registerCallback(boost::bind(&sync_cb,_1,_2,_3));



    takeoff_pose.pose.position.x = local_pose.pose.position.x;
    takeoff_pose.pose.position.y = local_pose.pose.position.y;
    takeoff_pose.pose.position.z = local_pose.pose.position.z + takeoff_height;
    takeoff_pose.pose.orientation.w = 1;
    takeoff_pose.pose.orientation.x = 0;
    takeoff_pose.pose.orientation.y = 0;
    takeoff_pose.pose.orientation.z = 0;


    target_pose_1.ref_pose.resize(41);
    target_pose_1.ref_twist.resize(41);
    target_pose_2.ref_pose.resize(41);
    target_pose_2.ref_twist.resize(41);
    target_pose_3.ref_pose.resize(41);
    target_pose_3.ref_twist.resize(41);
    target_pose_4.ref_pose.resize(41);
    target_pose_4.ref_twist.resize(41);


    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
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
                        command_pub.publish(takeoff_pose);
                        if (ros::Time::now().toSec() - last_state_time.toSec() > 10.0){
                            hover_thrust_id = false;
                            hover_thrust = std::accumulate(thrust.begin(),thrust.end(),0.0) / thrust.size();
                            target_1_reached = true;
                            tau_theta_id = true;
                            last_state_time = ros::Time::now();
                        }
                    }

                    if(target_1_reached && !target_2_reached){
                        if (ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_theta_id = false;
                            target_2_reached = true;
                            tau_phi_id = true;
                            last_state_time = ros::Time::now();
                            while(ros::ok()){
                                command_pub.publish(takeoff_pose);
                                if (target_reached(takeoff_pose)){
                                    break;
                                }
                                ros::spinOnce();
                                ros::Duration(rate).sleep();                        
                            }
                        }
                        else{
                            update_x_maneuver();
                            target_pose_2.header.stamp = ros::Time::now();
                            command_pub.publish(x_maneuver_pose);
                        }
                    }

                    if(target_2_reached && !target_3_reached){
                       if (ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_phi_id = false;
                            target_3_reached = true;
                            tau_psi_id = true;
                            last_state_time = ros::Time::now();
                            while(ros::ok()){
                                command_pub.publish(takeoff_pose);
                                if (target_reached(takeoff_pose)){
                                    break;
                                }
                                ros::spinOnce();
                                ros::Duration(rate).sleep();                        
                            }
                        }
                        else{
                            update_y_maneuver();
                            target_pose_3.header.stamp = ros::Time::now();
                            command_pub.publish(y_maneuver_pose);
                        } 
                    }
                    
                    if(target_3_reached && !target_4_reached){
                         if (ros::Time::now().toSec() - last_state_time.toSec() > 15.0){
                            tau_psi_id = false;
                            target_4_reached = true;
                            state = LAND;
                            while(ros::ok()){
                                command_pub.publish(takeoff_pose);
                                if (target_reached(takeoff_pose)){
                                    break;
                                }
                                ros::spinOnce();
                                ros::Duration(rate).sleep();                        
                            }
                        }
                        else{
                            update_yaw_maneuver();
                            target_pose_4.header.stamp = ros::Time::now();
                            command_pub.publish(yaw_maneuver_pose);
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
                    state = FINISH;
                }
                break;
            }

            case FINISH:{
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