/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
int setpoint_number = 0;
// float set_points[5][3] = {{0,0,6},{1.5,-0.671518,3.4},{2,-0.671518,3.4},{2.8,1.799765,3.4},{3.5,1.799765,3.4}};
float set_points[5][3] = {{0,0,3.2},{1.5,-1,3.2},{2,-1,3.2},{2.8,2,3.2},{3.5,2,3.2}};
// float set_points[7][3] = {{10,10,1},{1,1,3},{2,1,3},{2,2,3},{3,2,3},{3,1,3},{5,1,3}};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
geometry_msgs::PoseStamped current_set_pose;
void current_set_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_set_pose = *msg;
}
geometry_msgs::PoseStamped pose;
void set_pose(int a, int b, int c){
    pose.pose.position.x = a;
    pose.pose.position.y = b;
    pose.pose.position.z = c;
}

double distance(int number){
   float x = set_points[number][0];
   float y = set_points[number][1];
   float z = set_points[number][2];
   double distance_cal = sqrt(pow(x-current_pose.pose.position.x,2)+pow(y-current_pose.pose.position.y,2)+ pow(z-current_pose.pose.position.z,2));
   return distance_cal;
}

bool is_set_point_reached(int number){
    double distance_cal = distance(number);
    
    if(distance_cal<=0.05)
    {   
        ROS_INFO("setpoint= %f %f %f current_point= %f %f %f distance = %f",set_points[number][0],set_points[number][1],set_points[number][2],current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,distance_cal);
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("move_base_simple/goal", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, current_pose_cb);
    ros::Subscriber current_set_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10, current_set_pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //  ROS_INFO("loading................");

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
         local_pos_pub.publish(pose);
        // if(setpoint_number<=4){
        //     ROS_INFO("Pose is published= " + setpoint_number);
        //     set_pose(set_points[setpoint_number][0],set_points[setpoint_number][1],set_points[setpoint_number][2]);
        //     local_pos_pub.publish(pose);
        // }else{
        //     ROS_INFO("Final Reached");
        //     local_pos_pub.publish(pose);
        // }
        // bool increase_setpoint_number = is_set_point_reached(setpoint_number);
        // if(increase_setpoint_number){
        //     setpoint_number++;
        // }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}