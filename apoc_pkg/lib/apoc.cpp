
/*
*   file：      apoc.c
*   name:       apoc.c
*   discribe:   apoc构造函数
*               ros话题初始化，变量初始化
*   vision:     1.0
*   path:
*/

#include "apoc_pkg/apoc.h"

apoc::apoc() : rate(20.0){

    //ros话题初始化
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &AutoPilotControl::state_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &AutoPilotControl::local_pos_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //变量初始化
    current_state.connected = false;
    current_state.armed = false;//disarm
    current_state.mode = "STABILIZED";  // 默认模式

    //pose
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    //home_pose
    home_pose.header.frame_id = "map";
    home_pose.pose.position.x = 0;
    home_pose.pose.position.y = 0;
    home_pose.pose.position.z = 0;
    home_pose.pose.orientation.x = 0.0;
    home_pose.pose.orientation.y = 0.0;
    home_pose.pose.orientation.z = 0.0;
    home_pose.pose.orientation.w = 1.0;

    //current_pose
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = 0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;

    last_request = ros::Time::now();

}