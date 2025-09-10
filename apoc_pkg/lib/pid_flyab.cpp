/*
* file:     pid_flab.cpp
* name：    flytoPIDcorrect
* describe：使用PID控制飞行到绝对坐标fly_ab_x/y/z/yaw
* input：   fly_pid_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute
* vision:   1.0
* method：  使用pid控制器计算步长，使用flytoAbsolute飞到中间步，直到到达目的地，reachCheck检查到达
*/

#include "apoc_pkg/apoc.h"



bool apoc::flytoPIDcorrect(float fly_pid_x, float fly_pid_y, float fly_pid_z, float fly_pid_yaw) {

    if (!current_state.armed) {
        ROS_WARN("Cannot execute PID flight: Vehicle is not armed");
        return false;
    }

    // 2. 初始化PID控制器（使用带参数的构造函数）
    pidctrl pid_x(
        PID_X_KP, PID_X_KI, PID_X_KD,
        PID_X_OUT_MIN, PID_X_OUT_MAX,
        PID_X_INT_MIN, PID_X_INT_MAX
    );
    pidctrl pid_y(
        PID_Y_KP, PID_Y_KI, PID_Y_KD,
        PID_Y_OUT_MIN, PID_Y_OUT_MAX,
        PID_Y_INT_MIN, PID_Y_INT_MAX
    );
    pidctrl pid_z(
        PID_Z_KP, PID_Z_KI, PID_Z_KD,
        PID_Z_OUT_MIN, PID_Z_OUT_MAX,
        PID_Z_INT_MIN, PID_Z_INT_MAX
    );
    pidctrl pid_yaw(
        PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
        PID_YAW_OUT_MIN, PID_YAW_OUT_MAX,
        PID_YAW_INT_MIN, PID_YAW_INT_MAX
    );

    // 3. 设置PID目标值（绝对坐标）
    pid_x.setSetpoint(fly_pid_x);
    pid_y.setSetpoint(fly_pid_y);
    pid_z.setSetpoint(fly_pid_z);

    //待修改

    // 处理偏航角周期性（确保目标角在[-π, π]范围内）
    float normalized_yaw = fmod(fly_pid_yaw, 2 * M_PI);
    if (normalized_yaw > M_PI) normalized_yaw -= 2 * M_PI;
    else if (normalized_yaw < -M_PI) normalized_yaw += 2 * M_PI;
    pid_yaw.setSetpoint(normalized_yaw);

    // 4. 初始化控制变量
    ros::Rate control_rate(PID_CONTROL_RATE); // PID控制频率
    ros::Time start_time = ros::Time::now();
    bool target_reached = false;

    ROS_INFO_STREAM("Start PID flight to target: [X:" << fly_pid_x 
                  << ", Y:" << fly_pid_y << ", Z:" << fly_pid_z 
                  << ", Yaw:" << normalized_yaw << " rad]");

    // 5. PID控制主循环（分步飞行直到到达目标）
    while (ros::ok()) {
        // 5.1 检查超时
        if ((ros::Time::now() - start_time).toSec() > PID_FLIGHT_TIMEOUT) {
            ROS_WARN("PID flight timed out (exceed %ds)", (int)PID_FLIGHT_TIMEOUT);
            return false;
        }

        // 5.2 检查是否已到达目标（调用reachCheck函数验证）
        target_reached = reachCheck(fly_pid_x, fly_pid_y, fly_pid_z, normalized_yaw);
        if (target_reached) {
            ROS_INFO("Successfully reached target via PID control");
            return true;
        }

        // 5.3 获取当前位置和姿态
        float current_x = current_position.pose.position.x;
        float current_y = current_position.pose.position.y;
        float current_z = current_position.pose.position.z;

        // 5.4 处理当前偏航角（转换为[-π, π]范围，与目标角统一）
        tf2::Quaternion current_quat(
            current_position.pose.orientation.x,
            current_position.pose.orientation.y,
            current_position.pose.orientation.z,
            current_position.pose.orientation.w
        );
        tf2::Matrix3x3 current_mat(current_quat);
        double roll, pitch, current_yaw;
        current_mat.getRPY(roll, pitch, current_yaw);
        // 归一化当前偏航角
        current_yaw = fmod(current_yaw, 2 * M_PI);
        if (current_yaw > M_PI) current_yaw -= 2 * M_PI;
        else if (current_yaw < -M_PI) current_yaw += 2 * M_PI;

        // 5.5 计算PID控制输出（步长增量）
        float delta_x = pid_x.compute(current_x);    // X轴步长增量
        float delta_y = pid_y.compute(current_y);    // Y轴步长增量
        float delta_z = pid_z.compute(current_z);    // Z轴步长增量
        float delta_yaw = pid_yaw.compute(current_yaw); // 偏航角步长增量

        // 5.6 计算中间目标点（当前位置 + PID步长增量）
        float via_x = current_x + delta_x;
        float via_y = current_y + delta_y;
        float via_z = current_z + delta_z;
        float via_yaw = current_yaw + delta_yaw;
        // 归一化中间目标偏航角
        via_yaw = fmod(via_yaw, 2 * M_PI);
        if (via_yaw > M_PI) via_yaw -= 2 * M_PI;
        else if (via_yaw < -M_PI) via_yaw += 2 * M_PI;

        // 5.7 调用绝对飞行函数，飞往中间目标点
        if (!flytoAbsolute(via_x, via_y, via_z, via_yaw)) {
            ROS_WARN("Failed to send intermediate target via flytoAbsolute");
            // 短暂延迟后重试，避免频繁失败
            ros::Duration(0.1).sleep();
            control_rate.sleep();
            continue;
        }

        // 5.8 打印调试信息（可选，便于监控控制过程）
        ROS_DEBUG_STREAM("PID Control: Current[X:" << current_x << ", Y:" << current_y 
                       << ", Z:" << current_z << ", Yaw:" << current_yaw 
                       << "] | Delta[X:" << delta_x << ", Y:" << delta_y 
                       << ", Z:" << delta_z << ", Yaw:" << delta_yaw 
                       << "] | Via[X:" << via_x << ", Y:" << via_y 
                       << ", Z:" << via_z << ", Yaw:" << via_yaw << "]");

        // 5.9 控制频率同步
        ros::spinOnce();
        control_rate.sleep();
    }

    // 6. 异常退出处理
    ROS_ERROR("PID flight exited abnormally (ROS node shutdown)");
    return false;
}