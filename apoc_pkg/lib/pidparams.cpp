#include "apoc.h"
#include <ros/param.h>  // 用于从参数服务器读取参数

// 实现PID控制器参数初始化函数
void apoc::initPIDControllers() {
    /*********************************************************************
     * 1. 定义PID参数变量（默认值针对多旋翼无人机优化，可通过launch文件覆盖）
     ********************************************************************/
    // X轴位置PID参数（默认：响应较快，抑制超调）
    float kp_x = 0.8f, ki_x = 0.2f, kd_x = 0.1f;
    float out_min_x = -0.5f, out_max_x = 0.5f;  // X轴控制量限幅（米/步，防止急刹）
    float int_min_x = -0.3f, int_max_x = 0.3f;  // X轴积分限幅（防止积分饱和）

    // Y轴位置PID参数（与X轴对称，保持平面运动一致性）
    float kp_y = 0.8f, ki_y = 0.2f, kd_y = 0.1f;
    float out_min_y = -0.5f, out_max_y = 0.5f;
    float int_min_y = -0.3f, int_max_y = 0.3f;

    // Z轴位置PID参数（更保守，避免高度剧烈波动）
    float kp_z = 1.0f, ki_z = 0.3f, kd_z = 0.15f;
    float out_min_z = -0.3f, out_max_z = 0.3f;  // Z轴控制量限幅（比XY轴小，防止炸机）
    float int_min_z = -0.2f, int_max_z = 0.2f;

    // Yaw偏航角PID参数（低超调，确保航向稳定）
    float kp_yaw = 0.6f, ki_yaw = 0.1f, kd_yaw = 0.05f;
    float out_min_yaw = -0.2f, out_max_yaw = 0.2f;  // Yaw控制量限幅（弧度/步，防止甩尾）
    float int_min_yaw = -0.1f, int_max_yaw = 0.1f;


    /*********************************************************************
     * 2. 从ROS参数服务器读取参数（覆盖默认值，支持动态配置）
     * 参数路径格式：/apoc/pid/[轴名]/[参数名]（例如 /apoc/pid/x/kp）
     ********************************************************************/
    // X轴参数读取
    ros::param::get("/apoc/pid/x/kp", kp_x);
    ros::param::get("/apoc/pid/x/ki", ki_x);
    ros::param::get("/apoc/pid/x/kd", kd_x);
    ros::param::get("/apoc/pid/x/out_min", out_min_x);
    ros::param::get("/apoc/pid/x/out_max", out_max_x);
    ros::param::get("/apoc/pid/x/int_min", int_min_x);
    ros::param::get("/apoc/pid/x/int_max", int_max_x);

    // Y轴参数读取
    ros::param::get("/apoc/pid/y/kp", kp_y);
    ros::param::get("/apoc/pid/y/ki", ki_y);
    ros::param::get("/apoc/pid/y/kd", kd_y);
    ros::param::get("/apoc/pid/y/out_min", out_min_y);
    ros::param::get("/apoc/pid/y/out_max", out_max_y);
    ros::param::get("/apoc/pid/y/int_min", int_min_y);
    ros::param::get("/apoc/pid/y/int_max", int_max_y);

    // Z轴参数读取
    ros::param::get("/apoc/pid/z/kp", kp_z);
    ros::param::get("/apoc/pid/z/ki", ki_z);
    ros::param::get("/apoc/pid/z/kd", kd_z);
    ros::param::get("/apoc/pid/z/out_min", out_min_z);
    ros::param::get("/apoc/pid/z/out_max", out_max_z);
    ros::param::get("/apoc/pid/z/int_min", int_min_z);
    ros::param::get("/apoc/pid/z/int_max", int_max_z);

    // Yaw偏航角参数读取
    ros::param::get("/apoc/pid/yaw/kp", kp_yaw);
    ros::param::get("/apoc/pid/yaw/ki", ki_yaw);
    ros::param::get("/apoc/pid/yaw/kd", kd_yaw);
    ros::param::get("/apoc/pid/yaw/out_min", out_min_yaw);
    ros::param::get("/apoc/pid/yaw/out_max", out_max_yaw);
    ros::param::get("/apoc/pid/yaw/int_min", int_min_yaw);
    ros::param::get("/apoc/pid/yaw/int_max", int_max_yaw);


    /*********************************************************************
     * 3. 将参数设置到PID控制器实例
     ********************************************************************/
    // X轴PID参数设置
    pid_x.setPIDctrlParams(kp_x, ki_x, kd_x, out_min_x, out_max_x, int_min_x, int_max_x);
    // Y轴PID参数设置
    pid_y.setPIDctrlParams(kp_y, ki_y, kd_y, out_min_y, out_max_y, int_min_y, int_max_y);
    // Z轴PID参数设置
    pid_z.setPIDctrlParams(kp_z, ki_z, kd_z, out_min_z, out_max_z, int_min_z, int_max_z);
    // Yaw偏航角PID参数设置
    pid_yaw.setPIDctrlParams(kp_yaw, ki_yaw, kd_yaw, out_min_yaw, out_max_yaw, int_min_yaw, int_max_yaw);


    /*********************************************************************
     * 4. 打印初始化结果（便于调试和参数验证）
     ********************************************************************/
    ROS_INFO("=== PID Controllers Initialized ===");
    ROS_INFO("X轴PID: kp=%.2f, ki=%.2f, kd=%.2f | out=[%.2f, %.2f] | int=[%.2f, %.2f]",
             kp_x, ki_x, kd_x, out_min_x, out_max_x, int_min_x, int_max_x);
    ROS_INFO("Y轴PID: kp=%.2f, ki=%.2f, kd=%.2f | out=[%.2f, %.2f] | int=[%.2f, %.2f]",
             kp_y, ki_y, kd_y, out_min_y, out_max_y, int_min_y, int_max_y);
    ROS_INFO("Z轴PID: kp=%.2f, ki=%.2f, kd=%.2f | out=[%.2f, %.2f] | int=[%.2f, %.2f]",
             kp_z, ki_z, kd_z, out_min_z, out_max_z, int_min_z, int_max_z);
    ROS_INFO("YawPID: kp=%.2f, ki=%.2f, kd=%.2f | out=[%.2f, %.2f] | int=[%.2f, %.2f]",
             kp_yaw, ki_yaw, kd_yaw, out_min_yaw, out_max_yaw, int_min_yaw, int_max_yaw);
    ROS_INFO("===================================");
}