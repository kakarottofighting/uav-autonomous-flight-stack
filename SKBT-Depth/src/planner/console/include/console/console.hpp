#ifndef _CONSOLE_HPP_
#define _CONSOLE_HPP_

#include <tf/tf.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <limits>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <visual_utils/planning_visualization.hpp>
#include <tf/transform_broadcaster.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

//自定义的控制台状态消息
#include <console/ConsoleState.h>
#include "odom_fusion/OdomStamp.h"

#include "Minimumsnap.hpp"
#include "odom_fusion/OdomStamp.h"

#include "CSF_Lidar.h"
#include <livox_ros_driver2/CustomMsg.h>


using namespace std;
using namespace Eigen;




enum Main_STATE{             
    LEVEL1 = 0,             //第n关
    LEVEL2,
    LEVEL3,
    LEVEL4,
    LEVEL5,
    LANDING,                 //降落
    INIT,                   //初始状态
    WAIT_TAKE_OFF_FINISHED //等待起飞完成
};

enum Sub_STATE{             
    Stage_1 = 0,
    Stage_2,
    Stage_3,
    Stage_4,
    Stage_5,
    Stage_6,
    Stage_7,
    Stage_8,
    Stage_0
};



class CONSOLE{
private:
    // 轨迹规划器
    TRAJECTORY_GENERATOR TG;
    C_S_FILTER_LIDAR csf;

    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud;

    sensor_msgs::PointCloud2 Cloud_msg;
    
    double TF_height;
    
    bool rviz_delay; //是否按照实际的时间节奏去可视化轨迹

    // 圆相关
    Eigen::Vector3d circle1_center    = Vector3d::Zero();
    Eigen::Vector3d circle2_center    = Vector3d::Zero();
    double Circle_height;
    double circle2_height;
    double circle2_startvel;

    // 隧道相关
    Eigen::Vector3d suidao_center    = Vector3d::Zero();
    double suidao_angle;
    double Suidao_height;

    int stagenum1, stagenum2, stagenum3, stagenum4, stagenum5;

    // 迷宫相关
    double door_height;
    double window_height;

    // 用于存储每一关是否被启用
    std::vector<bool> levelenable_list;

    // 每一关的阶段数
    std::vector<int> stagenum_list;

    // 每一关的每一阶段的规划结束标志和飞行结束标志
    
    std::vector<std::vector<bool>> planfinishflag_list;
    std::vector<std::vector<bool>> flyfinishflag_list;

    bool task_begin = false;
    
    //每一关的起点
    Eigen::Vector3d level1_start = Vector3d::Zero();
    Eigen::Vector3d level2_start = Vector3d::Zero();
    Eigen::Vector3d level3_start = Vector3d::Zero();
    Eigen::Vector3d level4_start = Vector3d::Zero();
    Eigen::Vector3d level5_start = Vector3d::Zero();
    std::vector<Eigen::Vector3d> start_list;

    //每一关的yaw角
    double yaw1, yaw2, yaw3, yaw4, yaw5;
    std::vector<double> yaw_list;

    double fly_altitude;
    
    ros::Timer ControlCmdCallback_Timer;
    ros::Timer Transformer_Timer;
    ros::Timer FSM_Timer;
    tf::TransformBroadcaster broadcaster;

    /*状态机当前状态*/
    Main_STATE main_state = INIT;
    Sub_STATE sub_state = Stage_0;



    //当前跟踪的轨迹
    MiniSnapTraj Cur_Traj;
    //轨迹开始时间初始化标志
    bool Traj_time_init = false;
    //每一条轨迹的开始时间
    ros::Time tra_start_time;

    //存储当前阶段路标点
    std::vector<Eigen::Vector3d> Waypoints_list;
    Eigen::MatrixXd Waypoints_matrix;
    // 调整yaw角是否完成,初始化成true表示初始不需要调整yaw角
    bool rotate_finished = true;
    double desired_yaw = 0.0;

    bool traj_rotate = false;

    //Apriltag相关
    Eigen::Matrix3d CtB_R; //相机系到机体系的姿态
    Eigen::Vector3d CtB_t;
    
    std::vector<Eigen::Vector3d> tag_to_center_list; //存储每个码相对于降落点中心的位置偏置
    bool apriltag_detected = false;
    std::vector<Eigen::Vector3d> CenterCalbyTag_list;

    Eigen::Vector3d landing_center;



    PlanningVisualization::Ptr trajVisual_;
    nav_msgs::Path trajPath;
    ros::Publisher trajPath_pub;
    ros::Publisher controlCmd_pub;
    ros::Publisher vel_cmd_pub;
    ros::Publisher land_pub;
    ros::Publisher pose_pub;
    ros::Publisher cloud_pub;
    
    ros::Subscriber odom_sub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber apriltag_sub;

    Eigen::Quaterniond odom_q;
    Eigen::Vector3d odom_pos, odom_vel;
    double vins_altitude;

    bool has_odom;
    
    quadrotor_msgs::PositionCommand posiCmd;

    double last_yaw_, last_yaw_dot_;
    
    double landing_height;
    bool is_land;


    
    void waypointsVisual(const Eigen::MatrixXd& path);
    std::pair<double, double> calculate_yaw(Traj_info& t_info);
    double clamp(double value, double min, double max);
    
    void ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg);
    void ControlCmdCallback(const ros::TimerEvent &e);
    void OdomCallback(const odom_fusion::OdomStamp::ConstPtr& msg);
    void LidarCallback(const livox_ros_driver2::CustomMsgConstPtr& custom_msg);

    /*TF树坐标变换关系*/
    void TransformerCallback(const ros::TimerEvent &e);
    /*控制状态机*/
    void ControlFSM(const ros::TimerEvent &e);
    /*根据轨迹解析控制指令*/
    bool ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool isAdjustYaw);

    void getVisual(MiniSnapTraj& trajectory, MatrixXd wps);

    bool Find_circle();
    double getYawfromQuaternion(Eigen::Quaterniond& q);
    bool Find_suidao();
    bool Find_window(int k = 1, double dis = 6);

    void delay(int milliseconds);

    void nextLevel(Main_STATE& m);
    void nextStage(Sub_STATE& m);

    Eigen::MatrixXd vectorToMatrix(const std::vector<Eigen::Vector3d>& vec);
    Eigen::Vector3d start_forward(double vel);
    double Get_right_distance();
    double Get_left_distance();
    double Correction_z(double z);
    int Find_trees();

    Eigen::Vector3d body_to_world(Eigen::Vector3d& p_b);
    

    
public:
    
    
    CONSOLE(/* argvs */){
        TF_height = 0.0;
        has_odom = false;
        is_land = false;

        Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        odom_q.x() = 0.0;
        odom_q.y() = 0.0;
        odom_q.z() = 0.0;
        odom_q.w() = 1.0;

        odom_pos.x() = 0.0;
        odom_pos.y() = 0.0;
        odom_pos.z() = 0.0;

        suidao_angle = 0.0;
        CtB_R << 0, -1,  0,
                -1,  0,  0,
                 0,  0, -1;
        CtB_t << 0.067, 0, -0.16;
    }
    ~CONSOLE(){

    }
    void init(ros::NodeHandle& nh);
};






#endif




