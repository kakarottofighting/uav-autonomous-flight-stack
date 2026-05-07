#ifndef _MINIMUMSNAP_HPP_
#define _MINIMUMSNAP_HPP_

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <visual_utils/planning_visualization.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <chrono>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <yaml-cpp/yaml.h>

using namespace Eigen;
struct MiniSnapTraj{
    ros::Time       start_time;         //轨迹起始时间
    int             traj_nums;          //分段轨迹数量
    double          time_duration;      //轨迹总时间
    Eigen::VectorXd poly_time;          //每段轨迹的结束时间点，相对轨迹起始点来说
    Eigen::MatrixXd poly_coeff;         //多项式轨迹系数矩阵
    MiniSnapTraj(){};
    MiniSnapTraj(ros::Time _start_time, Eigen::VectorXd _poly_time, Eigen::MatrixXd _poly_coeff){
        start_time  = _start_time;
        traj_nums   = _poly_time.size();
        poly_time.resize(traj_nums);
        double sum = 0;
        for (int i = 0; i < _poly_time.size(); i++){
            sum += _poly_time(i);
            poly_time[i] = sum;
        }
        poly_coeff  = _poly_coeff;
        for (int i = 0; i < _poly_time.size(); i++)  time_duration += _poly_time(i);
    };
    ~MiniSnapTraj(){};
};

struct Traj_info{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Traj_info(Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a)
    : position(p), velocity(v), acceleration(a) {}
};



class TRAJECTORY_GENERATOR  
{
private:

    double _vis_traj_width;
    double _Vel, _Acc;
    int _dev_order, _min_order;
    int _poly_num1D;
    

    Vector3d _startPos  = Vector3d::Zero();
    Vector3d _startVel  = Vector3d::Zero();
    Vector3d _startAcc  = Vector3d::Zero();
    Vector3d _endVel    = Vector3d::Zero();
    Vector3d _endAcc    = Vector3d::Zero();

    std::vector<Eigen::Vector3d> traj;
    // double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _Ct;
    
    Eigen::VectorXd _Px, _Py, _Pz;

    Eigen::MatrixXd getQ(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getM(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num, const int d_order);

    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                         const Eigen::MatrixXd &M,
                                         const Eigen::MatrixXd &Ct,
                                         const Eigen::VectorXd &WayPoints1D,
                                         const Eigen::VectorXd &StartState1D,
                                         const Eigen::VectorXd &EndState1D,
                                         const int seg_num, 
                                         const int d_order);

public:
    YAML::Node config;
    TRAJECTORY_GENERATOR(){
        config = YAML::LoadFile("/home/skbt/SKBT_Drone/src/planner/minimumsnap/config/minimumsnap_Para.yaml");
        _Vel = config["Vel"].as<double>();
        _Acc = config["Acc"].as<double>();
        _dev_order = config["dev_order"].as<int>();
        _min_order = config["min_order"].as<int>();
        _vis_traj_width = config["vis_traj_width"].as<double>();
        _poly_num1D = 2 * _dev_order;
    };

    ~TRAJECTORY_GENERATOR(){};

    Eigen::MatrixXd PolyQPGeneration(
        const int order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time);

    int Factorial(int x);

    Eigen::VectorXd timeAllocation(Eigen::MatrixXd& Path, double vel = -1);
    Traj_info getTrajInfo(const MiniSnapTraj& traj, double time);
    MiniSnapTraj trajGeneration(Eigen::MatrixXd& path, ros::Time Time_start, double vel_desire = -1, Eigen::Vector3d startVel = Vector3d::Zero());
   
};

#endif // _MINIMUMSNAP_HPP_