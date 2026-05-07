#include <iostream>
#include <chrono>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "odom_fusion/OdomStamp.h"


using namespace Eigen;



class C_S_FILTER{
public:
    int width_ori;       //原始深度图的宽和高
    int height_ori;
    cv::Mat CV_Depth_ori;
    
    int width;           //稀疏后的宽和高
    int height;

    double m;            //布料节点质量
    double dt;           //时间步长

    double Kt;           //Traction劲度系数
    double Ks;           //Shear劲度系数
    double Kf;           //Flexion劲度系数

    int Kspars;          //稀疏系数

    int Cnt_Area_Threshold;

    double Kerase;       //地面擦除系数

    double thresh;

    int MaxIterations;

    Eigen::Matrix3d  Intri_Matrix;           //相机内参
    Eigen::Matrix3d  Intri_Matrix_Inverse;   //内参的逆

    Eigen::Matrix3d Rotation_Matrix_CtoB;  //相机系相对于Body系的旋转矩阵
    Eigen::Quaterniond q_CtoB;             //相机系相对于Body系的四元数

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Cloth0;    //用于备份Depth_Cloth以进行帧间比较
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Vel_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Image; 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Coll_Distance;    //存储布和每个实际深度的距离，用于碰撞检测
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Coll_Flag;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UtD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DtU_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LtR_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtL_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LUtRD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RUtLD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RDtLU_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LDtRU_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UtD2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DtU2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LtR2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtL2_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_Traction;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LU_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RU_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RD_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LD_Shear;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R2_Flexion;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Internal_Force;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> External_Force;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Force_Matrix;


    cv::Mat Barrier_Mask;
    cv::Mat Barrier_Mask_OriSize;
    cv::Mat Access_Mask;
    cv::Mat Hole_Mask;
    cv::Mat CV_Depth;
    cv::Mat CV_Cloth;
    cv::Mat Safety_Filter; //用于擦掉上下边缘区域

    cv::Size New_Size;
    cv::Size Ori_Size;
    
    std::vector<std::vector<cv::Point>> Hole_Cnts;
    std::vector<cv::Point2f> Hole_Centers;
    std::vector<Eigen::Vector3d> Hole_3Dpos;

    std::vector<std::vector<cv::Point>> Barrier_Cnts;
    std::vector<cv::Point2f> Barrier_Centers;
    std::vector<Eigen::Vector3d> Barrier_3Dpos;

    void Input_Depth(cv::Mat& Depth);
    void Reset_Cloth();
    void Cal_force();
    void Collision_Detection();
    void Update_Cloth();
    void Shot();
    void Barrier_Check();
    void Clear_BG();
    void Find_Hole(const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());
    void Find_Barrier(const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());
    void Show_Result(char X);
    //Eigen::Vector3d Get_3Dpos_Body(cv::Point2f& pix_point, double& depth);
    Eigen::Vector3d Get_3Dpos(cv::Point2f& pix_point, double& depth, const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());

    
    
    C_S_FILTER(int W, int H): width_ori(W), height_ori(H) {
        m = 0.004;
        Kt = 2.0;
        Ks = 0.5;
        Kf = 0.01;
        Kspars = 4;
        Kerase = 0.2;
        dt = 0.01; //调参步骤：初始步长-》平衡位置-》受力平衡
        MaxIterations = 50;
        thresh = 1e-6;

        Intri_Matrix << 516.7852783203125, 0.0, 320.07366943359375,
                        0.0, 516.7852783203125, 239.31646728515625,
                        0.0, 0.0, 1.0;
        Intri_Matrix_Inverse = Intri_Matrix.inverse();

        Rotation_Matrix_CtoB << 0, 0, 1,
                                -1, 0, 0,
                                0, -1, 0;

        Eigen::Quaterniond q(Rotation_Matrix_CtoB);
        q_CtoB = q;

        Cnt_Area_Threshold = int((width_ori / 8) * (height_ori / 8)) ;

        Ori_Size.width = W;
        Ori_Size.height = H;

        width = int(W / Kspars);
        height = int(H / Kspars);
        New_Size.width = width;
        New_Size.height = height;

        Depth_Cloth.resize(height, width);
        Depth_Cloth0.resize(height, width);
        Vel_Cloth.resize(height, width);

        Depth_Image.resize(height, width);
        Coll_Distance.resize(height, width);
        Coll_Flag.resize(height, width);
        
        UtD_Cloth.resize(height, width);
        DtU_Cloth.resize(height, width);
        LtR_Cloth.resize(height, width);
        RtL_Cloth.resize(height, width);

        LUtRD_Cloth.resize(height, width);
        RUtLD_Cloth.resize(height, width);
        RDtLU_Cloth.resize(height, width);
        LDtRU_Cloth.resize(height, width);

        UtD2_Cloth.resize(height, width);
        DtU2_Cloth.resize(height, width);
        LtR2_Cloth.resize(height, width);
        RtL2_Cloth.resize(height, width);

        U_Traction.resize(height, width);
        D_Traction.resize(height, width);
        L_Traction.resize(height, width);
        R_Traction.resize(height, width);

        LU_Shear.resize(height, width);
        RU_Shear.resize(height, width);
        RD_Shear.resize(height, width);
        LD_Shear.resize(height, width);

        U2_Flexion.resize(height, width);
        D2_Flexion.resize(height, width);
        L2_Flexion.resize(height, width);
        R2_Flexion.resize(height, width);

        Internal_Force.resize(height, width);
        External_Force.resize(height, width);
        Force_Matrix.resize(height, width);

        External_Force.fill(9.8);
        External_Force = m * External_Force;

        Barrier_Mask.create(height, width, CV_8UC1);
        Barrier_Mask_OriSize.create(H, W, CV_8UC1);
        Access_Mask.create(height_ori, width_ori, CV_8UC1);
        Hole_Mask.create(height_ori, width_ori, CV_8UC1);
        CV_Cloth.create(height_ori, width_ori, CV_8UC3);
        

        int borderSize = int(height_ori * Kerase);
        cv::Rect centerRect(0, 0, W, H - borderSize);
        Safety_Filter = cv::Mat::zeros(H, W, CV_8UC1);
        // 在黑色图像中绘制白色的中心区域
        cv::rectangle(Safety_Filter, centerRect, cv::Scalar(255), -1); // -1 表示填充矩形

    }
      
};

