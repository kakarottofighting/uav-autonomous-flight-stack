#include "CSF.h"



void C_S_FILTER::Input_Depth(cv::Mat& Depth){
    CV_Depth_ori = Depth;
    
    cv::resize(Depth, CV_Depth, New_Size, 0, 0, cv::INTER_NEAREST);
    
    int startRow = height - height * Kerase;

    
    // 使用Eigen的Map将cv::Mat数据直接映射为Eigen::MatrixXf
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat_map(
        CV_Depth.ptr<float>(), CV_Depth.rows, CV_Depth.cols);

    // 将Eigen::MatrixXf转换为Eigen::MatrixXd
    Depth_Image = mat_map.cast<double>();
    // Set the bottom fifth of the matrix to 100.0
    Depth_Image.block(startRow, 0, int(height * Kerase), width) = Eigen::MatrixXd::Constant(int(height * Kerase), width, 100.0);
}

void C_S_FILTER::Reset_Cloth(){
    double Min_depth = Depth_Image.minCoeff();
    Depth_Cloth.fill(Min_depth);
    Vel_Cloth.fill(1.0);
    Coll_Flag.fill(1.0);

    U_Traction.fill(0.0);
    D_Traction.fill(0.0);
    L_Traction.fill(0.0);
    R_Traction.fill(0.0);

    LU_Shear.fill(0.0);
    RU_Shear.fill(0.0);
    RD_Shear.fill(0.0);
    LD_Shear.fill(0.0);

    U2_Flexion.fill(0.0);
    D2_Flexion.fill(0.0);
    L2_Flexion.fill(0.0);
    R2_Flexion.fill(0.0);

    CV_Cloth.setTo(cv::Scalar(0, 0, 0));

    Hole_Centers.clear();
    Barrier_Centers.clear();

    Hole_3Dpos.clear();
    Barrier_3Dpos.clear();
    

}


void C_S_FILTER::Cal_force(){

    // array方法计算内力，提高实时性
    U_Traction.fill(0.0);
    D_Traction.fill(0.0);
    L_Traction.fill(0.0);
    R_Traction.fill(0.0);

    LU_Shear.fill(0.0);
    RU_Shear.fill(0.0);
    RD_Shear.fill(0.0);
    LD_Shear.fill(0.0);

    U2_Flexion.fill(0.0);
    D2_Flexion.fill(0.0);
    L2_Flexion.fill(0.0);
    R2_Flexion.fill(0.0);


    // Use array operations for efficiency
    Eigen::ArrayXXd Depth_Cloth_array = Depth_Cloth.array();

    // Up Traction
    U_Traction.block(1, 0, height - 1, width) = (Kt * (Depth_Cloth_array.block(0, 0, height - 1, width) - Depth_Cloth_array.block(1, 0, height - 1, width))).matrix();

    // Down Traction
    D_Traction.block(0, 0, height - 1, width) = (Kt * (Depth_Cloth_array.block(1, 0, height - 1, width) - Depth_Cloth_array.block(0, 0, height - 1, width))).matrix();

    // Left Traction
    L_Traction.block(0, 1, height, width - 1) = (Kt * (Depth_Cloth_array.block(0, 0, height, width - 1) - Depth_Cloth_array.block(0, 1, height, width - 1))).matrix();

    // Right Traction
    R_Traction.block(0, 0, height, width - 1) = (Kt * (Depth_Cloth_array.block(0, 1, height, width - 1) - Depth_Cloth_array.block(0, 0, height, width - 1))).matrix();
    
    // LUtRD_Shear
    LU_Shear.block(1, 1, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(0, 0, height - 1, width - 1) - Depth_Cloth_array.block(1, 1, height - 1, width - 1))).matrix();

    // RUtLD_Shear
    RU_Shear.block(1, 0, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(0, 1, height - 1, width - 1) - Depth_Cloth_array.block(1, 0, height - 1, width - 1))).matrix();

    // RDtLU_Shear
    RD_Shear.block(0, 0, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(1, 1, height - 1, width - 1) - Depth_Cloth_array.block(0, 0, height - 1, width - 1))).matrix();

    // LDtRU_Shear
    LD_Shear.block(0, 1, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(1, 0, height - 1, width - 1) - Depth_Cloth_array.block(0, 1, height - 1, width - 1))).matrix();

    // U2_Flexion
    U2_Flexion.block(2, 0, height - 2, width) = (Kf * (Depth_Cloth_array.block(0, 0, height - 2, width) - Depth_Cloth_array.block(2, 0, height - 2, width))).matrix();

    // D2_Flexion
    D2_Flexion.block(0, 0, height - 2, width) = (Kf * (Depth_Cloth_array.block(2, 0, height - 2, width) - Depth_Cloth_array.block(0, 0, height - 2, width))).matrix();

    // L2_Flexion
    L2_Flexion.block(0, 2, height, width - 2) = (Kf * (Depth_Cloth_array.block(0, 0, height, width - 2) - Depth_Cloth_array.block(0, 2, height, width - 2))).matrix();

    // R2_Flexion
    R2_Flexion.block(0, 0, height, width - 2) = (Kf * (Depth_Cloth_array.block(0, 2, height, width - 2) - Depth_Cloth_array.block(0, 0, height, width - 2))).matrix();

    // // Up_traction
    // U_Traction = Depth_Cloth;
    // U_Traction.block(0, 0, height - 1, width) = Depth_Cloth.block(1, 0, height - 1, width);
    // U_Traction = Kt * (U_Traction - Depth_Cloth);

    // // Down_traction
    // D_Traction = Depth_Cloth;
    // D_Traction.block(1, 0, height - 1, width) = Depth_Cloth.block(0, 0, height - 1, width);
    // D_Traction = Kt * (D_Traction - Depth_Cloth);

    // // Left_traction
    // L_Traction = Depth_Cloth;
    // L_Traction.block(0, 0, height, width - 1) = Depth_Cloth.block(0, 1, height, width - 1);
    // L_Traction = Kt * (L_Traction - Depth_Cloth);

    // // Right_traction
    // R_Traction = Depth_Cloth;
    // R_Traction.block(0, 1, height, width - 1) = Depth_Cloth.block(0, 0, height, width - 1);
    // R_Traction = Kt * (R_Traction - Depth_Cloth);

    // // LUtRD_Shear
    // LU_Shear = Depth_Cloth;
    // LU_Shear.block(1, 1, height - 1, width - 1) = Depth_Cloth.block(0, 0, height - 1, width - 1);
    // LU_Shear = Ks * (LU_Shear - Depth_Cloth);

    // // RUtLD_Shear
    // RU_Shear = Depth_Cloth;
    // RU_Shear.block(1, 0, height - 1, width - 1) = Depth_Cloth.block(0, 1, height - 1, width - 1);
    // RU_Shear = Ks * (RU_Shear - Depth_Cloth);

    // // RDtLU_Shear
    // RD_Shear = Depth_Cloth;
    // RD_Shear.block(0, 1, height - 1, width - 1) = Depth_Cloth.block(1, 0, height - 1, width - 1);
    // RD_Shear = Ks * (RD_Shear - Depth_Cloth);

    // // LDtRU_Shear
    // LD_Shear = Depth_Cloth;
    // LD_Shear.block(0, 0, height - 1, width - 1) = Depth_Cloth.block(1, 1, height - 1, width - 1);
    // LD_Shear = Ks * (LD_Shear - Depth_Cloth);

    // // U2_Flexion
    // U2_Flexion = Depth_Cloth;
    // U2_Flexion.block(0, 0, height - 2, width) = Depth_Cloth.block(2, 0, height - 2, width);
    // U2_Flexion = Kf * (U2_Flexion - Depth_Cloth);

    // // D2_Flexion
    // D2_Flexion = Depth_Cloth;
    // D2_Flexion.block(2, 0, height - 2, width) = Depth_Cloth.block(0, 0, height - 2, width);
    // D2_Flexion = Kf * (D2_Flexion - Depth_Cloth);

    // // L2_Flexion
    // L2_Flexion = Depth_Cloth;
    // L2_Flexion.block(0, 0, height, width - 2) = Depth_Cloth.block(0, 2, height, width - 2);
    // L2_Flexion = Kf * (L2_Flexion - Depth_Cloth);

    // // R2_Flexion
    // R2_Flexion = Depth_Cloth;
    // R2_Flexion.block(0, 2, height, width - 2) = Depth_Cloth.block(0, 0, height, width - 2);
    // R2_Flexion = Kf * (R2_Flexion - Depth_Cloth);

    //Force
    Internal_Force = U_Traction + D_Traction + L_Traction + R_Traction
                 + LU_Shear + RU_Shear + RD_Shear + LD_Shear
                 + U2_Flexion + D2_Flexion + L2_Flexion + R2_Flexion;

    Force_Matrix = Internal_Force + External_Force;
} 

void C_S_FILTER::Collision_Detection(){
   Coll_Distance = Depth_Image - Depth_Cloth;
   //std::cout << Coll_Distance << std::endl;
//    // 遍历矩阵，更新 Barrier_Mask
//     for (int i = 0; i < Coll_Flag.rows(); ++i) {
//         for (int j = 0; j < Coll_Flag.cols(); ++j) {
//             if (Coll_Distance(i, j) < 0.01) {
//                 Coll_Flag(i, j) = 0.0;
//             } 
//             else{
//                 Coll_Flag(i, j) = 1.0;
//             }
//         }
//     }
   Coll_Flag = (Coll_Distance.array() > 0.01).cast<double>();
   Depth_Cloth = (Coll_Distance.array() < 0.01).select(Depth_Image, Depth_Cloth);
} 


void C_S_FILTER::Update_Cloth(){
    Vel_Cloth = Vel_Cloth + Force_Matrix * dt / m;
    Vel_Cloth = Vel_Cloth.cwiseProduct(Coll_Flag);
    Depth_Cloth = Depth_Cloth + Vel_Cloth * dt;
} 

void C_S_FILTER::Shot(){
    int iteration = 0;
    while (iteration < MaxIterations)
    {
        iteration ++;
        Collision_Detection();
        Cal_force();
        Update_Cloth();
        // std::cout << Vel_Cloth << std::endl;
        if (Vel_Cloth.sum() / Vel_Cloth.size() < thresh){
            // std::cout << iteration << std::endl;
            break;
        }
    }
    // std::cout <<  iteration << std::endl;
} 




void C_S_FILTER::Barrier_Check(){
    // 将 Eigen::MatrixXd 转换为 Eigen::ArrayXXd，以便使用数组操作
    Eigen::ArrayXXd Coll_Flag_Array = Coll_Flag.array();

    // 创建一个 Eigen::ArrayXXd 掩码矩阵，0.0 的位置为 255，1.0 的位置为 0
    Eigen::ArrayXXd Barrier_Mask_Array = (Coll_Flag_Array == 0.0).cast<double>() * 255.0;

    // 使用 Eigen::Map 将 Eigen::ArrayXXd 数据映射到 cv::Mat
    Eigen::Map<Eigen::Matrix<uchar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(Barrier_Mask.data, Barrier_Mask.rows, Barrier_Mask.cols);

    // 将 Barrier_Mask_Array 转换为 uchar 类型并赋值给 map
    map = Barrier_Mask_Array.cast<uchar>();

    cv::resize(Barrier_Mask, Barrier_Mask_OriSize, Ori_Size, 0, 0, cv::INTER_NEAREST);
    cv::bitwise_not(Barrier_Mask_OriSize, Access_Mask);
    cv::bitwise_and(Access_Mask, Safety_Filter, Access_Mask);
    Hole_Mask = Access_Mask.clone();
}

void C_S_FILTER::Clear_BG(){
    for (int y = 0; y < height_ori; ++y) {
        if (Access_Mask.at<uchar>(y, 0) == 255) {
            cv::floodFill(Hole_Mask, cv::Point(0, y), cv::Scalar(0), 0, cv::Scalar(0), cv::Scalar(0), cv::FLOODFILL_FIXED_RANGE);
            break;
        }
    }

}

void C_S_FILTER::Find_Hole(const Eigen::Quaterniond& q, const Eigen::Vector3d& p){
    Clear_BG();
    // 查找轮廓
    cv::findContours(Hole_Mask, Hole_Cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 使用 std::remove_if 和 erase 删除面积小于指定阈值的轮廓
    Hole_Cnts.erase(
        std::remove_if(Hole_Cnts.begin(), Hole_Cnts.end(),
            [this](const std::vector<cv::Point>& contour) {
                return cv::contourArea(contour) < this->Cnt_Area_Threshold;
            }),
        Hole_Cnts.end()
    );

    // 遍历每个轮廓
    for (size_t i = 0; i < Hole_Cnts.size(); i++) {
        // 计算轮廓的矩
        cv::Moments mu = cv::moments(Hole_Cnts[i]);

        // 计算质心
        if (mu.m00 != 0) { // 防止除以零
            cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            Hole_Centers.push_back(center);
            Hole_3Dpos.push_back(Get_3Dpos(center, Depth_Cloth(int(center.y / Kspars), int(center.x / Kspars)), q, p));
        }
    }

    
}

void C_S_FILTER::Find_Barrier(const Eigen::Quaterniond& q, const Eigen::Vector3d& p){
    // 查找轮廓
    cv::findContours(Barrier_Mask_OriSize, Barrier_Cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 使用 std::remove_if 和 erase 删除面积小于指定阈值的轮廓
    Barrier_Cnts.erase(
        std::remove_if(Barrier_Cnts.begin(), Barrier_Cnts.end(),
            [this](const std::vector<cv::Point>& contour) {
                return cv::contourArea(contour) < this->Cnt_Area_Threshold;
            }),
        Barrier_Cnts.end()
    );
    // 遍历每个轮廓
    for (size_t i = 0; i < Barrier_Cnts.size(); i++) {
        // 计算轮廓的矩
        cv::Moments mu = cv::moments(Barrier_Cnts[i]);

        // 计算质心
        if (mu.m00 != 0) { // 防止除以零
            cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            Barrier_Centers.push_back(center);
            Barrier_3Dpos.push_back(Get_3Dpos(center, Depth_Cloth(int(center.y / Kspars), int(center.x / Kspars)), q, p));
        }
    }
}


Eigen::Vector3d C_S_FILTER::Get_3Dpos(cv::Point2f& pix_point, double& depth, const Eigen::Quaterniond& q, const Eigen::Vector3d& p){ 
    Eigen::Vector3d UV1(pix_point.x, pix_point.y, 1);
    return q * (q_CtoB * (depth * Intri_Matrix_Inverse * UV1)) + p;
}

void C_S_FILTER::Show_Result(char X){
    

    // 查找Depth_Cloth中的最小值和最大值
    double min_depth = Depth_Cloth.minCoeff();
    double max_depth = Depth_Cloth.maxCoeff();

    // 遍历Coll_Flag并根据条件进行涂色
    for (int i = 0; i < Coll_Flag.rows(); ++i) {
        for (int j = 0; j < Coll_Flag.cols(); ++j) {
            if (Coll_Flag(i, j) == 0.0) {
                // 涂成红色
                CV_Cloth.at<cv::Vec3b>(i * Kspars, j * Kspars) = cv::Vec3b(0, 0, 255);
            } else {
                // 根据Depth_Cloth的值涂成蓝色
                double depth_value = Depth_Cloth(i, j);
                double normalized_value = (depth_value - min_depth) / (max_depth - min_depth);
                int blue_intensity = static_cast<int>(normalized_value * 255);
                CV_Cloth.at<cv::Vec3b>(i* Kspars, j * Kspars) = cv::Vec3b(blue_intensity, 0, 0);
            }
        }
    }

    // 将32FC1图像归一化并转换为8位无符号整型
    cv::Mat depth32FC1_normalized, depth_8U;
    cv::normalize(CV_Depth_ori, depth32FC1_normalized, 0, 255, cv::NORM_MINMAX);
    depth32FC1_normalized.convertTo(depth_8U, CV_8UC1);

    // 将单通道的8位图像转换为三通道
    cv::Mat depth_8U_color, Mask_color;
    cv::cvtColor(depth_8U, depth_8U_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(Access_Mask, Mask_color, cv::COLOR_GRAY2BGR);

    std::string Text;
    cv::Point textOrg;
    if(X == 'H'){
        cv::drawContours(Mask_color, Hole_Cnts, -1, cv::Scalar(0, 255, 0), 2); // 在副本图像上绘制轮廓
        for (size_t i = 0; i < Hole_Centers.size(); i++) {
            cv::circle(Mask_color, Hole_Centers[i], 5, cv::Scalar(0, 0, 255), -1);
            Text = "(" + std::to_string(Hole_3Dpos[i][0]).substr(0, 5)  + ", " + std::to_string(Hole_3Dpos[i][1]).substr(0, 5) + ", " + std::to_string(Hole_3Dpos[i][2]).substr(0, 5) + ")";
            textOrg.x = Hole_Centers[i].x - 100;
            textOrg.y = Hole_Centers[i].y + 40;
            cv::putText(Mask_color, Text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
        }
    }

    else if(X == 'B'){
        cv::drawContours(Mask_color, Barrier_Cnts, -1, cv::Scalar(0, 255, 0), 2); // 在副本图像上绘制轮廓
        for (size_t i = 0; i < Barrier_Centers.size(); i++) {
            cv::circle(Mask_color, Barrier_Centers[i], 5, cv::Scalar(0, 0, 255), -1);
            Text = "(" + std::to_string(Barrier_3Dpos[i][0]).substr(0, 5)  + ", " + std::to_string(Barrier_3Dpos[i][1]).substr(0, 5) + ", " + std::to_string(Barrier_3Dpos[i][2]).substr(0, 5) + ")";
            textOrg.x = Barrier_Centers[i].x - 100;
            textOrg.y = Barrier_Centers[i].y + 40;
            cv::putText(Mask_color, Text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
        }
    }


    // 并列合并两个图像
    cv::Mat combined1;
    cv::Mat combined2;
    cv::hconcat(depth_8U_color, CV_Cloth, combined1);
    cv::hconcat(combined1, Mask_color, combined2);

    // 显示结果
    cv::imshow("Combined Image", combined2);
    cv::waitKey(5);
} 
