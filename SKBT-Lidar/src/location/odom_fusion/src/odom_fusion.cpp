#include <odom_fusion/odom_fusion.h>


void ODOM_FUSION::init(ros::NodeHandle& nh){
    configureUkf(nh);
    odom_fusion_pub = nh.advertise<odom_fusion::OdomStamp>("/fusion/odom", 50);
    odom_sub        = nh.subscribe("odom", 100, &ODOM_FUSION::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    //TF_timer = nh.createTimer(ros::Duration(0.1), std::bind(&ODOM_FUSION::readLaser, this, std::placeholders::_1));
    std::thread TF_thread(&ODOM_FUSION::readLaser, this);
    TF_thread.detach();  // 分离线程
}

void ODOM_FUSION::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    const double filtered_height = Average_filter(altitude);

    odom_withH.pose.header = msg->header;
    odom_withH.pose.header.frame_id = "world";
    odom_withH.pose.pose = msg->pose;
    odom_withH.tf_height = filtered_height;

    if(first_pose){
        if (use_ukf) {
            resetUkf(*msg, filtered_height);
            applyUkfOutput(*msg, filtered_height, 0.0);
        }
        odom_fusion_pub.publish(odom_withH);
        last_pose = odom_withH;
        first_pose = false;
        return;
    }

    // 计算时间间隔
    ros::Duration dt = msg->header.stamp - last_pose.pose.header.stamp;
    double dt_sec = dt.toSec();

    if (use_ukf && dt_sec > 0) {
        applyUkfOutput(*msg, filtered_height, std::min(dt_sec, ukf_max_dt));
        odom_fusion_pub.publish(odom_withH);
        last_pose = odom_withH;
        return;
    }

    if (dt_sec > 0) {
        // 计算线速度
        double vx = (odom_withH.pose.pose.pose.position.x - last_pose.pose.pose.pose.position.x) / dt_sec;
        double vy = (odom_withH.pose.pose.pose.position.y - last_pose.pose.pose.pose.position.y) / dt_sec;
        double vz = (odom_withH.pose.pose.pose.position.z - last_pose.pose.pose.pose.position.z) / dt_sec;
        // double vz = (odom_withH.tf_height - last_pose.tf_height) / dt_sec;

        // 计算角速度
        tf::Quaternion q1, q2;
        tf::quaternionMsgToTF(last_pose.pose.pose.pose.orientation, q1);
        tf::quaternionMsgToTF(odom_withH.pose.pose.pose.orientation, q2);

        tf::Quaternion q_diff = q1.inverse() * q2;
        tf::Matrix3x3 m(q_diff);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double wx = roll / dt_sec;
        double wy = pitch / dt_sec;
        double wz = yaw / dt_sec;
    
        odom_withH.pose.twist.twist.linear.x = vx;
        odom_withH.pose.twist.twist.linear.y = vy;
        odom_withH.pose.twist.twist.linear.z = vz;

        odom_withH.pose.twist.twist.angular.x = wx;
        odom_withH.pose.twist.twist.angular.y = wy;
        odom_withH.pose.twist.twist.angular.z = wz;

    }
    odom_fusion_pub.publish(odom_withH);
    last_pose = odom_withH;

}

void ODOM_FUSION::configureUkf(const ros::NodeHandle& nh){
    nh.param("use_ukf", use_ukf, false);
    nh.param("ukf/alpha", ukf_alpha, ukf_alpha);
    nh.param("ukf/beta", ukf_beta, ukf_beta);
    nh.param("ukf/kappa", ukf_kappa, ukf_kappa);
    nh.param("ukf/process_pos_noise", ukf_process_pos_noise, ukf_process_pos_noise);
    nh.param("ukf/process_vel_noise", ukf_process_vel_noise, ukf_process_vel_noise);
    nh.param("ukf/odom_pos_noise", ukf_odom_pos_noise, ukf_odom_pos_noise);
    nh.param("ukf/height_noise", ukf_height_noise, ukf_height_noise);
    nh.param("ukf/max_dt", ukf_max_dt, ukf_max_dt);

    if (use_ukf) {
        ROS_INFO("[odom_fusion] UKF fusion enabled.");
    }
}

void ODOM_FUSION::resetUkf(const nav_msgs::Odometry& odom, double height){
    ukf_x.setZero();
    ukf_x(0) = odom.pose.pose.position.x;
    ukf_x(1) = odom.pose.pose.position.y;
    ukf_x(2) = have_altitude ? height : odom.pose.pose.position.z;
    ukf_x(3) = odom.twist.twist.linear.x;
    ukf_x(4) = odom.twist.twist.linear.y;
    ukf_x(5) = odom.twist.twist.linear.z;

    ukf_P.setIdentity();
    ukf_P.diagonal() << 0.05, 0.05, 0.05, 0.20, 0.20, 0.20;
    ukf_initialized = true;
}

void ODOM_FUSION::computeUkfWeights(std::array<double, 2 * UKF_STATE_DIM + 1>& wm,
                                    std::array<double, 2 * UKF_STATE_DIM + 1>& wc) const{
    const double lambda = ukf_alpha * ukf_alpha * (UKF_STATE_DIM + ukf_kappa) - UKF_STATE_DIM;
    const double denom = UKF_STATE_DIM + lambda;

    wm[0] = lambda / denom;
    wc[0] = wm[0] + (1.0 - ukf_alpha * ukf_alpha + ukf_beta);
    for (int i = 1; i < 2 * UKF_STATE_DIM + 1; ++i) {
        wm[i] = 0.5 / denom;
        wc[i] = wm[i];
    }
}

std::array<ODOM_FUSION::UkfState, 2 * ODOM_FUSION::UKF_STATE_DIM + 1>
ODOM_FUSION::makeSigmaPoints() const{
    std::array<UkfState, 2 * UKF_STATE_DIM + 1> sigma_points;
    const double lambda = ukf_alpha * ukf_alpha * (UKF_STATE_DIM + ukf_kappa) - UKF_STATE_DIM;
    UkfCov scaled_cov = (UKF_STATE_DIM + lambda) * ukf_P;

    Eigen::LLT<UkfCov> llt(scaled_cov);
    if (llt.info() != Eigen::Success) {
        scaled_cov += UkfCov::Identity() * 1e-6;
        llt.compute(scaled_cov);
    }

    const UkfCov sqrt_cov = llt.matrixL();
    sigma_points[0] = ukf_x;
    for (int i = 0; i < UKF_STATE_DIM; ++i) {
        sigma_points[i + 1] = ukf_x + sqrt_cov.col(i);
        sigma_points[i + 1 + UKF_STATE_DIM] = ukf_x - sqrt_cov.col(i);
    }
    return sigma_points;
}

void ODOM_FUSION::predictUkf(double dt){
    if (dt <= 0.0) {
        return;
    }

    auto sigma_points = makeSigmaPoints();
    std::array<double, 2 * UKF_STATE_DIM + 1> wm;
    std::array<double, 2 * UKF_STATE_DIM + 1> wc;
    computeUkfWeights(wm, wc);

    for (auto& point : sigma_points) {
        point(0) += point(3) * dt;
        point(1) += point(4) * dt;
        point(2) += point(5) * dt;
    }

    UkfState x_pred = UkfState::Zero();
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        x_pred += wm[i] * sigma_points[i];
    }

    UkfCov p_pred = UkfCov::Zero();
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        const UkfState dx = sigma_points[i] - x_pred;
        p_pred += wc[i] * dx * dx.transpose();
    }

    UkfCov q = UkfCov::Zero();
    q.diagonal() << ukf_process_pos_noise, ukf_process_pos_noise, ukf_process_pos_noise,
                    ukf_process_vel_noise, ukf_process_vel_noise, ukf_process_vel_noise;
    ukf_x = x_pred;
    ukf_P = p_pred + q * std::max(dt, 1e-3);
}

void ODOM_FUSION::updateUkfPosition(const Eigen::Vector3d& position){
    auto sigma_points = makeSigmaPoints();
    std::array<double, 2 * UKF_STATE_DIM + 1> wm;
    std::array<double, 2 * UKF_STATE_DIM + 1> wc;
    computeUkfWeights(wm, wc);

    Eigen::Vector3d z_pred = Eigen::Vector3d::Zero();
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        z_pred += wm[i] * sigma_points[i].head<3>();
    }

    Eigen::Matrix3d S = Eigen::Matrix3d::Identity() * ukf_odom_pos_noise * ukf_odom_pos_noise;
    Eigen::Matrix<double, UKF_STATE_DIM, 3> C = Eigen::Matrix<double, UKF_STATE_DIM, 3>::Zero();
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        const Eigen::Vector3d dz = sigma_points[i].head<3>() - z_pred;
        const UkfState dx = sigma_points[i] - ukf_x;
        S += wc[i] * dz * dz.transpose();
        C += wc[i] * dx * dz.transpose();
    }

    const Eigen::Matrix<double, UKF_STATE_DIM, 3> K = C * S.inverse();
    ukf_x += K * (position - z_pred);
    ukf_P -= K * S * K.transpose();
    ukf_P = 0.5 * (ukf_P + ukf_P.transpose());
}

void ODOM_FUSION::updateUkfHeight(double height){
    auto sigma_points = makeSigmaPoints();
    std::array<double, 2 * UKF_STATE_DIM + 1> wm;
    std::array<double, 2 * UKF_STATE_DIM + 1> wc;
    computeUkfWeights(wm, wc);

    double z_pred = 0.0;
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        z_pred += wm[i] * sigma_points[i](2);
    }

    double S = ukf_height_noise * ukf_height_noise;
    UkfState C = UkfState::Zero();
    for (int i = 0; i < 2 * UKF_STATE_DIM + 1; ++i) {
        const double dz = sigma_points[i](2) - z_pred;
        const UkfState dx = sigma_points[i] - ukf_x;
        S += wc[i] * dz * dz;
        C += wc[i] * dx * dz;
    }

    const UkfState K = C / S;
    ukf_x += K * (height - z_pred);
    ukf_P -= K * S * K.transpose();
    ukf_P = 0.5 * (ukf_P + ukf_P.transpose());
}

void ODOM_FUSION::applyUkfOutput(const nav_msgs::Odometry& msg, double filtered_height, double dt_sec){
    if (!ukf_initialized) {
        resetUkf(msg, filtered_height);
    }

    predictUkf(dt_sec);
    updateUkfPosition(Eigen::Vector3d(msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z));
    if (have_altitude) {
        updateUkfHeight(filtered_height);
    }

    odom_withH.pose.pose.pose.position.x = ukf_x(0);
    odom_withH.pose.pose.pose.position.y = ukf_x(1);
    odom_withH.pose.pose.pose.position.z = ukf_x(2);
    odom_withH.pose.pose.pose.orientation = msg.pose.pose.orientation;

    odom_withH.pose.twist.twist.linear.x = ukf_x(3);
    odom_withH.pose.twist.twist.linear.y = ukf_x(4);
    odom_withH.pose.twist.twist.linear.z = ukf_x(5);

    if (dt_sec > 0.0 && !first_pose) {
        tf::Quaternion q1, q2;
        tf::quaternionMsgToTF(last_pose.pose.pose.pose.orientation, q1);
        tf::quaternionMsgToTF(odom_withH.pose.pose.pose.orientation, q2);

        tf::Quaternion q_diff = q1.inverse() * q2;
        tf::Matrix3x3 m(q_diff);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        odom_withH.pose.twist.twist.angular.x = roll / dt_sec;
        odom_withH.pose.twist.twist.angular.y = pitch / dt_sec;
        odom_withH.pose.twist.twist.angular.z = yaw / dt_sec;
    }
}



void ODOM_FUSION::readLaser(){
    while (true)
    {
        if(serial_port.available()){
            uint8_t buffer[9];
            size_t bytes_read = serial_port.read(buffer, 9);
            // 检查是否成功读取到完整的数据帧
            if (bytes_read != 9){
                std::cout << "Failed to read complete data frame" <<  std::endl;
            }
            else{
                for (size_t i = 0; i < 9; ++i) {
                    if(buffer[i] == 0x59 && buffer[(i + 1) % 9] == 0x59){
                        // 解析距离值
                        uint16_t distance = (buffer[(i + 3) % 9] << 8) | buffer[(i + 2) % 9];
                        tmp_altitude = static_cast<float>(distance) / 100.0;  // 距离值以米为单位
                        if(tmp_altitude != 16.25){
                            altitude = tmp_altitude;
                            // std::cout << altitude <<  std::endl;
                            have_altitude = true;    
                        }
                    }
                }
            }
        }
    }
} 

double ODOM_FUSION::Average_filter(double tf_data){
  sliding_window.push_back(tf_data);
        SW_sum += tf_data;

        if (sliding_window.size() > SW_len) {
            SW_sum -= sliding_window.front();
            sliding_window.pop_front();
        }

        return SW_sum / sliding_window.size();
}
