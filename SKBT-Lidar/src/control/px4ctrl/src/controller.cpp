#include "controller.h"
#include <algorithm>
#include <cmath>

using namespace std;

     

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d ctrl_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
      ctrl_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
      if (param_.nmpc.enable) {
        ctrl_acc = solveNmpcAcceleration(des, odom);
      }
      des_acc = ctrl_acc + Eigen::Vector3d(0,0,param_.gra);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(odom.q);
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
      // yaw = fromQuaternion2yaw(des.q);
      yaw_imu = fromQuaternion2yaw(imu.q);
      // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
      //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
      //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      u.q = imu.q * odom.q.inverse() * q;


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

double LinearControl::clampValue(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

Eigen::Vector3d LinearControl::getReferencePosition(const Desired_State_t &des, double t) const
{
  return des.p + des.v * t + 0.5 * des.a * t * t + des.j * t * t * t / 6.0;
}

Eigen::Vector3d LinearControl::getReferenceVelocity(const Desired_State_t &des, double t) const
{
  return des.v + des.a * t + 0.5 * des.j * t * t;
}

Eigen::Vector3d LinearControl::projectControlAcceleration(const Eigen::Vector3d &acc) const
{
  Eigen::Vector3d projected = acc;
  const double max_acc = std::max(0.1, param_.nmpc.max_acc);
  if (projected.norm() > max_acc) {
    projected = projected.normalized() * max_acc;
  }

  Eigen::Vector3d total_acc = projected + Eigen::Vector3d(0.0, 0.0, param_.gra);
  const double min_total_z = std::max(0.0, param_.nmpc.min_thrust) * thr2acc_;
  const double max_total_z = std::max(param_.nmpc.min_thrust, param_.nmpc.max_thrust) * thr2acc_;
  total_acc.z() = clampValue(total_acc.z(), min_total_z, max_total_z);

  if (param_.max_angle > 0.0) {
    const double max_horizontal = std::tan(param_.max_angle) * std::max(total_acc.z(), 1e-3);
    const double horizontal_norm = total_acc.head<2>().norm();
    if (horizontal_norm > max_horizontal) {
      total_acc.head<2>() *= max_horizontal / horizontal_norm;
    }
  }

  return total_acc - Eigen::Vector3d(0.0, 0.0, param_.gra);
}

Eigen::Vector3d LinearControl::solveNmpcAcceleration(const Desired_State_t &des, const Odom_Data_t &odom)
{
  Eigen::Vector3d Kp,Kv;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;

  const Eigen::Vector3d feedback_seed =
      des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

  const int horizon = std::max(1, std::min(param_.nmpc.horizon, 40));
  const int gradient_steps = std::max(1, std::min(param_.nmpc.gradient_steps, 20));
  const double dt = clampValue(param_.nmpc.dt, 0.005, 0.10);
  const double alpha = clampValue(param_.nmpc.alpha, 1e-4, 0.5);

  std::vector<Eigen::Vector3d> acc_seq(horizon, Eigen::Vector3d::Zero());
  const Eigen::Vector3d warm_start = nmpc_warm_started_ ? last_nmpc_acc_ : feedback_seed;
  for (int k = 0; k < horizon; ++k) {
    acc_seq[k] = projectControlAcceleration(warm_start + des.j * (k * dt));
  }

  for (int iter = 0; iter < gradient_steps; ++iter) {
    std::vector<Eigen::Vector3d> pred_p(horizon + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> pred_v(horizon + 1, Eigen::Vector3d::Zero());
    pred_p[0] = odom.p;
    pred_v[0] = odom.v;

    for (int k = 0; k < horizon; ++k) {
      pred_p[k + 1] = pred_p[k] + pred_v[k] * dt + 0.5 * acc_seq[k] * dt * dt;
      pred_v[k + 1] = pred_v[k] + acc_seq[k] * dt;
    }

    std::vector<Eigen::Vector3d> grad(horizon, Eigen::Vector3d::Zero());
    Eigen::Vector3d lambda_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d lambda_v = Eigen::Vector3d::Zero();

    for (int k = horizon - 1; k >= 0; --k) {
      const double t_next = (k + 1) * dt;
      lambda_p += 2.0 * param_.nmpc.q_pos * (pred_p[k + 1] - getReferencePosition(des, t_next));
      lambda_v += 2.0 * param_.nmpc.q_vel * (pred_v[k + 1] - getReferenceVelocity(des, t_next));

      const Eigen::Vector3d ref_acc = des.a + des.j * (k * dt);
      grad[k] += 2.0 * param_.nmpc.r_acc * (acc_seq[k] - ref_acc);

      const Eigen::Vector3d prev_acc = (k == 0) ? (nmpc_warm_started_ ? last_nmpc_acc_ : feedback_seed) : acc_seq[k - 1];
      grad[k] += 2.0 * param_.nmpc.r_delta_acc * (acc_seq[k] - prev_acc);
      if (k < horizon - 1) {
        grad[k] -= 2.0 * param_.nmpc.r_delta_acc * (acc_seq[k + 1] - acc_seq[k]);
      }

      grad[k] += 0.5 * dt * dt * lambda_p + dt * lambda_v;
      lambda_v += dt * lambda_p;
    }

    for (int k = 0; k < horizon; ++k) {
      acc_seq[k] = projectControlAcceleration(acc_seq[k] - alpha * grad[k]);
    }
  }

  last_nmpc_acc_ = acc_seq.front();
  nmpc_warm_started_ = true;
  return last_nmpc_acc_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
  nmpc_warm_started_ = false;
  last_nmpc_acc_.setZero();
}





