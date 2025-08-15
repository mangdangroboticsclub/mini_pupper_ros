#ifndef IMU_EKF_NODE_HPP_
#define IMU_EKF_NODE_HPP_

// SPDX-License-Identifier: Apache-2.0
//
// Copyright (c) 2025 MangDang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class ImuEkfNode : public rclcpp::Node {
    public:
    ImuEkfNode();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    // ekf timing
    rclcpp::TimerBase::SharedPtr ekf_timer_;
    static constexpr int EkfPeriodMs = 20;
    void ekf_loop_ ();

    // Keep ROS time if you also use it elsewhere (e.g., stamps):
    rclcpp::Time last_ekf_time_;

    // steady clock + last steady tick (monotonic)
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time  last_ekf_time_steady_;

    // msg pointers
    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    geometry_msgs::msg::Twist::SharedPtr last_twist_;

    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscription_;
    void imu_data_callback_ (sensor_msgs::msg::Imu::ConstSharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    void cmd_vel_callback_ (geometry_msgs::msg::Twist::ConstSharedPtr msg);

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_publisher_;

    // ekf
    Eigen::Matrix4d X_ = Eigen::Matrix4d::Identity(); // pose (world←state)
    Matrix6d P_; // covariance
    Matrix6d Q_; // process noise

    Eigen::Matrix2d r_accel_; // meas noise (roll,pitch)
    Eigen::Matrix3d r_slam_; // meas noise (x,y,yaw)

    Eigen::Matrix<double, 2, 6> H_accel_;
    Eigen::Matrix<double, 3, 6> H_slam_;

    void predict_ (const double dt, const Vector6d& u);
    void update_accel_ (const Eigen::Vector3d& accel);
    void ekf_publish_ (const rclcpp::Time &stamp);
    void ekf_log_ ();

    // SE3 tools
    Eigen::Matrix3d skew_ (Eigen::Vector3d w);
    Eigen::Vector3d unskew_ (Eigen::Matrix3d W);
    Eigen::Matrix3d so3_exp_ (Eigen::Vector3d w);
    Eigen::Vector3d so3_log_ (Eigen::Matrix3d R);
    Eigen::Matrix3d so3_left_jacobian_ (Eigen::Vector3d w);
    Eigen::Matrix3d so3_left_jacobian_inv_ (Eigen::Vector3d w);
    Eigen::Matrix4d se3_exp_ (Vector6d xi);
    Vector6d se3_log_ (Eigen::Matrix4d T);
};

#endif