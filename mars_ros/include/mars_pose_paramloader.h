// Copyright (C) 2024 Christian Brommer and Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>.

#ifndef MARSPOSE_PARAMLOADER_H
#define MARSPOSE_PARAMLOADER_H

#include "rclcpp/rclcpp.hpp"

class ParamLoad
{
public:
  bool publish_on_propagation_{ true };  ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };       ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };         ///< If true, all verbose infos are printed
  bool verbose_ooo_{ true };             ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };  ///< If true, all out of order propagation sensor meas are discarded
  bool pub_cov_{ true };                 ///< Publish covariances in the ext core state message if true
  bool pub_path_{ false };               ///< Publish all core states as nav_msgs::Path (for rviz)
  int buffer_size_{ 2000 };              ///< Set mars buffersize

  bool use_tcpnodelay_{ true };  ///< Use tcp no delay for the ROS msg. system
  bool bypass_init_service_{ false };

  int pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  int sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  int sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements

  double g_rate_noise_;
  double g_bias_noise_;
  double a_noise_;
  double a_bias_noise_;

  Eigen::Vector3d core_init_cov_p_;
  Eigen::Vector3d core_init_cov_v_;
  Eigen::Vector3d core_init_cov_q_;
  Eigen::Vector3d core_init_cov_bw_;
  Eigen::Vector3d core_init_cov_ba_;

  Eigen::Vector3d pose1_pos_meas_noise_;
  Eigen::Vector3d pose1_rot_meas_noise_;
  bool pose1_use_dyn_meas_noise_{ false };
  Eigen::Vector3d pose1_cal_p_ip_;
  Eigen::Quaterniond pose1_cal_q_ip_;
  Eigen::Matrix<double, 6, 1> pose1_state_init_cov_;

  Eigen::Vector3d pose2_pos_meas_noise_;
  Eigen::Vector3d pose2_rot_meas_noise_;
  bool pose2_use_dyn_meas_noise_{ false };
  Eigen::Vector3d pose2_cal_p_ip_;
  Eigen::Quaterniond pose2_cal_q_ip_;
  Eigen::Matrix<double, 6, 1> pose2_state_init_cov_;


  void check_size(const int& size_in, const int& size_comp)
  {
    if (size_comp != size_in)
    {
      std::cerr << "YAML array with wrong size" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  ParamLoad(rclcpp::Node* node)
  {
    node->declare_parameter("pub_on_prop", true);
    node->declare_parameter("use_ros_time_now", false);
    node->declare_parameter("verbose", false);
    node->declare_parameter("verbose_out_of_order", true);
    node->declare_parameter("discard_ooo_prop_meas", false);
    node->declare_parameter("pub_cov", true);
    node->declare_parameter("pub_path", true);
    node->declare_parameter("buffer_size", 2000);

    node->declare_parameter("use_tcpnodelay", true);
    node->declare_parameter("bypass_init_service", false);

    node->declare_parameter("pub_cb_buffer_size", 1);
    node->declare_parameter("sub_imu_cb_buffer_size", 200);
    node->declare_parameter("sub_sensor_cb_buffer_size", 1);

    node->declare_parameter("gyro_rate_noise", 0.426188);
    node->declare_parameter("gyro_bias_noise", 0.154667);
    node->declare_parameter("acc_noise", 0.89791);
    node->declare_parameter("acc_bias_noise", 0.28402);

    node->declare_parameter("core_init_cov_p", std::vector<double>{ 0.5, 0.5, 0.5 });
    node->declare_parameter("core_init_cov_v", std::vector<double>{ 0.3, 0.3, 0.3 });
    node->declare_parameter("core_init_cov_q", std::vector<double>{ 0.1218, 0.1218, 0.1218 });
    node->declare_parameter("core_init_cov_bw", std::vector<double>{ 0.0076, 0.0076, 0.0076 });
    node->declare_parameter("core_init_cov_ba", std::vector<double>{ 0.01, 0.01, 0.01 });

    node->declare_parameter("pose1_pos_meas_noise", std::vector<double>{ 0.1, 0.1, 0.1 });
    node->declare_parameter("pose1_rot_meas_noise", std::vector<double>{ 0.03, 0.03, 0.03 });
    node->declare_parameter("pose1_cal_p_ip", std::vector<double>{ 0.0, 0.0, 0.0 });
    node->declare_parameter("pose1_cal_q_ip", std::vector<double>{ 1.0, 0.0, 0.0, 0.0 });
    node->declare_parameter("pose1_state_init_cov", std::vector<double>{ 0.01, 0.01, 0.01, 0.03, 0.03, 0.03 });

    node->declare_parameter("pose2_pos_meas_noise", std::vector<double>{ 0.1, 0.1, 0.1 });
    node->declare_parameter("pose2_rot_meas_noise", std::vector<double>{ 0.03, 0.03, 0.03 });
    node->declare_parameter("pose2_cal_p_ip", std::vector<double>{ 0.0, 0.0, 0.0 });
    node->declare_parameter("pose2_cal_q_ip", std::vector<double>{ 1.0, 0.0, 0.0, 0.0 });
    node->declare_parameter("pose2_state_init_cov", std::vector<double>{ 0.01, 0.01, 0.01, 0.03, 0.03, 0.03 });

    publish_on_propagation_ = node->get_parameter("pub_on_prop").as_bool();
   
    std::cout << publish_on_propagation_ << std::endl;
   
    use_ros_time_now_ = node->get_parameter("use_ros_time_now").as_bool();
    verbose_output_ = node->get_parameter("verbose").as_bool();
    verbose_ooo_ = node->get_parameter("verbose_out_of_order").as_bool();
    discard_ooo_prop_meas_ = node->get_parameter("discard_ooo_prop_meas").as_bool();
    pub_cov_ = node->get_parameter("pub_cov").as_bool();
    pub_path_ = node->get_parameter("pub_path").as_bool();
    buffer_size_ = node->get_parameter("buffer_size").as_int();

    use_tcpnodelay_ = node->get_parameter("use_tcpnodelay").as_bool();
    bypass_init_service_ = node->get_parameter("bypass_init_service").as_bool();

    pub_cb_buffer_size_ = node->get_parameter("pub_cb_buffer_size").as_int();
    sub_imu_cb_buffer_size_ = node->get_parameter("sub_imu_cb_buffer_size").as_int();
    sub_sensor_cb_buffer_size_ = node->get_parameter("sub_sensor_cb_buffer_size").as_int();

    g_rate_noise_ = node->get_parameter("gyro_rate_noise").as_double();
    g_bias_noise_ = node->get_parameter("gyro_bias_noise").as_double();
    a_noise_ = node->get_parameter("acc_noise").as_double();
    a_bias_noise_ = node->get_parameter("acc_bias_noise").as_double();

    std::vector<double> core_init_cov_p = node->get_parameter("core_init_cov_p").as_double_array();
    std::vector<double> core_init_cov_v = node->get_parameter("core_init_cov_v").as_double_array();
    std::vector<double> core_init_cov_q = node->get_parameter("core_init_cov_q").as_double_array();
    std::vector<double> core_init_cov_bw = node->get_parameter("core_init_cov_bw").as_double_array();
    std::vector<double> core_init_cov_ba = node->get_parameter("core_init_cov_ba").as_double_array();

    check_size(core_init_cov_p.size(), 3);
    check_size(core_init_cov_v.size(), 3);
    check_size(core_init_cov_q.size(), 3);
    check_size(core_init_cov_bw.size(), 3);
    check_size(core_init_cov_ba.size(), 3);

    core_init_cov_p_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_p.data());
    core_init_cov_v_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_v.data());
    core_init_cov_q_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_q.data());
    core_init_cov_bw_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_bw.data());
    core_init_cov_ba_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_ba.data());

    std::vector<double> pose1_pos_meas_noise = node->get_parameter("pose1_pos_meas_noise").as_double_array();
    std::vector<double> pose1_rot_meas_noise = node->get_parameter("pose1_rot_meas_noise").as_double_array();
    std::vector<double> pose1_cal_p_ip = node->get_parameter("pose1_cal_p_ip").as_double_array();
    std::vector<double> pose1_cal_q_ip = node->get_parameter("pose1_cal_q_ip").as_double_array();
    std::vector<double> pose1_state_init_cov = node->get_parameter("pose1_state_init_cov").as_double_array();

    check_size(pose1_pos_meas_noise.size(), 3);
    check_size(pose1_rot_meas_noise.size(), 3);
    check_size(pose1_cal_p_ip.size(), 3);
    check_size(pose1_cal_q_ip.size(), 4);
    check_size(pose1_state_init_cov.size(), 6);

    pose1_pos_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose1_pos_meas_noise.data());
    pose1_rot_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose1_rot_meas_noise.data());
    pose1_cal_p_ip_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose1_cal_p_ip.data());
    pose1_cal_q_ip_ = Eigen::Quaterniond(pose1_cal_q_ip[0], pose1_cal_q_ip[1], pose1_cal_q_ip[2], pose1_cal_q_ip[3]);
    pose1_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(pose1_state_init_cov.data());

    std::vector<double> pose2_pos_meas_noise = node->get_parameter("pose2_pos_meas_noise").as_double_array();
    std::vector<double> pose2_rot_meas_noise = node->get_parameter("pose2_rot_meas_noise").as_double_array();
    std::vector<double> pose2_cal_p_ip = node->get_parameter("pose2_cal_p_ip").as_double_array();
    std::vector<double> pose2_cal_q_ip = node->get_parameter("pose2_cal_q_ip").as_double_array();
    std::vector<double> pose2_state_init_cov = node->get_parameter("pose2_state_init_cov").as_double_array();

    check_size(pose1_pos_meas_noise.size(), 3);
    check_size(pose1_rot_meas_noise.size(), 3);
    check_size(pose1_cal_p_ip.size(), 3);
    check_size(pose1_cal_q_ip.size(), 4);
    check_size(pose1_state_init_cov.size(), 6);

    pose2_pos_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose2_pos_meas_noise.data());
    pose2_rot_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose2_rot_meas_noise.data());
    pose2_cal_p_ip_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(pose2_cal_p_ip.data());
    pose2_cal_q_ip_ = Eigen::Quaterniond(pose2_cal_q_ip[0], pose2_cal_q_ip[1], pose2_cal_q_ip[2], pose2_cal_q_ip[3]);
    pose2_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(pose2_state_init_cov.data());
  }
};

#endif  // MARSPOSE_PARAMLOADER_H
