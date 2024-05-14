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

#ifndef MARSWRAPPERPOSE_H
#define MARSWRAPPERPOSE_H

#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "mars_pose_paramloader.h"
#include "mars_msg_conv.h"
#include "mars_wrapper_pose.h"
#include "rclcpp/rclcpp.hpp"

class MarsWrapperPose : public rclcpp::Node
{
public:
  MarsWrapperPose();

private:
  // Settings
  ParamLoad m_sett_;

  // Initialize framework components
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  // Sensor instances
  std::shared_ptr<mars::PoseSensorClass> pose1_sensor_sptr_;  /// Pose update sensor instance

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_measurement_;  ///< IMU measurement subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_measurement_;  ///< Pose stamped measurement
                                                                                           ///< subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_pose_with_cov_measurement_;  ///< Pose with covariance stamped subscriber

  // Publisher
  rclcpp::Publisher<mars_msgs::msg::ExtCoreState>::SharedPtr pub_ext_core_state_;  ///< Publisher for the Core-State
                                                                                   ///< mars_ros::ExtCoreState message
  rclcpp::Publisher<mars_msgs::msg::ExtCoreStateLite>::SharedPtr
      pub_ext_core_state_lite_;                                                        ///< Publisher for the
                                                                                       ///< Core-State
                                                                                       ///< mars_ros::ExtCoreStateLite
                                                                                       ///< message
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_core_pose_state_;  ///< Publisher for the Core-State
                                                                                       ///< pose stamped message
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_core_odom_state_;  ///< Publisher for the Core-State odom
                                                                               ///< stamped message
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_core_path_;  ///< Publisher for all Core-States in buffer as
                                                                     ///< path message
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose1_state_;  ///< Publisher for the pose sensor
                                                                                   ///< calibration state
  MarsPathGen path_generator_;  ///< Generator and storage for nav_msgs::Path

  // Sensor Callbacks
  ///
  /// \brief ImuMeasurementCallback IMU measurment callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the propagation sensor routine
  ///
  void ImuMeasurementCallback(const sensor_msgs::msg::Imu::ConstPtr& meas);

  ///
  /// \brief PoseMeasurementCallback Pose measurement callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void PoseMeasurementCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& meas);

  ///
  /// \brief PoseWithCovMeasurementCallback Pose with cov measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void PoseWithCovMeasurementCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& meas);

  Eigen::Vector3d p_wi_init_;     ///< Latest position which will be used to initialize the filter
  Eigen::Quaterniond q_wi_init_;  ///< Latest orientation to initialize the filter
  bool do_state_init_{ false };   ///< Trigger if the filter should be initialized

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

  // Sensor Updates
  ///
  /// \brief PoseMeasurementUpdate Pose sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param pose_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  void PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
                             const mars::PoseMeasurementType& pose_meas, const mars::Time& timestamp);
};

#endif  // MARSWRAPPERPOSE_H
