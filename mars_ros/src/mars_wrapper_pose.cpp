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

#include "mars_wrapper_pose.h"

MarsWrapperPose::MarsWrapperPose() : Node("mars_ros_node"), m_sett_(this) {

    // Framework components
    imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
    core_states_sptr_ = std::make_shared<mars::CoreState>();
    
    core_states_sptr_.get()->set_initial_covariance(m_sett_.core_init_cov_p_, m_sett_.core_init_cov_v_,
                                                  m_sett_.core_init_cov_q_, m_sett_.core_init_cov_bw_,
                                                  m_sett_.core_init_cov_ba_);

    core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
    core_logic_ = mars::CoreLogic(core_states_sptr_);
    core_logic_.buffer_.set_max_buffer_size(m_sett_.buffer_size_);

    core_logic_.verbose_ = m_sett_.verbose_output_;
    core_logic_.verbose_out_of_order_ = m_sett_.verbose_ooo_;
    core_logic_.discard_ooo_prop_meas_ = m_sett_.discard_ooo_prop_meas_;

    core_states_sptr_->set_noise_std(
      Eigen::Vector3d(m_sett_.g_rate_noise_, m_sett_.g_rate_noise_, m_sett_.g_rate_noise_),
      Eigen::Vector3d(m_sett_.g_bias_noise_, m_sett_.g_bias_noise_, m_sett_.g_bias_noise_),
      Eigen::Vector3d(m_sett_.a_noise_, m_sett_.a_noise_, m_sett_.a_noise_),
      Eigen::Vector3d(m_sett_.a_bias_noise_, m_sett_.a_bias_noise_, m_sett_.a_bias_noise_));

    // Sensors
    pose1_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose1", core_states_sptr_);
    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett_.pose1_pos_meas_noise_, m_sett_.pose1_rot_meas_noise_;
    pose1_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);
    pose1_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.pose1_use_dyn_meas_noise_;

    mars::PoseSensorData pose_calibration;
    pose_calibration.state_.p_ip_ = m_sett_.pose1_cal_p_ip_;
    pose_calibration.state_.q_ip_ = m_sett_.pose1_cal_q_ip_;

    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << m_sett_.pose1_state_init_cov_;
    pose_calibration.sensor_cov_ = pose_cov;

    pose1_sensor_sptr_->set_initial_calib(std::make_shared<mars::PoseSensorData>(pose_calibration));

    // TODO(CHB) is set here for now, but will be managed by core logic in later versions
    pose1_sensor_sptr_->const_ref_to_nav_ = true;
    
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(m_sett_.sub_imu_cb_buffer_size_), rmw_qos_profile_sensor_data);
    sub_imu_measurement_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_in",
            qos,
            std::bind(&MarsWrapperPose::ImuMeasurementCallback, this, std::placeholders::_1));
    sub_pose_measurement_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose_in",
            qos,
            std::bind(&MarsWrapperPose::PoseMeasurementCallback, this, std::placeholders::_1));
    sub_pose_with_cov_measurement_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose_with_cov_in",
            qos,
            std::bind(&MarsWrapperPose::PoseWithCovMeasurementCallback, this, std::placeholders::_1));


    // Publisher
    pub_ext_core_state_ = this->create_publisher<mars_msgs::msg::ExtCoreState>("core_ext_state_out", m_sett_.pub_cb_buffer_size_);
    pub_ext_core_state_lite_ = this->create_publisher<mars_msgs::msg::ExtCoreStateLite>("core_ext_state_lite_out", m_sett_.pub_cb_buffer_size_);
    pub_core_pose_state_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("core_pose_state_out", m_sett_.pub_cb_buffer_size_);
    pub_core_odom_state_ = this->create_publisher<nav_msgs::msg::Odometry>("core_odom_state_out", m_sett_.pub_cb_buffer_size_);
    pub_pose1_state_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_cal_state_out", m_sett_.pub_cb_buffer_size_);
    if (m_sett_.pub_path_)
    {
        pub_core_path_ = this->create_publisher<nav_msgs::msg::Path>("core_states_path", m_sett_.pub_cb_buffer_size_);
    }
  
}


void MarsWrapperPose::ImuMeasurementCallback(const sensor_msgs::msg::Imu::ConstPtr& meas)
{
    // Map the measutement to the mars type
    mars::Time timestamp;

    if (m_sett_.use_ros_time_now_)
    {
        timestamp = mars::Time(this->now().nanoseconds() / 1e9);
    }
    else
    {
        timestamp = mars::Time((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);
    }

    // Generate a measurement data block
    mars::BufferDataType data;
    data.set_measurement(std::make_shared<mars::IMUMeasurementType>(MarsMsgConv::ImuMsgToImuMeas(*meas)));

    // Call process measurement
    const bool valid_update = core_logic_.ProcessMeasurement(imu_sensor_sptr_, timestamp, data);

    // Initialize the first time at which the propagation sensor occures
    if (!core_logic_.core_is_initialized_)
    {
        core_logic_.Initialize(p_wi_init_, q_wi_init_);
    }

    if (m_sett_.publish_on_propagation_ && valid_update)
    {
        this->RunCoreStatePublisher();
    }
}


void MarsWrapperPose::PoseMeasurementCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

  mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperPose::PoseWithCovMeasurementCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

  mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}


void MarsWrapperPose::RunCoreStatePublisher()
{
    mars::BufferEntryType latest_state;
    const bool valid_state = core_logic_.buffer_.get_latest_state(&latest_state);

    if (!valid_state)
    {
        return;
    }
    
    mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_state_.get())->state_;

    if (m_sett_.pub_cov_)
    {
        mars::CoreStateMatrix cov = static_cast<mars::CoreType*>(latest_state.data_.core_state_.get())->cov_;
        pub_ext_core_state_->publish(
            MarsMsgConv::ExtCoreStateToMsgCov(latest_state.timestamp_.get_seconds(), latest_core_state, cov));
    }
    else
    {
        pub_ext_core_state_->publish(
            MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
    }

    pub_ext_core_state_lite_->publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

    pub_core_pose_state_->publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

    pub_core_odom_state_->publish(
      MarsMsgConv::ExtCoreStateToOdomMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

    if (m_sett_.pub_path_)
    {
        pub_core_path_->publish(
            path_generator_.ExtCoreStateToPathMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
    }
}


void MarsWrapperPose::PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
                                            const mars::PoseMeasurementType& pose_meas, const mars::Time& timestamp)
{
    if (!do_state_init_)
    {
        mars::PoseSensorStateType pose_cal = sensor_sptr->get_state(pose1_sensor_sptr_->initial_calib_);
        q_wi_init_ = pose_meas.orientation_ * pose_cal.q_ip_.inverse();
        p_wi_init_ = pose_meas.position_ + q_wi_init_.toRotationMatrix() * (-pose_cal.p_ip_);

        do_state_init_ = true;
    }

    mars::Time timestamp_corr;

    if (m_sett_.use_ros_time_now_)
    {
        timestamp_corr = mars::Time(this->now().nanoseconds() / 1e9);
    }
    else
    {
        timestamp_corr = timestamp;
    }

    // Generate a measurement data block
    mars::BufferDataType data;
    data.set_measurement(std::make_shared<mars::PoseMeasurementType>(pose_meas));

    // Call process measurement
    if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
    {
        return;
    }

    // Publish the latest core state
    this->RunCoreStatePublisher();

    // Publish the latest sensor state
    mars::BufferEntryType latest_result;
    const bool valid_state = core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_result);

    if (!valid_state)
    {
        return;
    }

    mars::PoseSensorStateType pose_sensor_state = sensor_sptr.get()->get_state(latest_result.data_.sensor_state_);

    pub_pose1_state_->publish(MarsMsgConv::PoseStateToPoseMsg(latest_result.timestamp_.get_seconds(), pose_sensor_state));
}
