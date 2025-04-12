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

#include "mars_wrapper_dualpose.h"

MarsWrapperDualPose::MarsWrapperDualPose() : Node("mars_ros_node"), m_sett_(this) {

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
    // Pose1
    pose1_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose1", core_states_sptr_);
    {  // Limit scope of temp variables
        Eigen::Matrix<double, 6, 1> pose_meas_std;
        pose_meas_std << m_sett_.pose1_pos_meas_noise_, m_sett_.pose1_rot_meas_noise_;
        pose1_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);
        pose1_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.pose1_use_dyn_meas_noise_;

        mars::PoseSensorData pose_calibration;
        pose_calibration.state_.p_ip_ = m_sett_.pose1_cal_p_ip_;
        pose_calibration.state_.q_ip_ = m_sett_.pose1_cal_q_ip_;
        Eigen::Matrix<double, 6, 6> pose_cov;
        pose_cov.setZero();
        pose_cov.diagonal() << m_sett_.pose1_state_init_cov_;  // no calibration update
        pose_calibration.sensor_cov_ = pose_cov;
        pose1_sensor_sptr_->set_initial_calib(std::make_shared<mars::PoseSensorData>(pose_calibration));

        // TODO is set here for now, but will be managed by core logic in later versions
        pose1_sensor_sptr_->const_ref_to_nav_ = true;

        std::cout << "Info: [" << pose1_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
        std::cout << "\tPosition[m]: [" << pose_calibration.state_.p_ip_.transpose() << " ]" << std::endl;
        std::cout << "\tOrientation[1]: [" << pose_calibration.state_.q_ip_.w() << " "
                << pose_calibration.state_.q_ip_.vec().transpose() << " ]" << std::endl;
        std::cout << "\tOrientation[deg]: ["
                << pose_calibration.state_.q_ip_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
                << " ]" << std::endl;
    }

    // Pose2
    pose2_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose2", core_states_sptr_);
    {  // Limit scope of temp variables
        Eigen::Matrix<double, 6, 1> pose_meas_std;
        pose_meas_std << m_sett_.pose1_pos_meas_noise_, m_sett_.pose2_rot_meas_noise_;
        pose2_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);
        pose2_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.pose2_use_dyn_meas_noise_;

        mars::PoseSensorData pose_calibration;
        pose_calibration.state_.p_ip_ = m_sett_.pose2_cal_p_ip_;
        pose_calibration.state_.q_ip_ = m_sett_.pose2_cal_q_ip_;
        Eigen::Matrix<double, 6, 6> pose_cov;
        pose_cov.setZero();
        pose_cov.diagonal() << m_sett_.pose2_state_init_cov_;  // no calibration update
        pose_calibration.sensor_cov_ = pose_cov;
        pose2_sensor_sptr_->set_initial_calib(std::make_shared<mars::PoseSensorData>(pose_calibration));

        // TODO is set here for now, but will be managed by core logic in later versions
        pose2_sensor_sptr_->const_ref_to_nav_ = true;

        std::cout << "Info: [" << pose2_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
        std::cout << "\tPosition[m]: [" << pose_calibration.state_.p_ip_.transpose() << " ]" << std::endl;
        std::cout << "\tOrientation[1]: [" << pose_calibration.state_.q_ip_.w() << " "
                << pose_calibration.state_.q_ip_.vec().transpose() << " ]" << std::endl;
        std::cout << "\tOrientation[deg]: ["
                << pose_calibration.state_.q_ip_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
                << " ]" << std::endl;
    }
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(m_sett_.sub_imu_cb_buffer_size_), rmw_qos_profile_sensor_data);
    sub_imu_measurement_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_in", qos,
            std::bind(&MarsWrapperDualPose::ImuMeasurementCallback, this, std::placeholders::_1));
    sub_pose1_measurement_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose1_in", qos,
            std::bind(&MarsWrapperDualPose::Pose1MeasurementCallback, this, std::placeholders::_1));
    sub_pose1_with_cov_measurement_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose1_with_cov_in", qos,
            std::bind(&MarsWrapperDualPose::Pose1WithCovMeasurementCallback, this, std::placeholders::_1));
    sub_odom1_measurement_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom1_in", qos,
            std::bind(&MarsWrapperDualPose::Odom1MeasurementCallback, this, std::placeholders::_1));
    sub_transform1_measurement_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform1_in", qos,
            std::bind(&MarsWrapperDualPose::Transform1MeasurementCallback, this, std::placeholders::_1));
    sub_pose2_measurement_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose2_in", qos,
            std::bind(&MarsWrapperDualPose::Pose2MeasurementCallback, this, std::placeholders::_1));
    sub_pose2_with_cov_measurement_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose2_with_cov_in", qos,
            std::bind(&MarsWrapperDualPose::Pose2WithCovMeasurementCallback, this, std::placeholders::_1));
    sub_odom2_measurement_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom2_in", qos,
            std::bind(&MarsWrapperDualPose::Odom2MeasurementCallback, this, std::placeholders::_1));
    sub_transform2_measurement_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform2_in", qos,
            std::bind(&MarsWrapperDualPose::Transform2MeasurementCallback, this, std::placeholders::_1));

    // Publisher
    pub_ext_core_state_ = this->create_publisher<mars_msgs::msg::ExtCoreState>("core_ext_state_out", m_sett_.pub_cb_buffer_size_);
    pub_ext_core_state_lite_ = this->create_publisher<mars_msgs::msg::ExtCoreStateLite>("core_ext_state_lite_out", m_sett_.pub_cb_buffer_size_);
    pub_core_pose_state_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("core_pose_state_out", m_sett_.pub_cb_buffer_size_);
    pub_core_odom_state_ = this->create_publisher<nav_msgs::msg::Odometry>("core_odom_state_out", m_sett_.pub_cb_buffer_size_);
    pub_pose1_state_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose1_cal_state_out", m_sett_.pub_cb_buffer_size_);
    pub_pose2_state_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose2_cal_state_out", m_sett_.pub_cb_buffer_size_);
    if (m_sett_.pub_path_)
    {
        pub_core_path_ = this->create_publisher<nav_msgs::msg::Path>("core_states_path", m_sett_.pub_cb_buffer_size_);
    }
}


void MarsWrapperDualPose::ImuMeasurementCallback(const sensor_msgs::msg::Imu::ConstPtr& meas)
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
    if (!core_logic_.core_is_initialized_ && do_state_init_)
    {
        core_logic_.Initialize(p_wi_init_, q_wi_init_);
    }

    if (m_sett_.publish_on_propagation_ && valid_update)
    {
        this->RunCoreStatePublisher();
    }
}

void MarsWrapperDualPose::Pose1MeasurementCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Pose1WithCovMeasurementCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Transform1MeasurementCallback(const geometry_msgs::msg::TransformStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::TransformMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Odom1MeasurementCallback(const nav_msgs::msg::Odometry::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Pose2MeasurementCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Pose2WithCovMeasurementCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Transform2MeasurementCallback(const geometry_msgs::msg::TransformStamped::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::TransformMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::Odom2MeasurementCallback(const nav_msgs::msg::Odometry::ConstPtr& meas)
{
    // Map the measurement to the mars sensor type
    mars::Time timestamp((double)meas->header.stamp.sec + (double)meas->header.stamp.nanosec / 1e9);

    mars::PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPose::RunCoreStatePublisher()
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


void MarsWrapperDualPose::PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
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
