/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <atomic>
#include <condition_variable>
#include <fins/node.hpp>
#include <mutex>
#include <queue>
#include <thread>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "fins/utils/time.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>

struct ImageData {
  double timestamp_sec;
  fins::AcqTime acq_time;
  cv::Mat image;
};

class VinsEstimatorNode : public fins::Node {
public:
  void define() override {
    set_name("VinsEstimator");
    set_description("An optimization-based multi-sensor state estimator.");
    set_category("SLAM");

    register_parameter<std::string>("config_file",
                                    &VinsEstimatorNode::on_config_update, "");

    register_input<0, cv::Mat>("left", &VinsEstimatorNode::on_left_image);

    register_input<1, cv::Mat>("right", &VinsEstimatorNode::on_right_image);

    register_output<0, nav_msgs::msg::Odometry>("odometry");
    register_output<1, geometry_msgs::msg::PoseStamped>("pose");
    register_output<2, geometry_msgs::msg::TransformStamped>("transform");
    register_output<3, nav_msgs::msg::Path>("path");
  }

  void initialize() override {
    logger->info("Initializing VINS-Fusion Node...");

    cv::setNumThreads(0);

    estimator.setOdometryCallback([this](double timestamp,
                                         const Eigen::Vector3d &P,
                                         const Eigen::Quaterniond &Q) {
      this->publish_odometry(timestamp, P, Q, false);
    });

    estimator.setIMUPropagateCallback([this](double timestamp,
                                             const Eigen::Vector3d &P,
                                             const Eigen::Quaterniond &Q) {
      this->publish_odometry(timestamp, P, Q, true);
    });

    is_running_ = true;
    sync_thread_ = std::thread(&VinsEstimatorNode::sync_process, this);

    logger->info("VINS-Fusion Node initialized.");
  }

  void deinitialize() {
    is_running_ = false;
    if (sync_thread_.joinable()) {
      sync_thread_.join();
    }
  }

  void run() override {}
  void pause() override {}
  void reset() override {}

  ~VinsEstimatorNode() { deinitialize(); }

  void on_config_update(const std::string &val) {
    config_file_ = val;
    readParameters(config_file_);
    estimator.setParameter();
  }

  void on_left_image(const fins::Msg<cv::Mat> &msg) {
    cv::Mat image = *msg;
    if (image.empty())
      return;

    std::lock_guard<std::mutex> lock(buf_mutex_);
    double t_sec = fins::to_seconds(msg.acq_time);
    img0_buf_.push({t_sec, msg.acq_time, image.clone()});
  }

  void on_right_image(const fins::Msg<cv::Mat> &msg) {
    cv::Mat image = *msg;
    if (image.empty())
      return;

    std::lock_guard<std::mutex> lock(buf_mutex_);
    double t_sec = fins::to_seconds(msg.acq_time);
    img1_buf_.push({t_sec, msg.acq_time, image.clone()});
  }

private:
  nav_msgs::msg::Path path_msg_;
  void sync_process() {
    while (is_running_) {
      cv::Mat image0, image1;
      double time = 0;
      bool new_pair_found = false;

      {
        std::lock_guard<std::mutex> lock(buf_mutex_);
        if (!img0_buf_.empty() && !img1_buf_.empty()) {
          double time0 = img0_buf_.front().timestamp_sec;
          double time1 = img1_buf_.front().timestamp_sec;

          if (time0 < time1 - 0.003) {
            img0_buf_.pop();
          } else if (time0 > time1 + 0.003) {
            img1_buf_.pop();
          } else {
            time = img0_buf_.front().timestamp_sec;
            image0 = img0_buf_.front().image;
            img0_buf_.pop();

            image1 = img1_buf_.front().image;
            img1_buf_.pop();

            new_pair_found = true;
          }
        }
      }

      if (new_pair_found) {
        cv::Mat img0_gray, img1_gray;

        if (image0.type() != CV_8UC1) {
          cv::cvtColor(image0, img0_gray, cv::COLOR_BGR2GRAY);
        } else {
          img0_gray = image0;
        }

        if (image1.type() != CV_8UC1) {
          cv::cvtColor(image1, img1_gray, cv::COLOR_BGR2GRAY);
        } else {
          img1_gray = image1;
        }

        cv::Mat image0_cloned = img0_gray.clone();
        cv::Mat image1_cloned = img1_gray.clone();
        try {
          estimator.inputImage(time, image0_cloned, image1_cloned);
        } catch (const std::exception &e) {
          logger->error("Estimator inputImage exception: {}", e.what());
          throw e;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  void publish_odometry(double timestamp_sec, const Eigen::Vector3d &P,
                        const Eigen::Quaterniond &Q, bool is_imu_propagate) {

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
    odom_msg.header.stamp.nanosec = static_cast<uint32_t>(
        (timestamp_sec - odom_msg.header.stamp.sec) * 1e9);
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "body";

    odom_msg.pose.pose.position.x = P.x();
    odom_msg.pose.pose.position.y = P.y();
    odom_msg.pose.pose.position.z = P.z();
    odom_msg.pose.pose.orientation.w = Q.w();
    odom_msg.pose.pose.orientation.x = Q.x();
    odom_msg.pose.pose.orientation.y = Q.y();
    odom_msg.pose.pose.orientation.z = Q.z();

    // for(int i=0; i<36; i++) odom_msg.pose.covariance[i] = ...

    this->send<0>(odom_msg, fins::from_seconds(timestamp_sec));

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    this->send<1>(pose_msg, fins::from_seconds(timestamp_sec));

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;
    tf_msg.transform.translation.x = P.x();
    tf_msg.transform.translation.y = P.y();
    tf_msg.transform.translation.z = P.z();
    tf_msg.transform.rotation.w = Q.w();
    tf_msg.transform.rotation.x = Q.x();
    tf_msg.transform.rotation.y = Q.y();
    tf_msg.transform.rotation.z = Q.z();
    this->send<2>(tf_msg, fins::from_seconds(timestamp_sec));

    pose_msg.header = odom_msg.header;
    path_msg_.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    this->send<3>(path_msg_, fins::from_seconds(timestamp_sec));
  }

  std::string config_file_;
  Estimator estimator;

  std::queue<ImageData> img0_buf_;
  std::queue<ImageData> img1_buf_;
  std::mutex buf_mutex_;

  std::thread sync_thread_;
  std::atomic<bool> is_running_{false};
};

EXPORT_NODE(VinsEstimatorNode)