/// @file aruco_detector_double.cpp
/// @brief ROS 2 node for detecting ArUco markers using two simulated camera feeds and fusing their detections.

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <unordered_map>
#include <mutex>

/// @class ArucoDetectorDouble
/// @brief Detects ArUco markers from two simulated cameras and fuses their poses.
class ArucoDetectorDouble : public rclcpp::Node {
public:
  /// @brief Constructor for ArucoDetectorDouble.
  ArucoDetectorDouble() : Node("aruco_detector_double"), tf_broadcaster_(this),
                          mouse_data_1_{this, "Camera 1"}, mouse_data_2_{this, "Camera 2"} {
    // Get the package share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_control");

    // Construct the full paths to the configuration files
    std::string calib_file_1 = package_share_directory + "/config/camera_calibration.yaml";
    std::string calib_file_2 = package_share_directory + "/config/camera_calibration.yaml";
    std::string transform_file = package_share_directory + "/config/transform.yaml";

    // Load calibration and transform data
    readCameraCalibration(calib_file_1, camMatrix_1_, distCoeffs_1_);
    readCameraCalibration(calib_file_2, camMatrix_2_, distCoeffs_2_);
    readTransforms(transform_file, 1, camera_transform_1_);
    readTransforms(transform_file, 2, camera_transform_2_);

    // Set marker parameters
    marker_length_ = 0.0425; // Adjust as per your simulation markers

    // Configure ArUco detector parameters
    configureDetectorParameters();

    // Initialize subscribers and synchronizer
    image_sub1_.subscribe(this, "camera1/image_raw");
    image_sub2_.subscribe(this, "camera2/image_raw");
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), image_sub1_, image_sub2_);
    sync_->registerCallback(std::bind(&ArucoDetectorDouble::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // Initialize publisher for /tf_detected
    tf_detected_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tf_detected", 10);
  }

  /// @brief Destructor to clean up OpenCV windows.
  ~ArucoDetectorDouble() {
    cv::destroyAllWindows();
  }

private:
  // ROS 2 subscriptions and synchronization
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub2_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  std::shared_ptr<Synchronizer> sync_;

  // ROS 2 publisher and TF broadcaster
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_detected_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Camera calibration and transform data
  cv::Mat camMatrix_1_, distCoeffs_1_, camMatrix_2_, distCoeffs_2_;
  Eigen::Matrix4d camera_transform_1_, camera_transform_2_;
  double marker_length_;

  // ArUco detection parameters
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // Detection storage and synchronization
  std::mutex detection_mutex_;
  std::unordered_map<int, bool> first_detection_1_, first_detection_2_;
  std::unordered_map<int, cv::Vec3d> first_tvecs_1_, first_tvecs_2_;
  std::unordered_map<int, cv::Vec3d> first_rvecs_1_, first_rvecs_2_;
  std::unordered_set<int> previously_detected_1_, previously_detected_2_;

  // Mouse callback data
  struct MouseCallbackData {
    ArucoDetectorDouble* self;
    std::string window_name;
  };
  MouseCallbackData mouse_data_1_;
  MouseCallbackData mouse_data_2_;

  /// @brief Reads camera calibration parameters from a file.
  void readCameraCalibration(const std::string& filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", filename.c_str());
      return;
    }
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
  }

  /// @brief Reads transformation matrices from a YAML file.
  void readTransforms(const std::string& filename, int camera_id, Eigen::Matrix4d& transform) {
    try {
      YAML::Node config = YAML::LoadFile(filename);
      if (!config["camera"]) {
        RCLCPP_ERROR(this->get_logger(), "No camera transforms found in: %s", filename.c_str());
        return;
      }
      for (const auto& camera : config["camera"]) {
        if (camera["id"].as<int>() == camera_id) {
          transform = parseTransform(camera["transform"]);
          return;
        }
      }
      RCLCPP_ERROR(this->get_logger(), "Transform for camera %d not found in: %s", camera_id, filename.c_str());
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML error in '%s': %s", filename.c_str(), e.what());
    }
  }

  /// @brief Parses a transformation matrix from a YAML node.
  Eigen::Matrix4d parseTransform(const YAML::Node& node) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int i = 0; i < node.size(); ++i)
      for (int j = 0; j < node[i].size(); ++j)
        transform(i, j) = node[i][j].as<double>();
    return transform;
  }

  /// @brief Configures ArUco detector parameters.
  void configureDetectorParameters() {
    detectorParams_ = cv::aruco::DetectorParameters::create();
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  }

  /// @brief Callback for synchronized image messages.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg1,
                     const sensor_msgs::msg::Image::ConstSharedPtr& img_msg2) {
    cv::Mat image1, image2;
    try {
      image1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image;
      image2 = cv_bridge::toCvShare(img_msg2, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    auto markers_1 = processSingleFrame(image1, camMatrix_1_, distCoeffs_1_, camera_transform_1_, "Camera 1",
                                        first_detection_1_, first_tvecs_1_, first_rvecs_1_, previously_detected_1_);
    auto markers_2 = processSingleFrame(image2, camMatrix_2_, distCoeffs_2_, camera_transform_2_, "Camera 2",
                                        first_detection_2_, first_tvecs_2_, first_rvecs_2_, previously_detected_2_);
    fuseAndPublishMarkers(markers_1, markers_2);
  }

  /// @brief Processes a single frame and detects markers.
  std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> processSingleFrame(
      cv::Mat& frame, cv::Mat& camMatrix, cv::Mat& distCoeffs, Eigen::Matrix4d& camera_transform,
      const std::string& camera_name, std::unordered_map<int, bool>& first_detection,
      std::unordered_map<int, cv::Vec3d>& first_tvecs, std::unordered_map<int, cv::Vec3d>& first_rvecs,
      std::unordered_set<int>& previously_detected) {
    cv::Mat gray, filtered, enhanced, undistortedFrame;
    std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> detected_markers;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s: Empty frame received", camera_name.c_str());
      undistortedFrame = cv::Mat::zeros(720, 1280, CV_8UC3);
      cv::putText(undistortedFrame, "No frame from " + camera_name, cv::Point(50, 360),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    } else {
      int height = frame.rows;
      int width = frame.cols;
      cv::Mat camMatrixNew = cv::getOptimalNewCameraMatrix(camMatrix, distCoeffs, cv::Size(width, height), 1, cv::Size(width, height));
      cv::undistort(frame, undistortedFrame, camMatrixNew, distCoeffs);

      cv::cvtColor(undistortedFrame, gray, cv::COLOR_BGR2GRAY);
      cv::bilateralFilter(gray, filtered, 3, 75, 20);
      cv::equalizeHist(filtered, enhanced);

      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners;
      cv::aruco::detectMarkers(gray, dictionary_, markerCorners, markerIds, detectorParams_);

      if (!markerCorners.empty()) {
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
        for (auto& corners : markerCorners)
          cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      }

      std::unordered_set<int> currently_detected(markerIds.begin(), markerIds.end());
      for (int prev_marker : previously_detected) {
        if (currently_detected.find(prev_marker) == currently_detected.end()) {
          RCLCPP_WARN(this->get_logger(), "%s: Marker %d lost, reinitializing", camera_name.c_str(), prev_marker);
          first_detection.erase(prev_marker);
          first_tvecs.erase(prev_marker);
          first_rvecs.erase(prev_marker);
        }
      }
      previously_detected = currently_detected;

      if (!markerIds.empty()) {
        std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
        std::vector<cv::Point3f> markerPoints = {
            cv::Point3f(-marker_length_ / 2, marker_length_ / 2, 0),
            cv::Point3f(marker_length_ / 2, marker_length_ / 2, 0),
            cv::Point3f(marker_length_ / 2, -marker_length_ / 2, 0),
            cv::Point3f(-marker_length_ / 2, -marker_length_ / 2, 0)
        };

        for (size_t i = 0; i < markerIds.size(); ++i) {
          int marker_id = markerIds[i];
          if (markerCorners[i].size() != 4 || !cv::solvePnP(markerPoints, markerCorners[i], camMatrixNew, distCoeffs, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE))
            continue;

          if (first_detection.count(marker_id) == 0) {
            first_detection[marker_id] = true;
            first_tvecs[marker_id] = tvecs[i];
            first_rvecs[marker_id] = rvecs[i];
          } else {
            const double rotation_threshold = 0.02;
            const double zero_crossing_threshold = 0.02;
            cv::Vec3d& prev_rvec = first_rvecs[marker_id];
            for (int j = 0; j < 3; j++) {
              double diff = std::abs(rvecs[i][j] - prev_rvec[j]);
              bool sign_changed = (rvecs[i][j] * prev_rvec[j] < 0);
              if (sign_changed && diff > rotation_threshold && diff >= zero_crossing_threshold)
                rvecs[i][j] = -rvecs[i][j];
            }
            first_tvecs[marker_id] = tvecs[i];
            first_rvecs[marker_id] = rvecs[i];
          }

          detected_markers[marker_id] = {rvecs[i], tvecs[i]};

          cv::aruco::drawDetectedMarkers(undistortedFrame, markerCorners, markerIds);
          std::vector<cv::Point3f> axisPoints = {
              cv::Point3f(0, 0, 0), cv::Point3f(marker_length_ * 1.5f, 0, 0),
              cv::Point3f(0, marker_length_ * 1.5f, 0), cv::Point3f(0, 0, marker_length_ * 1.5f)
          };
          std::vector<cv::Point2f> imagePoints;
          cv::projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix, distCoeffs, imagePoints);
          cv::line(undistortedFrame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X - Red
          cv::line(undistortedFrame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y - Green
          cv::line(undistortedFrame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z - Blue
        }
      }
    }

    cv::imshow(camera_name, undistortedFrame);
    if (camera_name == "Camera 1") {
      cv::setMouseCallback(camera_name, onMouse, &mouse_data_1_);
    } else if (camera_name == "Camera 2") {
      cv::setMouseCallback(camera_name, onMouse, &mouse_data_2_);
    }
    cv::waitKey(1);

    return detected_markers;
  }

  /// @brief Fuses marker detections from both cameras and publishes transforms.
  void fuseAndPublishMarkers(const std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>>& markers_1,
                             const std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>>& markers_2) {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> fused_markers;

    std::unordered_set<int> all_ids;
    for (const auto& [id, _] : markers_1) all_ids.insert(id);
    for (const auto& [id, _] : markers_2) all_ids.insert(id);

    for (int id : all_ids) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      std::vector<Eigen::Matrix4d> camera_transforms;

      if (markers_1.count(id)) {
        rvecs.push_back(markers_1.at(id).first);
        tvecs.push_back(markers_1.at(id).second);
        camera_transforms.push_back(camera_transform_1_);
      }
      if (markers_2.count(id)) {
        rvecs.push_back(markers_2.at(id).first);
        tvecs.push_back(markers_2.at(id).second);
        camera_transforms.push_back(camera_transform_2_);
      }

      if (!rvecs.empty()) {
        cv::Vec3d tvec = {0, 0, 0};
        for (const auto& t : tvecs) tvec += t;
        tvec /= static_cast<double>(tvecs.size());

        std::vector<Eigen::Quaterniond> quaternions;
        for (const auto& rvec : rvecs) {
          cv::Mat rot_mat;
          cv::Rodrigues(rvec, rot_mat);
          Eigen::Matrix3d eigen_rot;
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              eigen_rot(i, j) = rot_mat.at<double>(i, j);
          quaternions.emplace_back(eigen_rot);
        }

        Eigen::Quaterniond avg_quat(0, 0, 0, 0);
        for (const auto& q : quaternions) {
          avg_quat.coeffs() += q.coeffs();
        }
        avg_quat.coeffs() /= static_cast<double>(quaternions.size());
        avg_quat.normalize();

        Eigen::Matrix3d avg_rot = avg_quat.toRotationMatrix();
        cv::Mat avg_rot_mat(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            avg_rot_mat.at<double>(i, j) = avg_rot(i, j);
        cv::Vec3d rvec;
        cv::Rodrigues(avg_rot_mat, rvec);

        fused_markers[id] = {rvec, tvec};
        publishTransform(rvec, tvec, id, rvecs.size() == 2 ? "both" : (markers_1.count(id) ? "camera_1" : "camera_2"));
      }
    }
  }

  /// @brief Publishes the transform of a detected marker.
  void publishTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec, int marker_id, const std::string& source) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Matrix4d marker_transform = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col)
        marker_transform(row, col) = rotation_matrix.at<double>(row, col);
      marker_transform(row, 3) = tvec[row];
    }

    Eigen::Matrix4d world_to_marker;
    if (source == "camera_2") {
      world_to_marker = camera_transform_2_ * marker_transform;
    } else if (source == "camera_1" || source == "both") { // Default to camera_1 for "both"
      world_to_marker = camera_transform_1_ * marker_transform;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);
    transformStamped.transform.translation.x = world_to_marker(0, 3);
    transformStamped.transform.translation.y = world_to_marker(1, 3);
    transformStamped.transform.translation.z = world_to_marker(2, 3);

    Eigen::Matrix3d rotation = world_to_marker.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion(rotation);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    tf_broadcaster_.sendTransform(transformStamped);
    if (marker_id == 0)
      tf_detected_publisher_->publish(transformStamped);
  }

  /// @brief Static mouse callback function to display cursor coordinates.
  static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
    if (event == cv::EVENT_MOUSEMOVE) {
      std::stringstream ss;
      ss << data->window_name << " - Cursor: (" << x << ", " << y << ")";
      cv::setWindowTitle(data->window_name, ss.str());
    }
  }
};

/// @brief Main function to initialize and spin the node.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorDouble>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}