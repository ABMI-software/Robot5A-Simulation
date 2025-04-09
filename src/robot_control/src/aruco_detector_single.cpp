/// @file aruco_detector_single.cpp
/// @brief Node that detects ArUco markers in simulation images and broadcasts their poses as TF transforms.

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

/// @class ArucoDetectorSingle
/// @brief Detects ArUco markers in simulation images and publishes their poses using TF.
class ArucoDetectorSingle : public rclcpp::Node {
public:
  /// @brief Constructor for ArucoDetectorSingle.
  ArucoDetectorSingle() : Node("aruco_detector_single"), tf_broadcaster_(this) {
    // Get the package share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_control");

    // Construct the full paths to the configuration files
    std::string camera_calibration_file = package_share_directory + "/config/camera_calibration.yaml";
    std::string camera_transform_file = package_share_directory + "/config/transform.yaml";

    // Read camera calibration and transform parameters
    readCameraCalibration(camera_calibration_file, camMatrix_, distCoeffs_);
    readTransforms(camera_transform_file);

    // Set marker parameters
    marker_length_ = 0.0425; // Adjust as per your simulation markers

    // Configure ArUco detector parameters
    configureDetectorParameters();

    // Create a subscription for receiving images
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera1/image_raw", 10, // change this to change camera feed
        std::bind(&ArucoDetectorSingle::imageCallback, this, std::placeholders::_1));

    // Create a publisher for the /tf_detected topic
    tf_detected_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "/tf_detected", 10);
  }

  /// @brief Destructor to clean up OpenCV windows.
  ~ArucoDetectorSingle() {
    cv::destroyAllWindows();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_detected_publisher_;
  cv::Mat camMatrix_, distCoeffs_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  Eigen::Matrix4d camera_transform_;
  double marker_length_;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // Store first detected values for filtering
  std::unordered_map<int, bool> first_detection;
  std::unordered_map<int, cv::Vec3d> first_tvecs;
  std::unordered_map<int, cv::Vec3d> first_rvecs;
  std::unordered_set<int> previously_detected_markers;

  /// @brief Reads camera calibration parameters from a file.
  void readCameraCalibration(const std::string &filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", filename.c_str());
      return;
    }
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
  }

  /// @brief Reads transformation matrices from a YAML file.
  void readTransforms(const std::string &filename) {
    try {
      YAML::Node config = YAML::LoadFile(filename);
      if (config["camera"]) {
        for (const auto &camera_node : config["camera"]) {
          if (camera_node["id"].as<int>() == 1) { // change this to change camera transform
            camera_transform_ = parseTransform(camera_node["transform"]);
            break;
          }
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "No camera transforms found in the file: %s", filename.c_str());
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s", filename.c_str(), e.what());
    }
  }

  /// @brief Parses a transformation matrix from a YAML node.
  Eigen::Matrix4d parseTransform(const YAML::Node &node) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int i = 0; i < node.size(); ++i) {
      for (int j = 0; j < node[i].size(); ++j) {
        transform(i, j) = node[i][j].as<double>();
      }
    }
    return transform;
  }

  /// @brief Configures ArUco detector parameters.
  void configureDetectorParameters() {
    detectorParams_ = cv::aruco::DetectorParameters::create();
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  }

  /// @brief Callback function for processing received images.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame, gray, filtered, enhanced, undistortedFrame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame received from the camera");
      return;
    }

    // Undistort the frame
    int height = frame.rows;
    int width = frame.cols;
    cv::Mat camMatrixNew;
    cv::Rect roi;
    camMatrixNew = cv::getOptimalNewCameraMatrix(camMatrix_, distCoeffs_, cv::Size(width, height), 1, cv::Size(width, height), &roi);
    cv::undistort(frame, undistortedFrame, camMatrixNew, distCoeffs_);

    // Convert to grayscale
    cv::cvtColor(undistortedFrame, gray, cv::COLOR_BGR2GRAY);

    // Reduce noise while preserving edges
    cv::bilateralFilter(gray, filtered, 3, 75, 20);

    // Adjust brightness and contrast using histogram equalization
    cv::equalizeHist(filtered, enhanced);

    // Optional: Apply adaptive thresholding for binary contrast enhancement
    cv::Mat binary;
    cv::adaptiveThreshold(enhanced, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

    // Detect markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(gray, dictionary_, markerCorners, markerIds, detectorParams_);

    // Refine corner locations to sub-pixel accuracy
    if (!markerCorners.empty()) {
      cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
      for (auto &corners : markerCorners) {
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      }
    }

    std::unordered_set<int> currently_detected_markers(markerIds.begin(), markerIds.end());

    // Handle lost markers
    for (int prev_marker : previously_detected_markers) {
      if (currently_detected_markers.find(prev_marker) == currently_detected_markers.end()) {
        RCLCPP_WARN(this->get_logger(), "Marker %d lost, reinitializing.", prev_marker);
        first_detection.erase(prev_marker);
        first_tvecs.erase(prev_marker);
        first_rvecs.erase(prev_marker);
      }
    }
    previously_detected_markers = currently_detected_markers;

    if (!markerIds.empty()) {
      std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());

      for (size_t i = 0; i < markerIds.size(); ++i) {
        int marker_id = markerIds[i];

        if (markerCorners[i].size() != 4) {
          RCLCPP_WARN(this->get_logger(), "Marker corners not valid for marker ID %d", marker_id);
          continue;
        }

        // Define 3D points of the marker corners
        std::vector<cv::Point3f> markerPoints = {
            cv::Point3f(-marker_length_ / 2, marker_length_ / 2, 0),
            cv::Point3f(marker_length_ / 2, marker_length_ / 2, 0),
            cv::Point3f(marker_length_ / 2, -marker_length_ / 2, 0),
            cv::Point3f(-marker_length_ / 2, -marker_length_ / 2, 0)
        };

        if (!cv::solvePnP(markerPoints, markerCorners[i], camMatrixNew, distCoeffs_, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE)) {
          RCLCPP_WARN(this->get_logger(), "PnP failed for marker ID %d", marker_id);
          continue;
        }

        // Filter rotation vectors
        if (first_detection.count(marker_id) == 0) {
          first_detection[marker_id] = true;
          first_tvecs[marker_id] = tvecs[i];
          first_rvecs[marker_id] = rvecs[i];
        } else {
          const double rotation_threshold = 0.02;
          const double zero_crossing_threshold = 0.02;
          cv::Vec3d previous_rvec = first_rvecs[marker_id];

          for (int j = 0; j < 3; j++) {
            double current = rvecs[i][j];
            double previous = previous_rvec[j];
            double diff = std::abs(current - previous);
            bool sign_changed = (current * previous < 0);

            if (sign_changed) {
              if (diff < zero_crossing_threshold) {
                // Small oscillation, no action needed
              } else if (diff > rotation_threshold) {
                rvecs[i][j] = -current;
              }
            }
          }
          first_tvecs[marker_id] = tvecs[i];
          first_rvecs[marker_id] = rvecs[i];
        }

        // Publish transform
        publishTransform(rvecs[i], tvecs[i], marker_id);

        // Draw markers and axes
        cv::aruco::drawDetectedMarkers(undistortedFrame, markerCorners, markerIds);
        std::vector<cv::Point3f> axisPoints = {
            cv::Point3f(0, 0, 0),
            cv::Point3f(marker_length_ * 1.5f, 0, 0),
            cv::Point3f(0, marker_length_ * 1.5f, 0),
            cv::Point3f(0, 0, marker_length_ * 1.5f)
        };
        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix_, distCoeffs_, imagePoints);
        cv::line(undistortedFrame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X - Red
        cv::line(undistortedFrame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y - Green
        cv::line(undistortedFrame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z - Blue
      }
    }

    // Visualize world origin and principal point
    Eigen::Matrix4d world_to_camera = camera_transform_.inverse();
    Eigen::Matrix3d R = world_to_camera.block<3, 3>(0, 0);
    Eigen::Vector3d t = world_to_camera.block<3, 1>(0, 3);
    cv::Mat R_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        R_cv.at<double>(i, j) = R(i, j);
    cv::Mat rvec_cv;
    cv::Rodrigues(R_cv, rvec_cv);
    cv::Vec3d rvec(rvec_cv.at<double>(0), rvec_cv.at<double>(1), rvec_cv.at<double>(2));
    cv::Vec3d tvec(t(0), t(1), t(2));

    std::vector<cv::Point3f> objectPoints = {cv::Point3f(0, 0, 0)};
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, camMatrix_, distCoeffs_, imagePoints);
    cv::circle(undistortedFrame, cv::Point(static_cast<int>(imagePoints[0].x), static_cast<int>(imagePoints[0].y)), 5, cv::Scalar(255, 0, 255), -1);

    double cx = camMatrix_.at<double>(0, 2);
    double cy = camMatrix_.at<double>(1, 2);
    cv::circle(undistortedFrame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5, cv::Scalar(0, 255, 255), -1);
  
    // Define 3D points for the world origin axes
    std::vector<cv::Point3f> worldAxes = {
        cv::Point3f(0, 0, 0),          // Origin
        cv::Point3f(0.05, 0, 0),       // X-axis (red)
        cv::Point3f(0, 0.05, 0),       // Y-axis (green)
        cv::Point3f(0, 0, 0.05)        // Z-axis (blue)
    };

    // Project the 3D points to the image plane
    std::vector<cv::Point2f> imagePoints2;
    cv::projectPoints(worldAxes, rvec, tvec, camMatrix_, distCoeffs_, imagePoints2);

    // Draw the coordinate frame at the world origin
    cv::line(undistortedFrame, imagePoints2[0], imagePoints2[1], cv::Scalar(0, 0, 255), 3); // X-axis (Red)
    cv::line(undistortedFrame, imagePoints2[0], imagePoints2[2], cv::Scalar(0, 255, 0), 3); // Y-axis (Green)
    cv::line(undistortedFrame, imagePoints2[0], imagePoints2[3], cv::Scalar(255, 0, 0), 3); // Z-axis (Blue)


    // Display the processed image
    cv::imshow("Detected ArUco markers", undistortedFrame);
    cv::setMouseCallback("Detected ArUco markers", onMouse, this);
    cv::waitKey(1);
  }

  /// @brief Static mouse callback function to display cursor coordinates.
  static void onMouse(int event, int x, int y, int flags, void* userdata) {
    ArucoDetectorSingle* self = static_cast<ArucoDetectorSingle*>(userdata);
    if (event == cv::EVENT_MOUSEMOVE) {
      std::stringstream ss;
      ss << "Detected ArUco markers - Cursor: (" << x << ", " << y << ")";
      cv::setWindowTitle("Detected ArUco markers", ss.str());
    }
  }

  /// @brief Publishes the transform of a detected marker.
  void publishTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec, int marker_id) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
      }
      camera_to_marker(row, 3) = tvec[row];
    }

    Eigen::Matrix4d fixed_to_marker = camera_transform_ * camera_to_marker;

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);
    transformStamped.transform.translation.x = fixed_to_marker(0, 3);
    transformStamped.transform.translation.y = fixed_to_marker(1, 3);
    transformStamped.transform.translation.z = fixed_to_marker(2, 3);

    Eigen::Matrix3d rotation = fixed_to_marker.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion(rotation);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    tf_broadcaster_.sendTransform(transformStamped);
    if (marker_id == 0) {
      tf_detected_publisher_->publish(transformStamped);
    }
  }
};

/// @brief Main function to initialize and spin the node.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorSingle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}