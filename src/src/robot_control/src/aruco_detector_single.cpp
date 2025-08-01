/// @file multi_camera_viewer.cpp
/// @brief Node that displays multiple camera feeds simultaneously in a grid layout.

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>

/// @class MultiCameraViewer
/// @brief Subscribes to multiple camera topics and displays them in a grid layout.
class MultiCameraViewer : public rclcpp::Node {
public:
  /// @brief Constructor for MultiCameraViewer.
  MultiCameraViewer() : Node("multi_camera_viewer") {
    // Initialize camera topics - adjust these to match your actual camera topics
    camera_topics_ = {
      "camera1/image_raw",
      "camera2/image_raw", 
      "camera3/image_raw",
      "camera4/image_raw",
      "camera5/image_raw",
      "camera6/image_raw",
      "camera7/image_raw"
    };
    
    // Initialize camera names for display
    camera_names_ = {
      "Camera 1", "Camera 2", "Camera 3", "Camera 4", 
      "Camera 5", "Camera 6", "Camera 7"
    };
    
    // Resize vectors to hold images and status
    latest_images_.resize(camera_topics_.size());
    camera_active_.resize(camera_topics_.size(), false);
    
    // Create subscribers for each camera
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      auto subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topics_[i], 10,
        [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
          this->imageCallback(msg, i);
        });
      image_subscribers_.push_back(subscriber);
      
      RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", camera_topics_[i].c_str());
    }
    
    // Set display parameters
    display_width_ = 1200;
    display_height_ = 900;
    grid_cols_ = 3;
    grid_rows_ = 3;
    
    // Calculate individual image size for grid display
    cell_width_ = display_width_ / grid_cols_;
    cell_height_ = display_height_ / grid_rows_;
    
    // Create main display window
    cv::namedWindow("Multi-Camera View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Camera View", display_width_, display_height_);
    
    // Create timer for periodic display updates
    display_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), // ~30 FPS
      std::bind(&MultiCameraViewer::updateDisplay, this));
      
    RCLCPP_INFO(this->get_logger(), "Multi-Camera Viewer initialized");
    RCLCPP_INFO(this->get_logger(), "Grid layout: %dx%d, Cell size: %dx%d", 
                grid_cols_, grid_rows_, cell_width_, cell_height_);
  }

private:
  /// @brief Callback function for processing received images from a specific camera.
  /// @param msg The image message received.
  /// @param camera_index Index of the camera that sent the image.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, size_t camera_index) {
    try {
      // Convert ROS image to OpenCV format
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      
      // Store the latest image for this camera
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_images_[camera_index] = frame.clone();
        camera_active_[camera_index] = true;
      }
      
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception for camera %zu: %s", 
                   camera_index, e.what());
    }
  }
  
  /// @brief Creates a placeholder image for inactive cameras.
  /// @param camera_index Index of the camera.
  /// @return Placeholder image with camera information.
  cv::Mat createPlaceholderImage(size_t camera_index) {
    cv::Mat placeholder(cell_height_, cell_width_, CV_8UC3, cv::Scalar(50, 50, 50));
    
    // Add text information
    std::string text1 = camera_names_[camera_index];
    std::string text2 = "No Signal";
    std::string text3 = "Topic: " + camera_topics_[camera_index];
    
    // Calculate text positions
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    
    cv::Size text_size1 = cv::getTextSize(text1, font, font_scale, thickness, nullptr);
    cv::Size text_size2 = cv::getTextSize(text2, font, font_scale, thickness, nullptr);
    cv::Size text_size3 = cv::getTextSize(text3, font, 0.4, 1, nullptr);
    
    cv::Point text_pos1((cell_width_ - text_size1.width) / 2, cell_height_ / 2 - 20);
    cv::Point text_pos2((cell_width_ - text_size2.width) / 2, cell_height_ / 2 + 10);
    cv::Point text_pos3((cell_width_ - text_size3.width) / 2, cell_height_ / 2 + 35);
    
    // Draw text
    cv::putText(placeholder, text1, text_pos1, font, font_scale, cv::Scalar(255, 255, 255), thickness);
    cv::putText(placeholder, text2, text_pos2, font, font_scale, cv::Scalar(0, 0, 255), thickness);
    cv::putText(placeholder, text3, text_pos3, font, 0.4, cv::Scalar(200, 200, 200), 1);
    
    return placeholder;
  }
  
  /// @brief Updates the main display with all camera feeds.
  void updateDisplay() {
    // Create the main display image
    cv::Mat display_image(display_height_, display_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      // Calculate grid position
      int row = i / grid_cols_;
      int col = i % grid_cols_;
      
      // Calculate region of interest in the display image
      cv::Rect roi(col * cell_width_, row * cell_height_, cell_width_, cell_height_);
      
      cv::Mat cell_image;
      
      if (camera_active_[i] && !latest_images_[i].empty()) {
        // Resize the camera image to fit the cell
        cv::resize(latest_images_[i], cell_image, cv::Size(cell_width_, cell_height_));
        
        // Add camera label
        std::string label = camera_names_[i];
        cv::putText(cell_image, label, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                   
        // Add timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
        ss << "." << std::setfill('0') << std::setw(3) << ms.count();
        
        cv::putText(cell_image, ss.str(), cv::Point(10, cell_height_ - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
      } else {
        // Create placeholder for inactive camera
        cell_image = createPlaceholderImage(i);
      }
      
      // Copy the cell image to the display
      cell_image.copyTo(display_image(roi));
      
      // Draw border around each cell
      cv::rectangle(display_image, roi, cv::Scalar(100, 100, 100), 2);
    }
    
    // Add main title
    cv::putText(display_image, "Multi-Camera Monitoring System", 
               cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
               cv::Scalar(255, 255, 255), 2);
    
    // Count active cameras
    int active_count = std::count(camera_active_.begin(), camera_active_.end(), true);
    std::string status = "Active Cameras: " + std::to_string(active_count) + "/" + 
                        std::to_string(camera_topics_.size());
    cv::putText(display_image, status, cv::Point(display_width_ - 250, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    
    // Display the combined image
    cv::imshow("Multi-Camera View", display_image);
    
    // Handle key presses
    char key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) { // 'q' or ESC to quit
      RCLCPP_INFO(this->get_logger(), "Shutting down Multi-Camera Viewer");
      rclcpp::shutdown();
    } else if (key == 'r') { // 'r' to reset camera status
      std::fill(camera_active_.begin(), camera_active_.end(), false);
      RCLCPP_INFO(this->get_logger(), "Reset camera status");
    }
  }
  
  // Member variables
  std::vector<std::string> camera_topics_;
  std::vector<std::string> camera_names_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
  std::vector<cv::Mat> latest_images_;
  std::vector<bool> camera_active_;
  
  std::mutex image_mutex_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  
  int display_width_, display_height_;
  int grid_cols_, grid_rows_;
  int cell_width_, cell_height_;
};

/// @brief Main function that initializes and spins the MultiCameraViewer node.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MultiCameraViewer>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Multi-Camera Viewer");
  RCLCPP_INFO(node->get_logger(), "Press 'q' or ESC to quit");
  RCLCPP_INFO(node->get_logger(), "Press 'r' to reset camera status");
  
  rclcpp::spin(node);
  
  // Cleanup
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}