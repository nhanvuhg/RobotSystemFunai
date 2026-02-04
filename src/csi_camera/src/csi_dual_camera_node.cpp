#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>

using std::placeholders::_1;

class CSIDualCameraNode : public rclcpp::Node
{
public:
    CSIDualCameraNode() : Node("csi_dual_camera_node"), running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "CSI Dual Camera Node (Parallel Mode)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // Parameters
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 30);
        this->declare_parameter("cam0_topic", std::string("cam0Funai/image_raw"));
        this->declare_parameter("cam1_topic", std::string("cam1Funai/image_raw"));
        
        target_width_ = this->get_parameter("width").as_int();
        target_height_ = this->get_parameter("height").as_int();
        target_fps_ = this->get_parameter("fps").as_int();
        cam0_topic_ = this->get_parameter("cam0_topic").as_string();
        cam1_topic_ = this->get_parameter("cam1_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Config: %dx%d @ %d fps", 
                    target_width_, target_height_, target_fps_);

        // Initialize publishers
        pub_cam0_ = this->create_publisher<sensor_msgs::msg::Image>(cam0_topic_, 10);
        pub_cam1_ = this->create_publisher<sensor_msgs::msg::Image>(cam1_topic_, 10);
        
        RCLCPP_INFO(this->get_logger(), "Publishers: %s, %s", 
                    cam0_topic_.c_str(), cam1_topic_.c_str());

        // Cleanup any old camera processes
        full_system_cleanup();
        
        RCLCPP_INFO(this->get_logger(), "🚀 Initializing CAM0 and CAM1...");
        
        // Start both camera streams
        if (!start_camera(0, rpicam_pipe_cam0_)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start CAM0!");
            return;
        }
        
        if (!start_camera(1, rpicam_pipe_cam1_)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start CAM1!");
            stop_camera(rpicam_pipe_cam0_);
            return;
        }
        
        // Calculate YUV420 frame size
        yuv_frame_size_ = target_width_ * target_height_ * 3 / 2;
        
        // Start capture threads
        thread_cam0_ = std::thread(&CSIDualCameraNode::capture_loop_cam0, this);
        thread_cam1_ = std::thread(&CSIDualCameraNode::capture_loop_cam1, this);
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "✅ Dual Camera Node Ready!");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~CSIDualCameraNode()
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Shutting down dual camera node...");
        running_.store(false);
        
        // Wait for threads to finish
        if (thread_cam0_.joinable()) {
            thread_cam0_.join();
        }
        if (thread_cam1_.joinable()) {
            thread_cam1_.join();
        }
        
        // Stop camera streams
        stop_camera(rpicam_pipe_cam0_);
        stop_camera(rpicam_pipe_cam1_);
        
        RCLCPP_INFO(this->get_logger(), "✅ Cleanup complete");
    }

private:
    void full_system_cleanup()
    {
        RCLCPP_INFO(this->get_logger(), "  🧹 Cleaning up old processes...");
        std::system("pkill -9 rpicam-hello 2>/dev/null");
        std::system("pkill -9 rpicam-vid 2>/dev/null");
        std::system("pkill -9 rpicam-still 2>/dev/null");
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        RCLCPP_INFO(this->get_logger(), "  ✓ Cleanup done");
    }
    
    bool start_camera(int cam_id, FILE*& pipe)
    {
        RCLCPP_INFO(this->get_logger(), "  🎥 Starting CAM%d stream...", cam_id);
        
        // Build rpicam-vid command
        std::string cmd = "rpicam-vid --camera " + std::to_string(cam_id) + 
                         " -t 0 --nopreview --codec yuv420 " +
                         "--width " + std::to_string(target_width_) + " " +
                         "--height " + std::to_string(target_height_) + " " +
                         "--framerate " + std::to_string(target_fps_) + " " +
                         "--flush -o - 2>/dev/null";
        
        RCLCPP_INFO(this->get_logger(), "  📹 CMD: %s", cmd.c_str());
        
        pipe = popen(cmd.c_str(), "r");
        
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "  ❌ Failed to launch rpicam-vid for CAM%d!", cam_id);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "  ✓ CAM%d opened! YUV420: %zu bytes/frame", cam_id, yuv_frame_size_);
        return true;
    }
    
    void stop_camera(FILE*& pipe)
    {
        if (pipe) {
            pclose(pipe);
            pipe = nullptr;
        }
    }

    void capture_loop_cam0()
    {
        RCLCPP_INFO(this->get_logger(), "🎬 CAM0 capture thread started");
        
        std::vector<uint8_t> yuv_buffer(yuv_frame_size_);
        cv::Mat yuv_mat(target_height_ * 3 / 2, target_width_, CV_8UC1);
        cv::Mat bgr_frame(target_height_, target_width_, CV_8UC3);
        
        while (running_.load() && rclcpp::ok()) {
            if (!capture_and_publish(rpicam_pipe_cam0_, yuv_buffer, yuv_mat, bgr_frame, 
                                    pub_cam0_, "camera_input_tray", 0)) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "🛑 CAM0 capture thread stopped");
    }

    void capture_loop_cam1()
    {
        RCLCPP_INFO(this->get_logger(), "🎬 CAM1 capture thread started");
        
        std::vector<uint8_t> yuv_buffer(yuv_frame_size_);
        cv::Mat yuv_mat(target_height_ * 3 / 2, target_width_, CV_8UC1);
        cv::Mat bgr_frame(target_height_, target_width_, CV_8UC3);
        
        while (running_.load() && rclcpp::ok()) {
            if (!capture_and_publish(rpicam_pipe_cam1_, yuv_buffer, yuv_mat, bgr_frame, 
                                    pub_cam1_, "camera_output_tray", 1)) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "🛑 CAM1 capture thread stopped");
    }

    bool capture_and_publish(FILE* pipe, std::vector<uint8_t>& yuv_buffer, 
                            cv::Mat& yuv_mat, cv::Mat& bgr_frame,
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
                            const std::string& frame_id, int cam_id)
    {
        if (!pipe) {
            return false;
        }

        // Check if data is available (non-blocking)
        fd_set readfds;
        FD_ZERO(&readfds);
        int fd = fileno(pipe);
        FD_SET(fd, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 5000; // 5ms timeout

        int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);
        
        if (ready <= 0) {
            return false; // No data or error
        }

        // Read YUV420 frame data
        size_t bytes_read = fread(yuv_buffer.data(), 1, yuv_frame_size_, pipe);
        
        if (bytes_read != yuv_frame_size_) {
            // Incomplete frame - normal at boundaries
            return false;
        }

        // Zero-copy conversion: reuse pre-allocated matrices
        yuv_mat.data = yuv_buffer.data();
        cv::cvtColor(yuv_mat, bgr_frame, cv::COLOR_YUV2BGR_I420);

        // Prepare message header
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id;

        // Publish frame
        auto msg = cv_bridge::CvImage(header, "bgr8", bgr_frame).toImageMsg();
        
        if (publisher) {
            publisher->publish(*msg);
        }
        
        return true;
    }

    // Members
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam1_;
    
    FILE* rpicam_pipe_cam0_ = nullptr;
    FILE* rpicam_pipe_cam1_ = nullptr;
    
    std::thread thread_cam0_;
    std::thread thread_cam1_;
    std::atomic<bool> running_;
    
    int target_width_;
    int target_height_;
    int target_fps_;
    size_t yuv_frame_size_;
    
    std::string cam0_topic_;
    std::string cam1_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSIDualCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
================================================================================
🎯 DUAL CAMERA NODE - PARALLEL MODE
================================================================================

This node runs TWO cameras simultaneously on Raspberry Pi 5:
- CAM0 (Input Tray): --camera 0
- CAM1 (Output Tray): --camera 1

Key Features:
1. **Parallel Capture**: Two independent threads, one per camera
2. **No Switching**: Both cameras always active
3. **Independent Publishers**: 
   - cam0Funai/image_raw (Input Tray)
   - cam1Funai/image_raw (Output Tray)
4. **Thread-safe**: Each thread has its own buffers

Performance:
- 2x rpicam-vid processes
- 2x capture threads
- Minimal overhead with zero-copy YUV→BGR conversion

Hardware:
- Arducam GMSL2 8MP Camera Extension Kit
- 2x IMX219 cameras
- Raspberry Pi 5 with dual CSI support

Usage:
  ros2 run csi_camera csi_dual_camera_node

Topics:
  ros2 topic hz /cam0Funai/image_raw
  ros2 topic hz /cam1Funai/image_raw

================================================================================
*/
