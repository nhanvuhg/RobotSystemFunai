#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.hpp> 
#include <cstdlib> 

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class CSICameraNode : public rclcpp::Node
{
public:
    CSICameraNode() : Node("csi_camera_node"), current_cam_id_(1)
    {
        // Declare parameters
        this->declare_parameter("use_rpicam", false); // Mặc định GStreamer cho ổn định
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 30);
        
        use_rpicam_ = this->get_parameter("use_rpicam").as_bool();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        // Service để switch camera
        srv_switch_ = this->create_service<std_srvs::srv::SetBool>(
            "switch_camera", 
            std::bind(&CSICameraNode::handle_switch_request, this, _1, _2)
        );

        // Khởi động Camera 1 mặc định
        RCLCPP_INFO(this->get_logger(), "🚀 Starting with Camera 1...");
        RCLCPP_INFO(this->get_logger(), "  Mode: %s", use_rpicam_ ? "rpicam-vid" : "GStreamer");
        switch_hardware(1);
        start_stream();

        // Timer loop
        int interval_ms = 1000 / fps_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&CSICameraNode::capture_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "✅ CSI Camera Node ready.");
    }

    ~CSICameraNode()
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Shutting down camera...");
        if (cap_.isOpened()) {
            cap_.release();
        }
        // Giải phóng Publisher đang active
        if (pub_active_) {
            pub_active_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "✅ Camera released successfully.");
    }

private:
    void handle_switch_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        int target_cam = request->data ? 1 : 2;

        if (target_cam == current_cam_id_) {
            response->success = true;
            response->message = "Camera " + std::to_string(target_cam) + " is already active";
            RCLCPP_WARN(this->get_logger(), "⚠️  %s", response->message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🔄 Switching: Camera %d → Camera %d", 
                      current_cam_id_, target_cam);
        
        // Bước 1: Giải phóng stream (rất quan trọng)
        if (cap_.isOpened()) {
            cap_.release();
            RCLCPP_INFO(this->get_logger(), "  ✓ Released Camera %d stream", current_cam_id_);
        }
        
        // BƯỚC QUAN TRỌNG: CHỜ DRIVER DỌN DẸP
        rclcpp::sleep_for(std::chrono::milliseconds(200)); 

        // Bước 2: Switch hardware
        switch_hardware(target_cam);
        RCLCPP_INFO(this->get_logger(), "  ✓ Sent I2C command to Arducam multiplexer");
        
        // Bước 3: Đợi hardware ổn định (1.5 giây)
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        RCLCPP_INFO(this->get_logger(), "  ✓ Hardware stabilized");
        
        // Bước 4: Mở lại stream và tạo Publisher mới
        current_cam_id_ = target_cam; // Cập nhật trạng thái trước khi mở stream
        start_stream();
        
        // Bước 5: Cập nhật trạng thái
        if (cap_.isOpened()) {
            response->success = true;
            response->message = "✅ Switched to Camera " + std::to_string(target_cam);
            RCLCPP_INFO(this->get_logger(), "✅ Switch completed successfully!");
        } else {
            response->success = false;
            response->message = "❌ Failed to open Camera " + std::to_string(target_cam);
            RCLCPP_ERROR(this->get_logger(), "❌ Switch failed!");
        }
    }

    void switch_hardware(int cam_id)
    {
        // Lệnh I2C cho Arducam Multi-Camera Adapter HAT (Pi 5 dùng bus 4)
        std::string cmd;
        if (cam_id == 1) {
            cmd = "i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x01";
        } else if (cam_id == 2) {
            cmd = "i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x02";
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Invalid camera ID: %d", cam_id);
            return;
        }
        
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ I2C command failed with code: %d", ret);
        }
    }

    void start_stream()
    {
        // 1. NGẮT PUBLISHER CŨ VÀ CÁC THAM CHIẾU (Đảm bảo chỉ có 1 luồng dữ liệu)
        if (pub_active_) {
            pub_active_.reset(); 
        }

        // 2. TẠO TÊN TOPIC MỚI (ĐỘC QUYỀN)
        std::string output_topic;
        if (current_cam_id_ == 1) {
            output_topic = "/yolo/cam1/input";
        } else {
            output_topic = "/yolo/cam2/input";
        }
        
        // 3. TẠO PUBLISHER MỚI
        pub_active_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        RCLCPP_INFO(this->get_logger(), "  ✓ Publisher created for: %s", output_topic.c_str());
        
        // 4. Mở stream OpenCV
        std::string pipeline;
        
        if (use_rpicam_) {
            // OPTION A: rpicam-vid (Dễ bị treo khi switch)
            pipeline = "rpicam-vid -t 0 --inline --nopreview "
                        "--width " + std::to_string(width_) + " "
                        "--height " + std::to_string(height_) + " "
                        "--framerate " + std::to_string(fps_) + " "
                        "-o - | " 
                        "ffmpeg -i - -f rawvideo -pix_fmt bgr24 -an -sn pipe:1 2>/dev/null";
            cap_.open(pipeline, cv::CAP_FFMPEG);
            
        } else {
            // OPTION B: GStreamer với libcamerasrc (ỔN ĐỊNH CHO SWITCH)
            pipeline = 
                "libcamerasrc ! "
                "video/x-raw, format=YUY2, "
                "width=" + std::to_string(width_) + ", "
                "height=" + std::to_string(height_) + ", "
                "framerate=" + std::to_string(fps_) + "/1 ! "
                "queue ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink drop=1";
            cap_.open(pipeline, cv::CAP_GSTREAMER);
        }
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), 
                "❌ Failed to open camera stream!"
                "\n  Pipeline: %s", pipeline.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Camera stream opened successfully");
            RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d @ %d fps", width_, height_, fps_);
        }
    }

    void capture_loop()
    {
        if (!cap_.isOpened()) {
            static int error_count = 0;
            if (error_count++ % 100 == 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️  Camera stream not opened");
            }
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            static int read_error_count = 0;
            if (read_error_count++ % 30 == 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️  Failed to read frame");
            }
            return;
        }

        // Kiểm tra và publish
        if (pub_active_) {
            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = (current_cam_id_ == 1) ? "camera_1_link" : "camera_2_link";

            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub_active_->publish(*msg);
        }
    }

    // Members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_active_; // Publisher đang hoạt động
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_switch_;
    
    cv::VideoCapture cap_;
    int current_cam_id_;
    bool use_rpicam_;
    int width_;
    int height_;
    int fps_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSICameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}