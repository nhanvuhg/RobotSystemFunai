#!/bin/bash
# ==============================================================================
# Install Fixed Camera Node - Complete Replacement
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Installing Fixed Camera Node${NC}"
echo -e "${GREEN}========================================${NC}"

PACKAGE_PATH="${HOME}/ros2_ws/src/csi_camera"
SRC_FILE="${PACKAGE_PATH}/src/csi_camera_node.cpp"

if [ ! -d "$PACKAGE_PATH" ]; then
    echo -e "${RED}ERROR: Package not found at $PACKAGE_PATH${NC}"
    exit 1
fi

# Backup
echo -e "\n${BLUE}[1/3] Creating backup...${NC}"
BACKUP_FILE="${SRC_FILE}.backup_$(date +%Y%m%d_%H%M%S)"
if [ -f "$SRC_FILE" ]; then
    cp "$SRC_FILE" "$BACKUP_FILE"
    echo -e "${GREEN}✓ Backed up to: $(basename $BACKUP_FILE)${NC}"
fi

# Delete old file completely
echo -e "\n${BLUE}[2/3] Removing old file...${NC}"
rm -f "$SRC_FILE"
echo -e "${GREEN}✓ Old file removed${NC}"

# Create new file
echo -e "\n${BLUE}[3/3] Creating new file...${NC}"

cat > "$SRC_FILE" << 'EOFCPP'
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cstdlib>
#include <mutex>
#include <atomic>

using std::placeholders::_1;

struct CameraMode {
    int width;
    int height;
    int fps;
};

class CSICameraNode : public rclcpp::Node
{
public:
    CSICameraNode() : Node("csi_camera_node"), is_switching_(false), current_cam_id_(0)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "CSI Camera Node Starting");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        this->declare_parameter("use_rpicam", false);
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 30);
        this->declare_parameter("output_topic", std::string("/ai/image_overlay"));
        
        use_rpicam_ = this->get_parameter("use_rpicam").as_bool();
        target_width_ = this->get_parameter("width").as_int();
        target_height_ = this->get_parameter("height").as_int();
        target_fps_ = this->get_parameter("fps").as_int();
        output_topic_ = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Output: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target: %dx%d @ %d fps", 
                    target_width_, target_height_, target_fps_);

        sub_camera_select_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot/camera_select", 10,
            std::bind(&CSICameraNode::camera_select_callback, this, _1));

        pub_active_id_ = this->create_publisher<std_msgs::msg::Int32>(
            "/camera/active_id", 10);

        full_system_cleanup();
        
        RCLCPP_INFO(this->get_logger(), "Initializing Camera 0...");
        
        if (!safe_hardware_switch(0)) {
            RCLCPP_ERROR(this->get_logger(), "Hardware switch failed!");
            return;
        }
        
        detect_camera_capabilities();
        start_stream();
        
        if (cap_.isOpened()) {
            publish_active_id();
            
            int interval_ms = 1000 / actual_fps_;
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                std::bind(&CSICameraNode::capture_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "Camera Node Ready!");
            RCLCPP_INFO(this->get_logger(), "========================================");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
        }
    }

    ~CSICameraNode()
    {
        is_switching_.store(true);
        if (timer_) timer_->cancel();
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        {
            std::lock_guard<std::mutex> lock(cap_mutex_);
            if (cap_.isOpened()) cap_.release();
        }
        
        if (pub_active_) pub_active_.reset();
    }

private:
    void detect_camera_capabilities()
    {
        RCLCPP_INFO(this->get_logger(), "Detecting camera...");
        
        std::string cmd = "libcamera-hello --list-cameras 2>&1";
        FILE* pipe = popen(cmd.c_str(), "r");
        
        if (!pipe) {
            set_default_resolution();
            return;
        }
        
        char buffer[512];
        std::string output;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            output += buffer;
        }
        pclose(pipe);
        
        std::vector<CameraMode> modes;
        
        size_t pos = 0;
        while ((pos = output.find("x", pos)) != std::string::npos) {
            size_t start = pos;
            while (start > 0 && std::isdigit(output[start-1])) start--;
            
            int w = 0, h = 0;
            float fps = 0;
            
            size_t i = start;
            while (i < output.length() && std::isdigit(output[i])) {
                w = w * 10 + (output[i] - '0');
                i++;
            }
            
            if (i < output.length() && output[i] == 'x') i++;
            
            while (i < output.length() && std::isdigit(output[i])) {
                h = h * 10 + (output[i] - '0');
                i++;
            }
            
            size_t fps_pos = output.find('[', i);
            if (fps_pos != std::string::npos && fps_pos < i + 50) {
                fps_pos++;
                if (fps == 0) {
                    sscanf(output.c_str() + fps_pos, "%f", &fps);
                }
            }
            
            if (w > 0 && h > 0 && fps > 0) {
                CameraMode mode;
                mode.width = w;
                mode.height = h;
                mode.fps = (int)fps;
                modes.push_back(mode);
                RCLCPP_INFO(this->get_logger(), "  Found: %dx%d @ %d fps", w, h, (int)fps);
            }
            
            pos++;
        }
        
        if (modes.empty()) {
            set_default_resolution();
            return;
        }
        
        select_best_mode(modes);
    }
    
    void set_default_resolution()
    {
        actual_width_ = 2028;
        actual_height_ = 1520;
        actual_fps_ = 40;
        RCLCPP_INFO(this->get_logger(), "  Using default: %dx%d @ %d fps",
                    actual_width_, actual_height_, actual_fps_);
    }
    
    void select_best_mode(const std::vector<CameraMode>& modes)
    {
        RCLCPP_INFO(this->get_logger(), "Selecting best mode...");
        
        CameraMode best = modes[0];
        int best_score = -1000000;
        
        for (const auto& mode : modes) {
            int score = 0;
            
            if (mode.width >= target_width_ && mode.height >= target_height_) {
                score += 10000;
                score -= (mode.width - target_width_) / 10;
                score -= (mode.height - target_height_) / 10;
            } else {
                score -= 50000;
            }
            
            if (mode.fps >= target_fps_) {
                score += 1000;
                score -= std::abs(mode.fps - target_fps_) * 10;
            } else {
                score -= std::abs(mode.fps - target_fps_) * 50;
            }
            
            if (score > best_score) {
                best_score = score;
                best = mode;
            }
        }
        
        actual_width_ = best.width;
        actual_height_ = best.height;
        actual_fps_ = best.fps;
        
        RCLCPP_INFO(this->get_logger(), "Selected: %dx%d @ %d fps",
                    actual_width_, actual_height_, actual_fps_);
        
        if (actual_width_ != target_width_ || actual_height_ != target_height_) {
            RCLCPP_WARN(this->get_logger(), 
                "Will resize: %dx%d -> %dx%d",
                actual_width_, actual_height_, target_width_, target_height_);
        }
    }
    
    void full_system_cleanup()
    {
        std::system("pkill -9 libcamera-hello 2>/dev/null");
        std::system("pkill -9 libcamera-vid 2>/dev/null");
        std::system("pkill -9 rpicam-hello 2>/dev/null");
        std::system("pkill -9 rpicam-vid 2>/dev/null");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        std::system("fuser -k /dev/video* 2>/dev/null");
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    
    bool safe_hardware_switch(int cam_id)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to Camera %d", cam_id);
        
        std::string cmd;
        if (cam_id == 0) {
            cmd = "i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x01";
        } else if (cam_id == 1) {
            cmd = "i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x02";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera: %d", cam_id);
            return false;
        }
        
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "I2C failed: %d", ret);
            return false;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(3000));
        
        std::string check_cmd = "libcamera-hello --list-cameras 2>&1 | grep -q 'Available cameras'";
        for (int i = 0; i < 5; i++) {
            if (std::system(check_cmd.c_str()) == 0) {
                RCLCPP_INFO(this->get_logger(), "  Camera detected");
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
        
        RCLCPP_ERROR(this->get_logger(), "Camera not detected");
        return false;
    }
    
    void camera_select_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int target_cam = msg->data;
        
        if (target_cam != 0 && target_cam != 1) {
            RCLCPP_WARN(this->get_logger(), "Invalid camera: %d", target_cam);
            return;
        }

        if (target_cam == current_cam_id_) {
            RCLCPP_INFO(this->get_logger(), "Already on Camera %d", target_cam);
            return;
        }

        bool expected = false;
        if (!is_switching_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(this->get_logger(), "Switch in progress");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Switch: Camera %d -> %d", 
                    current_cam_id_, target_cam);
        
        if (timer_) timer_->cancel();
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        if (pub_active_) pub_active_.reset();
        
        {
            std::lock_guard<std::mutex> lock(cap_mutex_);
            if (cap_.isOpened()) cap_.release();
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        full_system_cleanup();
        
        if (!safe_hardware_switch(target_cam)) {
            is_switching_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Switch failed!");
            return;
        }
        
        current_cam_id_ = target_cam;
        detect_camera_capabilities();
        start_stream();
        
        if (cap_.isOpened()) {
            publish_active_id();
            is_switching_.store(false);
            
            int interval_ms = 1000 / actual_fps_;
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                std::bind(&CSICameraNode::capture_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "Camera %d active!", target_cam);
        } else {
            is_switching_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Failed to open Camera %d!", target_cam);
        }
    }
    
    void start_stream()
    {
        RCLCPP_INFO(this->get_logger(), "Starting stream...");
        
        if (pub_active_) pub_active_.reset();
        
        pub_active_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic_, 10);
        
        std::string pipeline = 
            "libcamerasrc ! "
            "video/x-raw, "
            "width=" + std::to_string(actual_width_) + ", "
            "height=" + std::to_string(actual_height_) + ", "
            "framerate=" + std::to_string(actual_fps_) + "/1 ! ";
        
        if (actual_width_ != target_width_ || actual_height_ != target_height_) {
            pipeline += 
                "videoscale ! "
                "video/x-raw, "
                "width=" + std::to_string(target_width_) + ", "
                "height=" + std::to_string(target_height_) + " ! ";
        }
        
        pipeline +=
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true sync=false max-buffers=2";
        
        for (int attempt = 1; attempt <= 3; ++attempt) {
            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                cap_.open(pipeline, cv::CAP_GSTREAMER);
            }
            
            if (cap_.isOpened()) {
                cv::Mat test_frame;
                bool got_frame = false;
                
                for (int i = 0; i < 10; ++i) {
                    {
                        std::lock_guard<std::mutex> lock(cap_mutex_);
                        if (cap_.read(test_frame) && !test_frame.empty()) {
                            got_frame = true;
                            break;
                        }
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }
                
                if (got_frame) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Camera opened! %dx%d", test_frame.cols, test_frame.rows);
                    return;
                }
                
                std::lock_guard<std::mutex> lock(cap_mutex_);
                cap_.release();
            }
            
            if (attempt < 3) {
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    }

    void capture_loop()
    {
        if (is_switching_.load()) return;

        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) return;

        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) return;

        if (pub_active_) {
            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = (current_cam_id_ == 0) ? 
                "camera_input_tray" : "camera_output_tray";

            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub_active_->publish(*msg);
        }
    }

    void publish_active_id()
    {
        if (!pub_active_id_) return;
        
        std_msgs::msg::Int32 msg;
        msg.data = current_cam_id_;
        pub_active_id_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_active_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_camera_select_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_active_id_;
    
    cv::VideoCapture cap_;
    std::mutex cap_mutex_;
    std::atomic<bool> is_switching_;
    int current_cam_id_;
    
    bool use_rpicam_;
    int target_width_;
    int target_height_;
    int target_fps_;
    int actual_width_;
    int actual_height_;
    int actual_fps_;
    std::string output_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSICameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
EOFCPP

echo -e "${GREEN}✓ New file created ($(wc -l < "$SRC_FILE") lines)${NC}"

# Verify
echo -e "\n${BLUE}Verifying file...${NC}"
if grep -q "struct CameraMode" "$SRC_FILE"; then
    echo -e "${GREEN}✓ CameraMode struct found${NC}"
else
    echo -e "${RED}❌ CameraMode struct missing!${NC}"
    exit 1
fi

if grep -q "class CSICameraNode" "$SRC_FILE"; then
    echo -e "${GREEN}✓ CSICameraNode class found${NC}"
else
    echo -e "${RED}❌ CSICameraNode class missing!${NC}"
    exit 1
fi

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e ""
echo -e "${YELLOW}Now build:${NC}"
echo -e "  ${BLUE}cd ~/ros2_ws${NC}"
echo -e "  ${BLUE}colcon build --packages-select csi_camera --symlink-install${NC}"
echo -e ""