#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <cstring>

using std::placeholders::_1;

struct CamProcess {
    pid_t pid = -1;
    int   fd  = -1;
};

static CamProcess launch_rpicam(int cam_id, int width, int height, int fps)
{
    int pipefd[2];
    if (pipe(pipefd) != 0) { perror("pipe"); return {}; }

    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        close(pipefd[0]);
        close(pipefd[1]);
        return {};
    }

    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[1]);

        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) { dup2(devnull, STDERR_FILENO); close(devnull); }

        std::string w = std::to_string(width);
        std::string h = std::to_string(height);
        std::string f = std::to_string(fps);
        std::string c = std::to_string(cam_id);

        execlp("rpicam-vid", "rpicam-vid",
               "--camera",    c.c_str(),
               "-t",          "0",
               "--nopreview",
               "--codec",     "yuv420",
               "--width",     w.c_str(),
               "--height",    h.c_str(),
               "--framerate", f.c_str(),
               "--denoise",   "cdn_off",   // FIX: giảm ISP load
               "--flush",
               "-o",          "-",
               nullptr);
        _exit(127);
    }

    close(pipefd[1]);

    int flags = fcntl(pipefd[0], F_GETFL, 0);
    fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK);

    // FIX: Tăng pipe buffer lên 1MB — giảm mất frame khi burst
    fcntl(pipefd[0], F_SETPIPE_SZ, 1 * 1024 * 1024);

    CamProcess cp;
    cp.pid = pid;
    cp.fd  = pipefd[0];
    return cp;
}

static void kill_cam_process(CamProcess& cp)
{
    if (cp.pid > 0) {
        kill(cp.pid, SIGKILL);
        waitpid(cp.pid, nullptr, 0);
        cp.pid = -1;
    }
    if (cp.fd >= 0) {
        close(cp.fd);
        cp.fd = -1;
    }
}

class CSIDualCameraNode : public rclcpp::Node
{
public:
    CSIDualCameraNode() : Node("csi_dual_camera_node"), running_(true)
    {
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "CSI Dual Camera Node");
        RCLCPP_INFO(get_logger(), "========================================");

        declare_parameter("width",      1280);
        declare_parameter("height",      720);
        declare_parameter("fps",          20);  // FIX: 30→20 giảm tải
        declare_parameter("cam0_topic", std::string("cam0Funai/image_raw"));
        declare_parameter("cam1_topic", std::string("cam1Funai/image_raw"));

        target_width_  = get_parameter("width").as_int();
        target_height_ = get_parameter("height").as_int();
        target_fps_    = get_parameter("fps").as_int();
        cam0_topic_    = get_parameter("cam0_topic").as_string();
        cam1_topic_    = get_parameter("cam1_topic").as_string();

        pub_cam0_ = create_publisher<sensor_msgs::msg::Image>(cam0_topic_, 10);
        pub_cam1_ = create_publisher<sensor_msgs::msg::Image>(cam1_topic_, 10);

        yuv_frame_size_ = (size_t)target_width_ * target_height_ * 3 / 2;

        full_system_cleanup();

        // Start CAM0
        RCLCPP_INFO(get_logger(), "🚀 Starting CAM0...");
        cam0_ = launch_rpicam(0, target_width_, target_height_, target_fps_);
        if (cam0_.pid < 0) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to start CAM0!"); return;
        }

        // Đợi CAM0 stream ổn định (frame thực sự, không phải delay cứng)
        if (!wait_for_first_frame(cam0_.fd, "CAM0", 15)) {
            kill_cam_process(cam0_); return;
        }

        // Start CAM1 chỉ sau khi CAM0 đã stream
        RCLCPP_INFO(get_logger(), "🚀 Starting CAM1...");
        cam1_ = launch_rpicam(1, target_width_, target_height_, target_fps_);
        if (cam1_.pid < 0) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to start CAM1!");
            kill_cam_process(cam0_); return;
        }

        wait_for_first_frame(cam1_.fd, "CAM1", 15);  // non-fatal nếu fail

        thread_cam0_ = std::thread(&CSIDualCameraNode::capture_loop, this, 0);
        thread_cam1_ = std::thread(&CSIDualCameraNode::capture_loop, this, 1);

        RCLCPP_INFO(get_logger(), "✅ Dual Camera Node Ready!");
    }

    ~CSIDualCameraNode()
    {
        running_.store(false);
        if (thread_cam0_.joinable()) thread_cam0_.join();
        if (thread_cam1_.joinable()) thread_cam1_.join();
        { std::lock_guard<std::mutex> lk(mtx_cam0_); kill_cam_process(cam0_); }
        { std::lock_guard<std::mutex> lk(mtx_cam1_); kill_cam_process(cam1_); }
    }

private:
    void full_system_cleanup()
    {
        std::system("pkill -9 rpicam-vid   2>/dev/null");
        std::system("pkill -9 rpicam-hello 2>/dev/null");
        std::system("pkill -9 rpicam-still 2>/dev/null");
        std::system("pkill -9 libcamera    2>/dev/null");  // FIX: thêm dòng này
        rclcpp::sleep_for(std::chrono::milliseconds(800));
    }

    // Helper: đợi frame thực sự từ camera, trả về false nếu timeout
    bool wait_for_first_frame(int fd, const char* name, int timeout_sec)
    {
        RCLCPP_INFO(get_logger(), "⏳ Waiting for %s first frame...", name);
        std::vector<uint8_t> buf(yuv_frame_size_);
        const auto timeout = std::chrono::steady_clock::now()
                             + std::chrono::seconds(timeout_sec);
        while (std::chrono::steady_clock::now() < timeout) {
            if (read_frame(fd, buf)) {
                RCLCPP_INFO(get_logger(), "✅ %s streaming", name);
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        RCLCPP_ERROR(get_logger(), "❌ %s never started streaming!", name);
        return false;
    }

    // FIX: timeout 200ms → 500ms, tránh false disconnect
    bool read_frame(int fd, std::vector<uint8_t>& buf)
    {
        if (fd < 0 || yuv_frame_size_ == 0) return false;

        size_t total = 0;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

        while (total < yuv_frame_size_) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline) return false;

            auto remaining_us =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    deadline - now).count();

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = static_cast<suseconds_t>(
                std::min<long>(remaining_us, 40000));

            int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
            if (ret < 0 && errno == EINTR) continue;
            if (ret <= 0) return false;

            ssize_t n = read(fd, buf.data() + total, yuv_frame_size_ - total);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                return false;
            }
            if (n == 0) return false;
            total += static_cast<size_t>(n);
        }
        return true;
    }

    void capture_loop(int cam_id)
    {
        RCLCPP_INFO(get_logger(), "🎬 CAM%d capture thread started", cam_id);

        std::vector<uint8_t> yuv_buf(yuv_frame_size_);
        cv::Mat bgr(target_height_, target_width_, CV_8UC3);

        // FIX: threshold 3s → 6s, tránh reconnect quá sớm
        const int FAIL_THRESHOLD = (target_fps_ > 0 ? target_fps_ : 20) * 6;
        int fails = 0;

        auto startup_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(10);
        bool grace_warned = false;

        while (running_.load() && rclcpp::ok()) {

            // FIX: Giữ lock trong suốt quá trình đọc
            // → fd không thể bị close bởi reconnect/destructor trong lúc read
            bool frame_ok;
            {
                std::lock_guard<std::mutex> lk(
                    (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                int fd = (cam_id == 0) ? cam0_.fd : cam1_.fd;
                frame_ok = read_frame(fd, yuv_buf);
            }

            if (!frame_ok) {
                if (std::chrono::steady_clock::now() < startup_deadline) {
                    if (!grace_warned) {
                        RCLCPP_INFO(get_logger(),
                            "⏳ CAM%d: waiting for stream...", cam_id);
                        grace_warned = true;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                fails++;
                if (fails < FAIL_THRESHOLD) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                RCLCPP_WARN(get_logger(),
                    "⚠️  CAM%d: %d consecutive failures — reconnecting...",
                    cam_id, fails);

                // FIX: Global reconnect lock — chỉ 1 cam reconnect tại 1 thời điểm
                // Ngăn CAM1 reconnect cùng lúc CAM0, tránh CSI bus conflict
                {
                    std::lock_guard<std::mutex> recon_lk(mtx_reconnect_);

                    {
                        std::lock_guard<std::mutex> lk(
                            (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                        CamProcess& cp = (cam_id == 0) ? cam0_ : cam1_;
                        kill_cam_process(cp);
                    }

                    rclcpp::sleep_for(std::chrono::milliseconds(1000));

                    CamProcess new_cp = launch_rpicam(
                        cam_id, target_width_, target_height_, target_fps_);

                    {
                        std::lock_guard<std::mutex> lk(
                            (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                        CamProcess& cp = (cam_id == 0) ? cam0_ : cam1_;
                        if (new_cp.pid > 0) {
                            cp = new_cp;
                            RCLCPP_INFO(get_logger(),
                                "✅ CAM%d reconnected (pid=%d)", cam_id, cp.pid);
                        } else {
                            RCLCPP_ERROR(get_logger(),
                                "❌ CAM%d reconnect failed — retry in 2s", cam_id);
                        }
                    }

                    if (new_cp.pid > 0) {
                        startup_deadline = std::chrono::steady_clock::now()
                                           + std::chrono::seconds(10);
                        grace_warned = false;
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    } else {
                        rclcpp::sleep_for(std::chrono::milliseconds(2000));
                    }
                }

                fails = 0;
                continue;
            }

            // Frame OK
            fails = 0;
            grace_warned = false;

            cv::Mat yuv_mat(target_height_ * 3 / 2, target_width_,
                            CV_8UC1, yuv_buf.data());
            cv::cvtColor(yuv_mat, bgr, cv::COLOR_YUV2BGR_I420);

            std_msgs::msg::Header hdr;
            hdr.stamp    = now();
            hdr.frame_id = (cam_id == 0) ? "camera_input_tray"
                                          : "camera_output_tray";

            auto msg = cv_bridge::CvImage(hdr, "bgr8", bgr).toImageMsg();

            if (cam_id == 0 && pub_cam0_) pub_cam0_->publish(*msg);
            else if (cam_id == 1 && pub_cam1_) pub_cam1_->publish(*msg);
        }

        RCLCPP_INFO(get_logger(), "🛑 CAM%d capture thread stopped", cam_id);
    }

    // Members
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam1_;

    CamProcess  cam0_;
    CamProcess  cam1_;
    std::mutex  mtx_cam0_;
    std::mutex  mtx_cam1_;
    std::mutex  mtx_reconnect_;  // FIX: serialize reconnects

    std::thread thread_cam0_;
    std::thread thread_cam1_;
    std::atomic<bool> running_;

    int    target_width_  {};
    int    target_height_ {};
    int    target_fps_    {};
    size_t yuv_frame_size_{};

    std::string cam0_topic_;
    std::string cam1_topic_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSIDualCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}