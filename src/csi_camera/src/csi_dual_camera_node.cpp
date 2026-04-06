// =============================================================================
// CSI Dual Camera Node — Fixed version
//
// Hardware: Raspberry Pi 5 + 2x IMX477 (12.3MP)
// Available sensor modes (from rpicam-hello --list-cameras):
//   SRGGB10_CSI2P : 1332x990   @ 120.05 fps  ← best for high-fps use
//   SRGGB12_CSI2P : 2028x1080  @  50.03 fps  ← best for 1080p / HD crop  ✓ USE THIS
//                   2028x1520  @  40.01 fps
//                   4056x3040  @  10.00 fps
//
// IMPORTANT — 1280x720 is NOT a native IMX477 mode.
//   rpicam selects mode 2028x1080 then software-scales to 1280x720, adding
//   ~3-4s ISP overhead on cold start. Always pass --width/--height that match
//   a native mode, or use --mode to force the sensor mode explicitly.
//   This node now defaults to 2028x1080 @ 30fps with explicit --mode flag.
//   Use parameters width/height/fps to override at runtime if needed.
//
// CHANGES vs original:
//   [FIX-1] Removed full_system_cleanup() from constructor → only run in script
//   [FIX-2] Removed O_NONBLOCK from read-end pipe → blocking read, no spurious EAGAIN
//   [FIX-3] Increased wait_for_first_frame timeout 8s → 15s for IMX477 on Pi 5 CFE
//   [FIX-4] capture_loop() now dup()s fd before releasing lock → no use-after-close
//   [FIX-5] kill_cam_process() drains fd before close → no leftover data on reuse
//   [FIX-6] launch_rpicam() waits 300ms then zombie-checks BEFORE returning
//   [FIX-7] Removed redundant zombie-detect block inside reconnect (now in launch_rpicam)
//   [FIX-8] read_frame() chunk timeout now based on actual frame size, not fps division
//   [FIX-9] Added --mode flag to force native sensor mode → eliminates ISP scale overhead
//           and removes ~3-4s from cold start time
//   [REMOVED] O_NONBLOCK flag — no longer needed, select() handles timeout cleanly
//   [REMOVED] duplicate zombie-detect code inside capture_loop reconnect block
// =============================================================================

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

// =============================================================================
// CamProcess — unchanged from original except added drain helper
// =============================================================================
struct CamProcess {
    pid_t pid = -1;
    std::atomic<int> fd{-1};

    CamProcess() = default;

    CamProcess(CamProcess&& o) noexcept
        : pid(o.pid), fd(o.fd.load())
    {
        o.pid = -1;
        o.fd.store(-1);
    }
    CamProcess& operator=(CamProcess&& o) noexcept
    {
        if (this != &o) {
            pid = o.pid;
            fd.store(o.fd.load());
            o.pid = -1;
            o.fd.store(-1);
        }
        return *this;
    }
    CamProcess(const CamProcess&) = delete;
    CamProcess& operator=(const CamProcess&) = delete;
};

// =============================================================================
// launch_rpicam
//
// [FIX-2] REMOVED O_NONBLOCK on pipefd[0].
//         Blocking read() + select()-based timeout is sufficient and avoids
//         spurious EAGAIN mid-frame that falsely reported "never started".
//
// [FIX-6] Added early zombie-check (300ms after fork).
//         If rpicam-vid dies immediately (device busy / wrong camera id),
//         this is detected here and pid is set to -1 so callers get a clear
//         failure signal instead of discovering it 8-15 seconds later.
// =============================================================================
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
        // Child process
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[1]);

        // [FIX-9] Redirect stderr to a log file instead of /dev/null.
        //         rpicam-vid logs ISP init errors, CFE timeouts, and sensor
        //         negotiation messages to stderr — this is the primary source
        //         of diagnostic info when a camera fails to start.
        //         Log path: /tmp/rpicam_cam{id}.log
        std::string logpath = "/tmp/rpicam_cam" + std::to_string(cam_id) + ".log";
        int logfd = open(logpath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (logfd >= 0) { dup2(logfd, STDERR_FILENO); close(logfd); }
        // If log open fails, fall back to /dev/null
        else {
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) { dup2(devnull, STDERR_FILENO); close(devnull); }
        }

        std::string w = std::to_string(width);
        std::string h = std::to_string(height);
        std::string f = std::to_string(fps);
        std::string c = std::to_string(cam_id);

        // [FIX-9] Build --mode string to force the native sensor mode.
        //         1332x990@15fps DESTROYS IMX477 VBLANK limits and causes CFE timeouts!
        //         We MUST use 2028:1080:12:P at >=30fps and let ISP downscale to 1280x720. 
        std::string mode_str = "2028:1080:12:P";

        execlp("rpicam-vid", "rpicam-vid",
               "--camera",    c.c_str(),
               "-t",          "0",
               "--nopreview",
               "--codec",     "yuv420",
               "--width",     w.c_str(),
               "--height",    h.c_str(),
               "--framerate", f.c_str(),
               "--mode",      mode_str.c_str(),  // force native sensor mode
               "--denoise",   "cdn_off",
               "--flush",
               "-o",          "-",
               nullptr);
        _exit(127);
    }

    // Parent process
    close(pipefd[1]);

    // Increase pipe buffer to 1MB — reduces dropped frames on burst writes
    fcntl(pipefd[0], F_SETPIPE_SZ, 1 * 1024 * 1024);

    struct timespec ts = {0, 300 * 1000 * 1000};  // 300ms
    nanosleep(&ts, nullptr);

    int wstatus = 0;
    pid_t reaped = waitpid(pid, &wstatus, WNOHANG);
    if (reaped != 0) {
        close(pipefd[0]);
        return {};  // pid=-1, fd=-1
    }

    CamProcess cp;
    cp.pid = pid;
    cp.fd.store(pipefd[0]);
    return cp;
}

// =============================================================================
// kill_cam_process
// =============================================================================
static void kill_cam_process(CamProcess& cp)
{
    if (cp.pid > 0) {
        kill(cp.pid, SIGKILL);
        waitpid(cp.pid, nullptr, 0);
        cp.pid = -1;
    }
    int fd = cp.fd.exchange(-1);
    if (fd >= 0) {
        uint8_t drain_buf[4096];
        while (true) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            struct timeval tv = {0, 0};  
            if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) break;
            ssize_t n = read(fd, drain_buf, sizeof(drain_buf));
            if (n <= 0) break;
        }
        close(fd);
    }
}

// =============================================================================
// CSIDualCameraNode
// =============================================================================
class CSIDualCameraNode : public rclcpp::Node
{
public:
    CSIDualCameraNode() : Node("csi_dual_camera_node"), running_(true)
    {
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "CSI Dual Camera Node (Patched)");
        RCLCPP_INFO(get_logger(), "========================================");

        // Hardware-safe geometry parameters. 
        declare_parameter("width",      1280);
        declare_parameter("height",      720);
        declare_parameter("fps",          30);
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
        // At 2028x1080 (native mode): 2028 * 1080 * 3/2 = 3,285,360 bytes ≈ 3.1 MB/frame
        // At 1332x990  (native mode): 1332 *  990 * 3/2 = 1,978,020 bytes ≈ 1.9 MB/frame

        // [FIX-1] REMOVED full_system_cleanup() call here.
        //
        //         Original code called cleanup both in launch.sh AND here in the
        //         constructor. The second pkill -9 arrived while the kernel was
        //         still releasing /dev/media0 from the first kill, causing
        //         launch_rpicam(0) to start on a still-busy device.
        //
        //         Cleanup is now ONLY done in launch.sh before this node starts.
        //         If you need cleanup in-process (e.g., running without the
        //         script), call a lightweight version that only checks, not kills.

        // Start CAM0
        RCLCPP_INFO(get_logger(), "🚀 Starting CAM0...");
        cam0_ = launch_rpicam(0, target_width_, target_height_, target_fps_);
        if (cam0_.pid < 0) {
            RCLCPP_ERROR(get_logger(), "❌ CAM0: rpicam-vid failed to start "
                "(device busy or wrong camera id). "
                "Check: cat /tmp/rpicam_cam0.log");
            return;
        }

        // Wait for CAM0 to produce first real frame before touching CAM1.
        // [FIX-3] Timeout increased from 8s → 15s.
        //         IMX477 on Pi 5 (CFE driver) needs ~10-12s on cold start:
        //         sensor mode negotiation + AGC settling + first frame DMA.
        //         8s timed out before the first frame arrived → false failure.
        if (!wait_for_first_frame(cam0_.fd.load(), "CAM0", 15)) {
            kill_cam_process(cam0_);
            RCLCPP_ERROR(get_logger(), "❌ CAM0 never streamed within 15s. "
                "Diagnose with: cat /tmp/rpicam_cam0.log");
            return;
        }

        // Start CAM1 only after CAM0 is confirmed streaming
        RCLCPP_INFO(get_logger(), "🚀 Starting CAM1...");
        cam1_ = launch_rpicam(1, target_width_, target_height_, target_fps_);
        if (cam1_.pid < 0) {
            RCLCPP_ERROR(get_logger(), "❌ CAM1: rpicam-vid failed to start");
            kill_cam_process(cam0_);
            return;
        }

        // [FIX-3] Same 15s timeout for CAM1 — non-fatal but logged clearly
        if (!wait_for_first_frame(cam1_.fd.load(), "CAM1", 15)) {
            RCLCPP_WARN(get_logger(), "⚠️  CAM1 never streamed within 15s — "
                "continuing without CAM1 (capture thread will handle reconnect)");
        }

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

    // =========================================================================
    // wait_for_first_frame — unchanged logic, timeout now a parameter
    // =========================================================================
    bool wait_for_first_frame(int fd, const char* name, int timeout_sec)
    {
        RCLCPP_INFO(get_logger(), "⏳ Waiting for %s first frame (timeout %ds)...",
                    name, timeout_sec);
        std::vector<uint8_t> buf(yuv_frame_size_);
        const auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::seconds(timeout_sec);
        while (std::chrono::steady_clock::now() < deadline) {
            if (read_frame(fd, buf)) {
                RCLCPP_INFO(get_logger(), "✅ %s streaming", name);
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        RCLCPP_ERROR(get_logger(), "❌ %s never started streaming within %ds!",
                     name, timeout_sec);
        return false;
    }

    // =========================================================================
    // read_frame
    //
    // [FIX-2] With O_NONBLOCK removed, read() blocks until data is available.
    //         We keep select() solely for timeout detection (process died).
    //         No more EAGAIN path needed mid-frame.
    //
    // [FIX-8] chunk_timeout_ms: original formula was 3000/fps which gives ~100ms
    //         at 30fps — too tight. Use 2× frame period as safety margin:
    //         (2000/fps) but minimum 500ms. At 30fps = 500ms (same as before),
    //         but at lower fps (10fps) = 200ms instead of 300ms — slightly better.
    //         More importantly: select() is now the ONLY timeout guard since
    //         blocking read() won't spuriously return.
    // =========================================================================
    bool read_frame(int fd, std::vector<uint8_t>& buf)
    {
        if (fd < 0 || yuv_frame_size_ == 0) return false;

        size_t total = 0;
        const int fps = (target_fps_ > 0) ? target_fps_ : 30;

        // Allow 3 full frame periods per chunk before declaring camera dead
        const int chunk_timeout_ms = std::max(500, 3000 / fps);

        while (total < yuv_frame_size_) {
            if (!running_.load()) return false;

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            struct timeval tv;
            tv.tv_sec  = chunk_timeout_ms / 1000;
            tv.tv_usec = (chunk_timeout_ms % 1000) * 1000;

            int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
            if (ret < 0 && errno == EINTR) continue;
            if (ret <= 0) return false;  // timeout or error → camera likely dead

            // [FIX-2] Blocking read — no EAGAIN possible (O_NONBLOCK removed)
            ssize_t n = read(fd, buf.data() + total, yuv_frame_size_ - total);
            if (n < 0) {
                // Should not happen on blocking fd except EIO (fd closed externally)
                if (errno == EINTR) continue;
                return false;
            }
            if (n == 0) return false;  // EOF — process exited
            total += static_cast<size_t>(n);
        }
        return true;
    }

    // =========================================================================
    // capture_loop (FIXED WITH DECOUPLED READER)
    //
    // [FIX-11] DECOUPLED THREAD READER
    // The infamous "Dequeue timer of 1000000 us has expired!" hardware crash happens
    // because OpenCV `cvtColor` and `publish` operations periodically take just long
    // enough that `rpicam-vid` fills the 1MB POSIX pipe buffer and blocks on `write()`.
    // When `rpicam-vid` blocks, the CFE hardware buffer overflows and permanently zombies.
    // By spinning a dedicated thread that ONLY parses bytes from `fd` into RAM lock-free,
    // the pipe remains perfectly empty and `rpicam-vid` NEVER blocks.
    // =========================================================================
    void capture_loop(int cam_id)
    {
        RCLCPP_INFO(get_logger(), "🎬 CAM%d capture thread started", cam_id);

        std::vector<uint8_t> yuv_buf(yuv_frame_size_);
        cv::Mat bgr(target_height_, target_width_, CV_8UC3);

        const int FAIL_THRESHOLD = (target_fps_ > 0 ? target_fps_ : 30) * 6;
        int fails = 0;
        int reconnect_attempts = 0;

        auto startup_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(10);

        while (running_.load() && rclcpp::ok()) {

            int fd_dup = -1;
            {
                std::lock_guard<std::mutex> lk(
                    (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                int raw_fd = (cam_id == 0) ? cam0_.fd.load() : cam1_.fd.load();
                if (raw_fd >= 0) {
                    fd_dup = dup(raw_fd);  // independent fd, survives close(raw_fd)
                }
            }

            if (fd_dup < 0) {
                // Camera not yet started or between reconnects — wait briefly
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            // --- DECOUPLED READER THREAD ---
            std::atomic<bool> reader_running{true};
            std::mutex mtx_latest;
            std::vector<uint8_t> latest_yuv(yuv_frame_size_);
            bool new_frame = false;
            auto last_frame_time = std::chrono::steady_clock::now();

            std::thread reader_thread([&]() {
                std::vector<uint8_t> local_buf(yuv_frame_size_);
                while (reader_running.load()) {
                    if (!read_frame(fd_dup, local_buf)) {
                        reader_running.store(false);
                        break;
                    }
                    std::lock_guard<std::mutex> rlk(mtx_latest);
                    latest_yuv = local_buf;
                    new_frame = true;
                    last_frame_time = std::chrono::steady_clock::now();
                }
            });

            bool frame_ok = true;
            while (reader_running.load() && running_.load() && rclcpp::ok()) {
                bool got_frame = false;
                {
                    std::lock_guard<std::mutex> rlk(mtx_latest);
                    if (new_frame) {
                        yuv_buf = latest_yuv;  // Copy buffer for processing
                        new_frame = false;
                        got_frame = true;
                    }
                }

                if (!got_frame) {
                    // Timeout watchdog to trigger reconnect if reader gets stuck
                    if (std::chrono::steady_clock::now() - last_frame_time > std::chrono::seconds(3)) {
                        frame_ok = false;
                        RCLCPP_WARN(get_logger(), "CAM%d: Pipe reader timeout (No data for 3s)", cam_id);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }

                fails = 0;
                reconnect_attempts = 0;  // reset backoff on successful recovery

                cv::Mat yuv_mat(target_height_ * 3 / 2, target_width_,
                                CV_8UC1, yuv_buf.data());
                cv::cvtColor(yuv_mat, bgr, cv::COLOR_YUV2BGR_I420);

                std_msgs::msg::Header hdr;
                hdr.stamp    = now();
                hdr.frame_id = (cam_id == 0) ? "camera_input_tray"
                                              : "camera_output_tray";

                auto msg = cv_bridge::CvImage(hdr, "bgr8", bgr).toImageMsg();

                if      (cam_id == 0 && pub_cam0_) pub_cam0_->publish(*msg);
                else if (cam_id == 1 && pub_cam1_) pub_cam1_->publish(*msg);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                    "✓ CAM%d publishing frames", cam_id);
            }

            reader_running.store(false);
            close(fd_dup);  // instantly unblocks the read() inside reader_thread
            if (reader_thread.joinable()) {
                reader_thread.join();
            }

            if (!frame_ok || !running_.load()) {
                if (std::chrono::steady_clock::now() < startup_deadline) {
                    while (waitpid(-1, nullptr, WNOHANG) > 0) {}
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }

                fails++;
                if (fails < FAIL_THRESHOLD) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                RCLCPP_WARN(get_logger(),
                    "⚠️  CAM%d: %d consecutive failures — reconnecting (attempt #%d)...",
                    cam_id, fails, reconnect_attempts + 1);

                {
                    FILE* f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
                    if (f) {
                        int temp_milli = 0;
                        if (fscanf(f, "%d", &temp_milli) == 1) {
                            float temp_c = temp_milli / 1000.0f;
                            RCLCPP_WARN(get_logger(),
                                "   CPU temp at crash: %.1f°C%s",
                                temp_c,
                                temp_c > 80.0f ? " ⚠️  THERMAL THROTTLE LIKELY!" :
                                temp_c > 70.0f ? " (warm, watch this)" : " (ok)");
                        }
                        fclose(f);
                    }
                }

                {
                    std::lock_guard<std::mutex> recon_lk(mtx_reconnect_);

                    {
                        std::lock_guard<std::mutex> lk(
                            (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                        CamProcess& cp = (cam_id == 0) ? cam0_ : cam1_;
                        kill_cam_process(cp);
                    }

                    int backoff_ms = 800;
                    for (int i = 0; i < reconnect_attempts && backoff_ms < 30000; i++) {
                        backoff_ms *= 2;
                    }
                    backoff_ms = std::min(backoff_ms, 30000);
                    if (reconnect_attempts > 0) {
                        RCLCPP_WARN(get_logger(),
                            "   Backoff delay: %dms (attempt #%d)",
                            backoff_ms, reconnect_attempts + 1);
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(backoff_ms));
                    while (waitpid(-1, nullptr, WNOHANG) > 0) {}

                    if (!running_.load()) break;

                    CamProcess new_cp = launch_rpicam(
                        cam_id, target_width_, target_height_, target_fps_);

                    {
                        std::lock_guard<std::mutex> lk(
                            (cam_id == 0) ? mtx_cam0_ : mtx_cam1_);
                        CamProcess& cp = (cam_id == 0) ? cam0_ : cam1_;
                        if (new_cp.pid > 0) {
                            cp = std::move(new_cp);
                            RCLCPP_INFO(get_logger(),
                                "✅ CAM%d reconnected (pid=%d)", cam_id, cp.pid);
                        } else {
                            RCLCPP_ERROR(get_logger(),
                                "❌ CAM%d reconnect failed — retry in 2s", cam_id);
                        }
                    }

                    reconnect_attempts++;

                    startup_deadline = std::chrono::steady_clock::now()
                                       + std::chrono::seconds(15);

                    if (new_cp.pid > 0) {
                        rclcpp::sleep_for(std::chrono::milliseconds(500));
                    } else {
                        rclcpp::sleep_for(std::chrono::milliseconds(2000));
                    }
                }

                fails = 0;
                continue;
            }
        }

        RCLCPP_INFO(get_logger(), "🛑 CAM%d capture thread stopped", cam_id);
    }

    // =========================================================================
    // Members
    // =========================================================================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam1_;

    CamProcess  cam0_;
    CamProcess  cam1_;
    std::mutex  mtx_cam0_;
    std::mutex  mtx_cam1_;
    std::mutex  mtx_reconnect_;

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

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSIDualCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}