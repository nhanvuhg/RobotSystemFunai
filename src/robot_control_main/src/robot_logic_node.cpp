#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/robot_mode.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "dobot_msgs_v3/srv/speed_l.hpp"
#include "dobot_msgs_v3/srv/acc_l.hpp"
#include "dobot_msgs_v3/srv/speed_j.hpp"
#include "dobot_msgs_v3/srv/acc_j.hpp"
#include "std_msgs/msg/bool.hpp"
#include "plc_msg/msg/plc_read.hpp"
#include "plc_msg/msg/plc_write.hpp"
#include "plc_msg/msg/plc_response.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sstream>

#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>

#include <deque>
#include <algorithm>

#include <array>
#include <cmath>
#include <limits>
#include <cstdint>

// using MoveJ = dobot_msgs_v3::srv::JointMovJ;

using namespace std::chrono_literals;
using vision_msgs::msg::Detection2D;

using EnableRobot = dobot_msgs_v3::srv::EnableRobot;
using GetPose = dobot_msgs_v3::srv::GetPose;
using GetAngle = dobot_msgs_v3::srv::GetAngle;
using JointMovJ = dobot_msgs_v3::srv::JointMovJ;
using RelMovL = dobot_msgs_v3::srv::RelMovL;
using DO = dobot_msgs_v3::srv::DO;
using RobotMode = dobot_msgs_v3::srv::RobotMode;
using Detection2DArray = vision_msgs::msg::Detection2DArray;
using SpeedL = dobot_msgs_v3::srv::SpeedL;
using AccL = dobot_msgs_v3::srv::AccL;
using SpeedJ = dobot_msgs_v3::srv::SpeedJ;
using AccJ = dobot_msgs_v3::srv::AccJ;

using PLCRead = plc_msg::msg::PLCRead;
using PLCWrite = plc_msg::msg::PLCWrite;
using PLCResponse = plc_msg::msg::PLCResponse;

// filter
struct RowFilter
{
    size_t window = 3;        // ✅ Giảm từ 5 → 3 (nhanh hơn 40%)
    int max_fall = 2;         // ✅ Cho phép giảm 2 frame (ổn định hơn)
    int ready_consec = 4;     // ✅ Chỉ cần 2 frame liên tục (nhanh hơn 33%)

    std::deque<int> hist;
    int last_filtered = 0;
    int ready_streak = 0;

    int filter_count(int raw_count)
    {
        hist.push_back(raw_count);
        if (hist.size() > window)
            hist.pop_front();

        std::vector<int> tmp(hist.begin(), hist.end());
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        int med = tmp[tmp.size() / 2];

        int y = med;
        if (y < last_filtered - max_fall)
        {
            y = last_filtered - max_fall;
        }
        last_filtered = y;
        return y;
    }

    bool update_ready(bool raw_ready)
    {
        if (raw_ready)
            ready_streak++;
        else
            ready_streak = 0;
        return ready_streak >= ready_consec;
    }

    void clear()
    {
        hist.clear();
        last_filtered = 0;
        ready_streak = 0;
    }
};

struct Box
{
    double x1, y1, x2, y2;
}; // axis-aligned bbox
struct SlotDef
{                                     // canonical slot (template)
    std::vector<cv::Point2f> corners; // 4 điểm theo ảnh chuẩn
    char orient;                      // 'H' (ngang) hoặc 'V' (dọc)
};

// Trạng thái ổn định cho mỗi slot
enum class SlotStableState : int
{
    EMPTY = 0,
    OCC_OK = 1,
    MIS = 2
};

class RobotLogicNode : public rclcpp::Node
{
public:
    RobotLogicNode() : Node("robot_logic_nova5"), detected_(false), has_button_(false)
    {
        std::vector<std::string> motion_sequence;
        this->declare_parameter("motion_sequence", std::vector<std::string>{});
        this->get_parameter("motion_sequence", motion_sequence);

        for (const auto &line : motion_sequence)
        {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ','))
            {
                tokens.push_back(token);
            }

            if (tokens.empty())
                continue;

            const std::string &type = tokens[0];

            if (type == "J" && tokens.size() == 7)
            {
                std::vector<double> joints;
                for (size_t i = 1; i <= 6; ++i)
                    joints.push_back(std::stod(tokens[i]));
                joint_sequences_.push_back(joints);
            }
            else if (type == "R" && tokens.size() == 4)
            {
                relmovl_sequences_.emplace_back(std::vector<double>{
                    std::stod(tokens[1]),
                    std::stod(tokens[2]),
                    std::stod(tokens[3])});
            }
            else if (type == "D" && tokens.size() == 3)
            {
                digital_output_steps_.emplace_back(
                    std::stoi(tokens[1]),
                    std::stoi(tokens[2]));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Malformed motion step: '%s'", line.c_str());
            }
        }
        // debug
        RCLCPP_INFO(this->get_logger(), "=== JointMovJ Sequences ===");
        for (size_t i = 0; i < joint_sequences_.size(); ++i)
        {
            std::ostringstream oss;
            for (auto v : joint_sequences_[i])
                oss << v << " ";
            RCLCPP_INFO(this->get_logger(), "J[%zu]: %s", i, oss.str().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "=== RelMovL Sequences ===");
        for (size_t i = 0; i < relmovl_sequences_.size(); ++i)
        {
            std::ostringstream oss;
            for (auto v : relmovl_sequences_[i])
                oss << v << " ";
            RCLCPP_INFO(this->get_logger(), "R[%zu]: %s", i, oss.str().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "=== DO Steps ===");
        for (size_t i = 0; i < digital_output_steps_.size(); ++i)
        {
            auto [index, status] = digital_output_steps_[i];
            RCLCPP_INFO(this->get_logger(), "D[%zu]: index=%d, status=%d", i, index, status);
        }

        std::string plc_ip = this->declare_parameter<std::string>("plc_ip", "192.168.27.6");
        plc_ip_read_ = this->declare_parameter<std::string>("plc_ip_read", plc_ip);
        plc_ip_write_ = this->declare_parameter<std::string>("plc_ip_write", plc_ip);

        db_number_ = this->declare_parameter<int>("db_number", 185);
        read_offset_ = this->declare_parameter<int>("read_offset", 0);
        read_size_ = this->declare_parameter<int>("read_size", 20);
        write_offset_ = this->declare_parameter<int>("write_offset", 0);
        read_period_ms_ = this->declare_parameter<int>("read_period_ms", 50);

        iou_thresh_ = this->declare_parameter<double>("iou_thresh", 0.10); // Increased to filter weak detections
        debug_logs_ = this->declare_parameter<bool>("debug_logs", true); // log debug
        slot_shrink_ratio_ = this->declare_parameter<double>("slot_shrink_ratio", 1.0); // NO SHRINK
        use_tray_from_detection_ = this->declare_parameter<bool>("use_tray_from_detection", false); // STATIC ROI

        // Lọc detection
        score_thresh_o = this->declare_parameter<double>("score_thresh", 0.15);
        nms_iou_thresh_ = this->declare_parameter<double>("nms_iou_thresh", 0.50);

        // Debounce / ổn định
        confirm_frames_ = this->declare_parameter<int>("confirm_frames", 2);

        // --- canonical tray corners--
        canonical_tray_ = {
            // cv::Point2f(205, 590),
            // cv::Point2f(315, 110),
            // cv::Point2f(980, 110),
            // cv::Point2f(1110, 590)};
            cv::Point2f(135, 468),
            cv::Point2f(310, 55),
            cv::Point2f(837, 45),
            cv::Point2f(920, 468)};
            //new position csi camera
        RCLCPP_INFO(this->get_logger(), "Robot logic node for Nova5 started.");

        // enable_client_ = create_client<EnableRobot>("/nova5/dobot_bringup/EnableRobot");
        // pose_client_ = create_client<GetPose>("/nova5/dobot_bringup/GetPose");
        // angle_client_ = create_client<GetAngle>("/nova5/dobot_bringup/GetAngle");
        // joint_client_ = create_client<JointMovJ>("/nova5/dobot_bringup/JointMovJ");
        // relmovl_client_ = create_client<RelMovL>("/nova5/dobot_bringup/RelMovL");
        // do_client_ = create_client<DO>("/nova5/dobot_bringup/DO");
        // robot_mode_client_ = create_client<RobotMode>("/nova5/dobot_bringup/RobotMode");
        // speedl_client_ = create_client<SpeedL>("/nova5/dobot_bringup/SpeedL");
        // accl_client_ = create_client<AccL>("/nova5/dobot_bringup/AccL");
        // speedj_client_ = create_client<SpeedJ>("/nova5/dobot_bringup/SpeedJ");
        // accj_client_ = create_client<AccJ>("/nova5/dobot_bringup/AccJ");

        init_rows_and_rois();
        buildCanonicalSlots();

        // Khởi tạo bộ nhớ cho ổn định theo số slot
        stable_state_.assign((int)canonical_slots_.size(), SlotStableState::EMPTY);
        empty_streak_.assign((int)canonical_slots_.size(), 0);
        occ_ok_streak_.assign((int)canonical_slots_.size(), 0);
        mis_streak_.assign((int)canonical_slots_.size(), 0);

        // yolov8_sub_cam1_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        //     "/cam1/detections_output", 10,
        //     std::bind(&RobotLogicNode::yolov8CallbackCam1, this, std::placeholders::_1));

        // CSI Camera subscriptions - updated for Funai naming convention
        yolov8_sub_cam1_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/cam0Funai/yolo/bounding_boxes", 10,
            std::bind(&RobotLogicNode::yolov8CallbackCam5, this, std::placeholders::_1));

        yolov8_sub_cam5_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/cam1Funai/yolo/bounding_boxes", 10,
            std::bind(&RobotLogicNode::detCallback, this, std::placeholders::_1));



        // yolov8_sub_camrealsense_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        //     "/camrealsense/detections_output", 10,
        //     std::bind(&RobotLogicNode::yolov8CallbackCamRealsense, this, std::placeholders::_1));

        empty_slot_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/empty_slots", rclcpp::QoS(10));

        // signal_take_cartridge_to_fill_machine_ = this->create_subscription<std_msgs::msg::Bool>(
        //     "/take_cartridge", 10,
        //     std::bind(&RobotLogicNode::takeCartridgeCallback, this, std::placeholders::_1));

        read_pub_ = this->create_publisher<plc_msg::msg::PLCRead>("/plc/read_request", 10);
        write_pub_ = this->create_publisher<plc_msg::msg::PLCWrite>("/plc/write_request", 10);

        read_sub_ = this->create_subscription<PLCResponse>(
            "/plc/read_response", 10,
            std::bind(&RobotLogicNode::onReadResponse, this, std::placeholders::_1));

        write_values_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/plc/write_values", 10,
            std::bind(&RobotLogicNode::onWriteValues, this, std::placeholders::_1));

        timer_plc = this->create_wall_timer(
            std::chrono::milliseconds(read_period_ms_),
            std::bind(&RobotLogicNode::publishReadRequest, this));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RobotLogicNode::run_next_step, this));
    }

private:
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string &service_name)
    {
        // RCLCPP_INFO(get_logger(), "[callService] Waiting for %s...", service_name.c_str());
        if (!client->wait_for_service(5s))
        {
            RCLCPP_ERROR(get_logger(), "[callService] %s not available after timeout", service_name.c_str());
            return nullptr;
        }
        auto future = client->async_send_request(request);
        auto res = future.get();
        // if (res)
        // {
        //     RCLCPP_INFO(get_logger(), "[callService] %s response: res=%d", service_name.c_str(), res->res);
        // }
        return res;
    }

    rclcpp::Client<EnableRobot>::SharedPtr enable_client_;
    rclcpp::Client<GetPose>::SharedPtr pose_client_;
    rclcpp::Client<GetAngle>::SharedPtr angle_client_;
    rclcpp::Client<JointMovJ>::SharedPtr joint_client_;
    rclcpp::Client<RelMovL>::SharedPtr relmovl_client_;
    rclcpp::Client<DO>::SharedPtr do_client_;
    rclcpp::Client<RobotMode>::SharedPtr robot_mode_client_;
    rclcpp::Client<SpeedL>::SharedPtr speedl_client_;
    rclcpp::Client<AccL>::SharedPtr accl_client_;
    rclcpp::Client<SpeedJ>::SharedPtr speedj_client_;
    rclcpp::Client<AccJ>::SharedPtr accj_client_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolov8_sub_cam1_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolov8_sub_cam2_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolov8_sub_cam5_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolov8_sub_camrealsense_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr empty_slot_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_take_cartridge_to_fill_machine_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double>> joint_sequences_;
    std::vector<std::vector<double>> relmovl_sequences_;
    std::vector<std::pair<int, int>> digital_output_steps_;
    std::vector<RowFilter> filters_;
    // tray output camera
    std::vector<cv::Point2f> canonical_tray_;
    std::vector<SlotDef> canonical_slots_;
    double iou_thresh_;
    bool debug_logs_;
    double slot_shrink_ratio_{0.9};
    bool use_tray_from_detection_{true};
    int count_check_change_tray_output = 0;

    // Bộ lọc detection
    double score_thresh_o{0.45};
    double nms_iou_thresh_{0.50};

    // Debounce
    int confirm_frames_{3};
    std::vector<SlotStableState> stable_state_;
    std::vector<int> empty_streak_, occ_ok_streak_, mis_streak_;

    std::vector<bool> row_full_;
    std::vector<bool> row_full_prev_;
    int num_full_ = 0;
    uint32_t full_bitset_ = 0;
    std::mutex row_status_mutex_;  // Thread-safe for row_full_ access

    int row6_cnt2_ = 0;
    int row6_cnt3_ = 0;

    std::string plc_ip_read_, plc_ip_write_;
    int db_number_{10};
    int read_offset_{0};
    int read_size_{40};
    int write_offset_{0};
    int read_period_ms_{100};

    rclcpp::Publisher<PLCRead>::SharedPtr read_pub_;
    rclcpp::Publisher<PLCWrite>::SharedPtr write_pub_;
    rclcpp::Subscription<PLCResponse>::SharedPtr read_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr write_values_sub_;
    rclcpp::TimerBase::SharedPtr timer_plc;


    struct DetectionState
    {
        int count_cam1 = 0;
        int count_cam2 = 0;
        bool ready() const
        {
            return count_cam1 == 4 && count_cam2 == 4;
        }
    } detection_state_;



    struct Point2
    {
        float x{}, y{};
    };

    struct ROIQuad
    {
        std::array<Point2, 4> pts;
        int min_x{}, max_x{}, min_y{}, max_y{};

        static ROIQuad FromCorners(const std::vector<std::pair<int, int>> &corners)
        {
            ROIQuad r{};
            for (size_t i = 0; i < 4; ++i)
            {
                r.pts[i] = Point2{static_cast<float>(corners[i].first),
                                  static_cast<float>(corners[i].second)};
            }
            r.min_x = std::min({corners[0].first, corners[1].first,
                                corners[2].first, corners[3].first});
            r.max_x = std::max({corners[0].first, corners[1].first,
                                corners[2].first, corners[3].first});
            r.min_y = std::min({corners[0].second, corners[1].second,
                                corners[2].second, corners[3].second});
            r.max_y = std::max({corners[0].second, corners[1].second,
                                corners[2].second, corners[3].second});
            return r;
        }

        inline bool bbox_contains(float x, float y) const
        {
            return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
        }

        inline bool contains(float x, float y) const
        {
            if (!bbox_contains(x, y))
                return false;
            auto cross = [](const Point2 &a, const Point2 &b, const Point2 &c)
            {
                // z-component of (b-a) x (c-a)
                return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            };
            const Point2 p{x, y};
            float c0 = cross(pts[0], pts[1], p);
            float c1 = cross(pts[1], pts[2], p);
            float c2 = cross(pts[2], pts[3], p);
            float c3 = cross(pts[3], pts[0], p);

            bool all_nonneg = (c0 >= 0 && c1 >= 0 && c2 >= 0 && c3 >= 0);
            bool all_nonpos = (c0 <= 0 && c1 <= 0 && c2 <= 0 && c3 <= 0);
            return all_nonneg || all_nonpos;
        }
    };

    struct DetectionCartridge
    {
        int count = 0;
        int ready_threshold = 8;
        std::string target_class_id_ = "0";
        bool ready() const { return count >= ready_threshold; }
        void reset() { count = 0; }
    };

    struct Det2D
    {
        double cx, cy;
        int class_id;
        double score;
    };

    std::vector<ROIQuad> rois_;
    std::vector<DetectionCartridge> rows_;

    bool detected_;
    bool has_button_;

    static constexpr const char *CLASS_TRAY = "0";
    static constexpr const char *CLASS_LINE_PASS = "1";
    static constexpr const char *CLASS_LINE_FALL = "2";
    static constexpr const char *CLASS_CARTRIDGE = "3";
    int slot_id = -1;
    int slot_id_temp;
    int slot_id_full;
    bool take_cartridge;
    int row_take_cartirdge;
    bool take_new_cartridge;
    bool end_cartridge_final;
    int count_check_cartridge_empty_tray = 0;

    using ValueT = typename decltype(PLCResponse().values)::value_type;

    std::vector<ValueT> last_values_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool new_resp_{false};

    int cmdCheckTakeCartridge;
    int valueTakeCartridge;

    int cmdCheckTray;
    int valueCheckTray;

    enum class Step
    {
        step_home_state,
        step_0_IDLE_state,
        step_1_check_take_cartridge_empty,
        step_2_add_cartridge_to_fill_machine,
        step_3_take_cartridge_fill_machine_to_scale,
        step_5_check_scale_cartridge,
        step_6_take_cartridge_from_scale_to_tray,
        step_7_check_tray_empty_position,
        step_8_put_cartridge_to_empty_position_on_tray,
        step_9_arrange_batch_cartridge_on_tray,
        step_10_take_cartridge_to_position_fail,
        check_cartridge_on_tray_empty,
        enable_robot
    };


    Step current_step_ = Step::step_0_IDLE_state;  // Normal operation
    // Step current_step_ = Step::step_5_check_scale_cartridge;  // TEST: Start at Step 5
    int count_check_change_tray = 0;

    void runMotionPlan()
    {
        // Track last logged step to avoid spam
        static Step last_logged_step_ = Step::step_home_state;
        static auto last_log_time_ = std::chrono::steady_clock::now();
        
        while (1)
        {
            // Log current step every 5 seconds OR when step changes
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();
            
            if (current_step_ != last_logged_step_ || elapsed >= 5)
            {
                const char* step_names[] = {
                    "HOME", "IDLE", "CHECK_TAKE", "ADD_TO_FILL", "TAKE_TO_SCALE", 
                    "CHECK_SCALE", "TAKE_TO_TRAY", "CHECK_TRAY_EMPTY",
                    "PUT_TO_TRAY", "ARRANGE_BATCH", "POSITION_FAIL", "CHECK_TRAY_INPUT", "ENABLE"
                };
                int step_idx = static_cast<int>(current_step_);
                if (step_idx >= 0 && step_idx < 14)
                {
                    RCLCPP_INFO(this->get_logger(), "🤖 [CURRENT STEP] %s", step_names[step_idx]);
                }
                last_logged_step_ = current_step_;
                last_log_time_ = now;
            }
            
            switch (current_step_)
            {
            case Step::step_home_state:
                step_home_state();
                break;
            case Step::step_0_IDLE_state:
                step_0_IDLE_state();
                break;
            case Step::step_1_check_take_cartridge_empty:
                step_1_check_take_cartridge_empty();
                break;
            case Step::step_2_add_cartridge_to_fill_machine:
                step_2_add_cartridge_to_fill_machine();
                break;
            case Step::step_3_take_cartridge_fill_machine_to_scale:
                step_3_take_cartridge_fill_machine_to_scale();
                break;

            case Step::step_5_check_scale_cartridge:
                step_5_check_scale_cartridge();
                break;
            case Step::step_6_take_cartridge_from_scale_to_tray:
                step_6_take_cartridge_from_scale_to_tray();
                break;
            case Step::step_7_check_tray_empty_position:
                step_7_check_tray_empty_position();
                break;
            case Step::step_8_put_cartridge_to_empty_position_on_tray:
                step_8_put_cartridge_to_empty_position_on_tray();
                break;
            case Step::check_cartridge_on_tray_empty:
                check_cartridge_on_tray_empty();
                break;
            case Step::enable_robot:
                enable_robot();
            }
            rclcpp::sleep_for(500ms);
        }
    }

    void step_home_state()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP] Home...");
        rclcpp::sleep_for(500ms);
        rclcpp::sleep_for(2000ms);
        current_step_ = Step::step_0_IDLE_state;
    }

    void step_0_IDLE_state()
    {
        if (cmdCheckTakeCartridge == 1)  // Check input tray (Camera 0)
        {
            current_step_ = Step::check_cartridge_on_tray_empty;
        }
        else if (cmdCheckTakeCartridge == 0)  // Check output tray (Camera 1)  
        {
            current_step_ = Step::step_7_check_tray_empty_position;
        }
        else
        {
            // Stay in IDLE until a command is received
            current_step_ = Step::step_0_IDLE_state;
        }
    }

    void step_1_check_take_cartridge_empty()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 1] Check row cartridge");
        if (row_take_cartirdge <= 6)
        {
            current_step_ = Step::check_cartridge_on_tray_empty;
            row_take_cartirdge++;
            take_new_cartridge = false;
        }
        else
        {
            current_step_ = Step::step_0_IDLE_state;
        }
    }

    void step_2_add_cartridge_to_fill_machine()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 2] Take and add cartridge to fill machine");
        count_check_cartridge_empty_tray = 0;
    }

    void step_3_take_cartridge_fill_machine_to_scale()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 3] Take the cartirdge from Fill machine to scale");
    }

    void step_5_check_scale_cartridge()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 5] Checking scale cartridge...");
        rclcpp::sleep_for(500ms);
        
        // USER REQUEST: Bypass scale check (row_full_[5])
        // Always proceed as if check passed

        RCLCPP_INFO(this->get_logger(), "[STEP 5] ✅ Scale check BYPASSED. Proceeding to take cartridge to tray.");
        writeToPLC("192.168.27.6", 185, 4, {1, 2});
        

        
        current_step_ = Step::step_6_take_cartridge_from_scale_to_tray;
        
        /* REMOVED:
        if (row_full_[5]) { ... } else { ... }
        */
    }

    void step_6_take_cartridge_from_scale_to_tray()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 6] Waiting for Camera 1 ready...");
        

        
        RCLCPP_INFO(this->get_logger(), "[STEP 6] ✅ Camera 1 Active. Proceeding to Step 7.");
        current_step_ = Step::step_7_check_tray_empty_position;
    }

    void step_7_check_tray_empty_position()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 7] Checking empty position on tray...");
        

        // 2. Wait for detection logic to update slot_id
        // (Wait 2s = ~60 frames)
        rclcpp::sleep_for(1000ms);

        // 3. Validate Slot ID before transitioning to Step 8
        if (slot_id > 0)
        {
            RCLCPP_INFO(this->get_logger(), "[STEP 7] ✅ Camera 1 Active & Empty Slot Found: %d. Proceeding to Step 8.", slot_id);
            current_step_ = Step::step_8_put_cartridge_to_empty_position_on_tray;
        }
        else
        {
            // No empty slot found -> Send tray change command immediately
            RCLCPP_WARN(this->get_logger(), "[STEP 7] ⚠️ Tray is FULL (slot_id=%d) -> Sending TRAY CHANGE command 12 to PLC", slot_id);
            writeToPLC("192.168.27.6", 185, 8, {0, 12});
            
            current_step_ = Step::step_0_IDLE_state;
        } 
    }

    void step_8_put_cartridge_to_empty_position_on_tray()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 8] Put the cartridge to the tray empty...");
        
        // Camera 1 switch moved to Step 7 for optimization
        // slot_id_temp = slot_id;
        // take_new_cartridge = true;
        // if (slot_id_full == 100)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 100});
        // else if (slot_id == 2)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 1});
        // else if (slot_id == 4)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 2});
        // else if (slot_id == 6)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 3});
        // else if (slot_id == 8)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 4});
        // else if (slot_id == 1)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 6});
        // else if (slot_id == 3)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 7});
        // else if (slot_id == 5)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 8});
        // else if (slot_id == 7)
        //     writeToPLC("192.168.27.6", 185, 8, {0, 9});
        if (slot_id == 1)
            writeToPLC("192.168.27.6", 185, 8, {0, 1});
        else if (slot_id == 2)
            writeToPLC("192.168.27.6", 185, 8, {0, 2});
        else if (slot_id == 3)
            writeToPLC("192.168.27.6", 185, 8, {0, 3});
        else if (slot_id == 4)
            writeToPLC("192.168.27.6", 185, 8, {0, 4});
        else if (slot_id == 5)
            writeToPLC("192.168.27.6", 185, 8, {0, 5});
        else if (slot_id == 6)
            writeToPLC("192.168.27.6", 185, 8, {0, 6});
        else if (slot_id == 7)
            writeToPLC("192.168.27.6", 185, 8, {0, 7});
        else if (slot_id == 8)
            writeToPLC("192.168.27.6", 185, 8, {0, 8});
        else if (slot_id == 9)
            writeToPLC("192.168.27.6", 185, 8, {0, 9});
        else  // No valid slot found -> send tray change command immediately
        {
            RCLCPP_WARN(this->get_logger(), "[STEP 8] ⚠️ No empty slot found (slot_id=%d) -> Sending tray change command 12", slot_id);
            writeToPLC("192.168.27.6", 185, 8, {0, 12});
        }
        
        current_step_ = Step::step_0_IDLE_state;
        rclcpp::sleep_for(100ms);
    }



    void check_cartridge_on_tray_empty()
    {
        RCLCPP_INFO(this->get_logger(), "[AI check] ========================================");
        RCLCPP_INFO(this->get_logger(), "[AI check] Check cartridge empty on the tray");
        
        
        // Lock mutex when reading row_full_ to avoid race condition
        std::lock_guard<std::mutex> lock(row_status_mutex_);
        
        // Log status of all rows
        RCLCPP_INFO(this->get_logger(), "[AI check] Row status: R1=%s R2=%s R3=%s R4=%s R5=%s",
                    row_full_[0] ? "FULL" : "empty",
                    row_full_[1] ? "FULL" : "empty",
                    row_full_[2] ? "FULL" : "empty",
                    row_full_[3] ? "FULL" : "empty",
                    row_full_[4] ? "FULL" : "empty");
        
        // PRIORITY: Row 1 → 2 → 3 → 4 → 5 (LOWEST FIRST)
        if (row_full_[0])  // Row 1 (index 0)
        {
            RCLCPP_INFO(this->get_logger(), "[AI check] ✅ Row 1 is FULL → Sending code 5 to PLC");
            writeToPLC("192.168.27.6", 185, 0, {0, 1});
            current_step_ = Step::step_0_IDLE_state;
        }
        else if (row_full_[1])  // Row 2 (index 1)
        {
            RCLCPP_INFO(this->get_logger(), "[AI check] ✅ Row 2 is FULL → Sending code 4 to PLC");
            writeToPLC("192.168.27.6", 185, 0, {0, 2});
            current_step_ = Step::step_0_IDLE_state;
        }
        else if (row_full_[2])  // Row 3 (index 2)
        {
            RCLCPP_INFO(this->get_logger(), "[AI check] ✅ Row 3 is FULL → Sending code 3 to PLC");
            writeToPLC("192.168.27.6", 185, 0, {0, 3});
            current_step_ = Step::step_0_IDLE_state;
        }
        else if (row_full_[3])  // Row 4 (index 3)
        {
            RCLCPP_INFO(this->get_logger(), "[AI check] ✅ Row 4 is FULL → Sending code 2 to PLC");
            writeToPLC("192.168.27.6", 185, 0, {0, 4});
            current_step_ = Step::step_0_IDLE_state;
        }
        else if (row_full_[4])  // Row 5 (index 4)
        {
            RCLCPP_INFO(this->get_logger(), "[AI check] ✅ Row 5 is FULL → Sending code 1 to PLC");
            writeToPLC("192.168.27.6", 185, 0, {0, 5});
            current_step_ = Step::step_0_IDLE_state;
        }
        else
        {
            // No rows are FULL
            RCLCPP_WARN(this->get_logger(), "[AI check] ⚠️ No rows are FULL");
            if (++count_check_change_tray >= 3)
            {
                RCLCPP_WARN(this->get_logger(), "[AI check] ⚠️ Requesting tray change after 3 retries (code 12)");
                writeToPLC("192.168.27.6", 185, 0, {0, 12}); // change tray
                current_step_ = Step::step_0_IDLE_state;
                count_check_change_tray = 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[AI check] 🔄 Retry check (%d/3)", count_check_change_tray);
                current_step_ = Step::check_cartridge_on_tray_empty;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "[AI check] ========================================");
        rclcpp::sleep_for(500ms);
    }

    void enable_robot()
    {
        RCLCPP_INFO(this->get_logger(), "[ENABLE] Enabling Robot...");
        callEnable();
        // current_step_ = 0;
    }



    void yolov8CallbackCam1(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        detection_state_.count_cam1 = 0;
        float score_thresh = 0.45f;
        std::vector<std::pair<int, int>> corners = {
            {164, 299},
            {518, 297},
            {521, 432},
            {166, 436}};
        int min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        int max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {

                const std::string &class_id = r.hypothesis.class_id;
                float confidence = r.hypothesis.score;
                if (class_id == "2")
                {
                    if (confidence > score_thresh)
                    {
                        float center_x = det.bbox.center.position.x;
                        float center_y = det.bbox.center.position.y;

                        if (center_x >= min_x && center_x <= max_x &&
                            center_y >= min_y && center_y <= max_y)
                        {
                            detection_state_.count_cam1++;
                        }
                    }
                }
            }
        }
    }

    void yolov8CallbackCam2(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        detection_state_.count_cam2 = 0;
        float score_thresh = 0.45f;
        std::vector<std::pair<int, int>> corners = {
            {127, 287},
            {130, 428},
            {481, 278},
            {480, 414}};
        int min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        int max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {

                const std::string &class_id = r.hypothesis.class_id;
                float confidence = r.hypothesis.score;
                if (class_id == "2")
                {
                    if (confidence > score_thresh)
                    {
                        float center_x = det.bbox.center.position.x;
                        float center_y = det.bbox.center.position.y;

                        if (center_x >= min_x && center_x <= max_x &&
                            center_y >= min_y && center_y <= max_y)
                        {
                            detection_state_.count_cam2++;
                        }
                    }
                }
            }
        }
    }

    void set_row_full(size_t i, bool full)
    {
        std::lock_guard<std::mutex> lock(row_status_mutex_);
        row_full_prev_[i] = row_full_[i];
        row_full_[i] = full;
    }

    void recompute_full_summary()
    {
        num_full_ = 0;
        full_bitset_ = 0;
        for (size_t i = 0; i < row_full_.size(); ++i)
        {
            if (row_full_[i])
            {
                ++num_full_;
                if (i < 32)
                    full_bitset_ |= (1u << i);
            }
        }
    }

    bool all_rows_full() const
    {
        return num_full_ == static_cast<int>(row_full_.size());
    }

    std::vector<int> full_rows_indices() const
    {
        std::vector<int> idx;
        for (size_t i = 0; i < row_full_.size(); ++i)
            if (row_full_[i])
                idx.push_back(static_cast<int>(i));
        return idx;
    }

    void init_rows_and_rois()
    {
        std::vector<std::vector<std::pair<int, int>>> all_corners = {
            // row_1
            {{740, 428}, {716, 77}, {803, 78}, {870, 428}},
            // row_2
            {{609, 429}, {624, 78}, {714, 78}, {734, 429}},
            // row_3
            {{484, 430}, {534, 84}, {618, 85}, {598, 430}},
            // row_4
            {{360, 433}, {450, 85}, {530, 84}, {460, 433}},
            // row_5
            {{205, 430}, {340, 88}, {440, 88}, {342, 430}}, 
            };  
     
        rois_.clear();
        rois_.reserve(all_corners.size());
        for (const auto &corners : all_corners)
        {
            rois_.push_back(ROIQuad::FromCorners(corners));
        }

        rows_.assign(6, DetectionCartridge{});

        for (size_t i = 0; i < rows_.size(); ++i)
        {
            rows_[i].target_class_id_ = "0";
        }
        // rows_[5].target_class_id_ = "3";
        // filter
        filters_.assign(6, RowFilter{});
        row_full_.assign(6, false);
        row_full_prev_.assign(6, false);
        num_full_ = 0;
        full_bitset_ = 0;
    }
    void yolov8CallbackCam5(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        float score_thresh_ = 0.45f;
        row6_cnt2_ = 0;
        row6_cnt3_ = 0;
        
        for (auto &row : rows_)
            row.reset();

        // ✅ OPTIMIZED: Check center only (no corners)
        for (const auto &det : msg->detections)
        {
            const float cx = det.bbox.center.position.x;
            const float cy = det.bbox.center.position.y;

            for (const auto &r : det.results)
            {
                const std::string &class_id = r.hypothesis.class_id;
                const float score = r.hypothesis.score;
                
                if (score <= score_thresh_)
                    continue;

                // Check rows 1-5
                for (size_t i = 0; i < 5; ++i)
                {
                    if (class_id == rows_[i].target_class_id_ && rois_[i].contains(cx, cy))
                    {
                        rows_[i].count++;
                        break;
                    }
                }
                
                // Row 6 (Scale)
                if (class_id == "2") row6_cnt2_++;
                else if (class_id == "3") row6_cnt3_++;
            }
        }

        // Filtering & Logging
        for (size_t i = 0; i < 5; ++i)
        {
            // Log raw count first
            if (rows_[i].ready())
            {
                RCLCPP_INFO(get_logger(), "Row %zu READY: %d/%d",
                    i + 1, rows_[i].count, rows_[i].ready_threshold);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Row %zu: %d/%d",
                    i + 1, rows_[i].count, rows_[i].ready_threshold);
            }

            // Apply filtering
            int filtered_count = filters_[i].filter_count(rows_[i].count);
            bool raw_ready = (rows_[i].count >= rows_[i].ready_threshold);
            bool stable_ready = filters_[i].update_ready(raw_ready);

            rows_[i].count = filtered_count;
            set_row_full(i, stable_ready);

            // Log filtered result
            RCLCPP_INFO(get_logger(),
                "Row %zu: raw=%d, filtered=%d, thr=%d, READY(raw)=%s, READY(stable)=%s (streak=%d/%d)",
                i + 1, rows_[i].count, filtered_count, rows_[i].ready_threshold,
                raw_ready ? "YES" : "NO",
                stable_ready ? "YES" : "NO",
                filters_[i].ready_streak, filters_[i].ready_consec);
        }

        // Row 6
        const int thr = rows_[0].ready_threshold;
        bool gate6 = (row6_cnt3_ >= thr) || ((row6_cnt2_ + row6_cnt3_) >= thr);
        bool full6_raw = (gate6 && row6_cnt3_ >= thr && row6_cnt2_ == 0);
        bool full6_stable = filters_[5].update_ready(full6_raw);
        set_row_full(5, full6_stable);
    }
    // camera tray output
    void buildCanonicalSlots()
    {
        canonical_slots_.clear();
        // row1 (Reasonable Expansion)
        canonical_slots_.push_back(SlotDef{{{185, 483}, {225, 368}, {568, 368}, {568, 483}}, 'H'});
        // row2
        canonical_slots_.push_back(SlotDef{{{220, 365}, {250, 268}, {573, 268}, {573, 365}}, 'H'});
        // row3
        canonical_slots_.push_back(SlotDef{{{250, 265}, {281, 174}, {576, 174}, {576, 265}}, 'H'});
        // row4
        canonical_slots_.push_back(SlotDef{{{285, 169}, {315, 85}, {605, 85}, {605, 171}}, 'H'});
        // row5 (middle vertical)
        canonical_slots_.push_back(SlotDef{{{575, 482}, {582, 180}, {684, 180}, {684, 483}}, 'V'});
        // row6 (User Defined)`
        canonical_slots_.push_back(SlotDef{{{700, 490}, {700, 375}, {1060, 375}, {1085, 490}}, 'H'});
        // row7 (User Defined)
        canonical_slots_.push_back(SlotDef{{{695, 375}, {695, 275}, {1040, 275}, {1070, 375}}, 'H'});
        // row8 (User Defined)
        canonical_slots_.push_back(SlotDef{{{692, 275}, {692, 180}, {1020, 180}, {1030, 275}}, 'H'});
        // row9 (User Defined)
        canonical_slots_.push_back(SlotDef{{{650, 175}, {650, 85}, {950, 85}, {965, 175}}, 'H'});
    }
    // ---------- Box từ detection ----------
    static Box detToBox(const Detection2D &det)
    {
        const auto &bb = det.bbox;
        double cx = bb.center.position.x;
        double cy = bb.center.position.y;
        double w = bb.size_x;
        double h = bb.size_y;
        return {cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2};
    }

    static char getOrientation(const Box &b)
    {
        double w = b.x2 - b.x1;
        double h = b.y2 - b.y1;
        return (w >= h) ? 'H' : 'V';
    }

    // Tạo AABB cho slot từ 4 góc (không giả định thứ tự)
    // static Box aabbFromCorners(const std::vector<cv::Point2f> &c)
    // {
    //     double xmin = std::numeric_limits<double>::infinity();
    //     double ymin = std::numeric_limits<double>::infinity();
    //     double xmax = -std::numeric_limits<double>::infinity();
    //     double ymax = -std::numeric_limits<double>::infinity();
    //     for (const auto &p : c)
    //     {
    //         xmin = std::min(xmin, (double)p.x);
    //         ymin = std::min(ymin, (double)p.y);
    //         xmax = std::max(xmax, (double)p.x);
    //         ymax = std::max(ymax, (double)p.y);
    //     }
    //     return {xmin, ymin, xmax, ymax};
    // }

    static Box polygonAABB(const std::vector<cv::Point2f> &poly)
    {
        double xmin = 1e18, ymin = 1e18;
        double xmax = -1e18, ymax = -1e18;
        for (const auto &p : poly)
        {
            xmin = std::min(xmin, (double)p.x);
            ymin = std::min(ymin, (double)p.y);
            xmax = std::max(xmax, (double)p.x);
            ymax = std::max(ymax, (double)p.y);
        }
        return {xmin, ymin, xmax, ymax};
    }

    static double IoU(const Box &A, const Box &B)
    {
        double xA = std::max(A.x1, B.x1);
        double yA = std::max(A.y1, B.y1);
        double xB = std::min(A.x2, B.x2);
        double yB = std::min(A.y2, B.y2);
        double iw = std::max(0.0, xB - xA);
        double ih = std::max(0.0, yB - yA);
        double inter = iw * ih;
        double areaA = (A.x2 - A.x1) * (A.y2 - A.y1);
        double areaB = (B.x2 - B.x1) * (B.y2 - B.y1);
        return inter / std::max(1e-6, areaA + areaB - inter);
    }

    // Warp 1 điểm bởi homography
    static cv::Point2f warpPoint(const cv::Point2f &pt, const cv::Mat &H)
    {
        cv::Mat src = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1.0);
        cv::Mat dst = H * src;
        double w = dst.at<double>(2, 0);
        return cv::Point2f(dst.at<double>(0, 0) / w, dst.at<double>(1, 0) / w);
    }

    static std::vector<cv::Point2f> shrinkPolygon(const std::vector<cv::Point2f> &poly, float ratio)
    {
        cv::Point2f c(0.f, 0.f);
        for (auto &p : poly)
        {
            c.x += p.x;
            c.y += p.y;
        }
        c.x /= (float)poly.size();
        c.y /= (float)poly.size();
        std::vector<cv::Point2f> out;
        out.reserve(poly.size());
        for (auto &p : poly)
        {
            out.emplace_back(c + (p - c) * ratio);
        }
        return out;
    }

    // Non-Maximum Suppression (greedy) cho dets
    static std::vector<int> nmsGreedy(const std::vector<Box> &boxes,
                                      const std::vector<double> &scores,
                                      double iou_thresh)
    {
        std::vector<int> idx(boxes.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int a, int b)
                  { return scores[a] > scores[b]; });

        std::vector<int> keep;
        std::vector<bool> removed(boxes.size(), false);

        for (size_t i = 0; i < idx.size(); ++i)
        {
            int ia = idx[i];
            if (removed[ia])
                continue;
            keep.push_back(ia);
            for (size_t j = i + 1; j < idx.size(); ++j)
            {
                int ib = idx[j];
                if (removed[ib])
                    continue;
                if (IoU(boxes[ia], boxes[ib]) >= iou_thresh)
                    removed[ib] = true;
            }
        }
        return keep;
    }

    // ---- Try build tray corners from detection ID=0 (bbox rectangle -> TL,BL,TR,BR) ----
    static bool trayFromDetections(const Detection2DArray &arr, std::vector<cv::Point2f> &out_corners)
    {
        for (const auto &d : arr.detections)
        {
            if (d.results.empty())
                continue;
            // class_id trong vision_msgs là string
            int cid = -1;
            try
            {
                cid = std::stoi(d.results[0].hypothesis.class_id);
            }
            catch (...)
            {
                continue;
            }
            if (cid != 0)
                continue; // only tray

            Box b = detToBox(d);
            // thứ tự: TL, BL, TR, BR (khớp cách mình khai báo canonical_tray_)
            out_corners.clear();
            out_corners.emplace_back((float)b.x1, (float)b.y2); // TL? Wait: image origin top-left.
            // ảnh gốc: (x1,y1) = top-left. ta cần: (46,479)=bottom-left ở canonical? bạn đã khai khác.
            // Để đơn giản: chọn theo quy ước (BL, TL, TR, BR) giống canonical bạn cung cấp:
            out_corners.clear();
            out_corners.emplace_back((float)b.x1, (float)b.y2); // BL
            out_corners.emplace_back((float)b.x1, (float)b.y1); // TL
            out_corners.emplace_back((float)b.x2, (float)b.y1); // TR
            out_corners.emplace_back((float)b.x2, (float)b.y2); // BR
            return true;
        }
        return false;
    }

    // Trả về vị trí trống hợp lệ theo quy tắc "liên tục từ 1..N"
    // Nếu vị trí trống bắt đầu không phải từ 1 (liên tục) -> trả -1
    static int select_contiguous_empty(const std::vector<int> &empty_slots, int total_slots = 9)
    {
        // tạo mảng đánh dấu trống
        std::vector<bool> is_empty(total_slots + 1, false);
        for (int id : empty_slots)
        {
            if (id >= 1 && id <= total_slots)
                is_empty[id] = true;
        }

        // duyệt từ 1..N
        for (int i = 1; i <= total_slots; ++i)
        {
            if (is_empty[i])
            {
                // kiểm tra tất cả slot trước i phải không trống
                bool all_full_before = true;
                for (int j = 1; j < i; ++j)
                {
                    if (is_empty[j])
                    {
                        all_full_before = false;
                        break;
                    }
                }
                if (all_full_before)
                    return i;
                else
                    return -1; // có ô trống trước đó bị bỏ qua -> không hợp lệ
            }
        }
        return -1; // không có ô nào trống
    }

    // ---------- main callback ----------
    void detCallback(const Detection2DArray::SharedPtr msg)
    {
        // (1) xác định 4 góc khay hiện tại
        std::vector<cv::Point2f> detected_tray;
        bool has_tray = false;
        if (use_tray_from_detection_)
        {
            has_tray = trayFromDetections(*msg, detected_tray);
            if (!has_tray && debug_logs_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "No class-0 tray detected; fallback to canonical tray.");
            }
        }
        if (!has_tray)
            detected_tray = canonical_tray_;

        // (2) homography canonical -> detected
        cv::Mat H = cv::getPerspectiveTransform(canonical_tray_, detected_tray);

        // (3) warp + shrink slots
        struct WarpSlot
        {
            std::vector<cv::Point2f> poly;
            Box aabb;
            char orient;
        };
        std::vector<WarpSlot> slots;
        slots.reserve(canonical_slots_.size());
        for (const auto &s : canonical_slots_)
        {
            std::vector<cv::Point2f> poly;
            poly.reserve(4);
            for (const auto &p : s.corners)
                poly.push_back(warpPoint(p, H));
            poly = shrinkPolygon(poly, (float)slot_shrink_ratio_);
            slots.push_back({poly, polygonAABB(poly), s.orient});
        }

        if (debug_logs_)
        {
            for (size_t i = 0; i < slots.size(); ++i)
            {
                const auto bb = slots[i].aabb;
                RCLCPP_INFO(this->get_logger(),
                            "Slot %zu AABB: (%.1f,%.1f)-(%.1f,%.1f) orient=%c",
                            i + 1, bb.x1, bb.y1, bb.x2, bb.y2, slots[i].orient);
            }
        }

        // (4) lọc detections: chỉ 1/2/3 + score >= score_thresh + NMS
        struct DetRec
        {
            Box box;
            char orient;
            size_t idx;
            double score;
        };
        std::vector<DetRec> dets_raw;
        dets_raw.reserve(msg->detections.size());

        for (size_t i = 0; i < msg->detections.size(); ++i)
        {
            const auto &d = msg->detections[i];
            if (d.results.empty())
                continue;

            if (debug_logs_) {
                RCLCPP_INFO(this->get_logger(), "Raw Det %zu: class=%s score=%.2f box=(%.1f,%.1f,%.1f,%.1f)",
                            i, d.results[0].hypothesis.class_id.c_str(), d.results[0].hypothesis.score, 
                            detToBox(d).x1, detToBox(d).y1, detToBox(d).x2, detToBox(d).y2);
            }

            int cid = -1;
            try { cid = std::stoi(d.results[0].hypothesis.class_id); } catch (...) { continue; }

            // Allow 1, 2, 3 and also 0 if it's high enough score (might be misclassified cartridge)
            if (cid != 1 && cid != 2 && cid != 3 && cid != 0)
                continue;

            double score = d.results[0].hypothesis.score;
            if (score < score_thresh_o)
                continue;

            Box b = detToBox(d);
            char o = getOrientation(b);
            dets_raw.push_back({b, o, i, score});
        }

        // NMS
        std::vector<Box> nms_boxes;
        nms_boxes.reserve(dets_raw.size());
        std::vector<double> nms_scores;
        nms_scores.reserve(dets_raw.size());
        for (auto &d : dets_raw)
        {
            nms_boxes.push_back(d.box);
            nms_scores.push_back(d.score);
        }
        std::vector<int> keep = nmsGreedy(nms_boxes, nms_scores, nms_iou_thresh_);

        std::vector<DetRec> dets;
        dets.reserve(keep.size());
        for (int k : keep)
            dets.push_back(dets_raw[k]);

        if (debug_logs_)
        {
            for (const auto &d : dets)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Detection KEPT: score=%.2f, box=(%.1f,%.1f,%.1f,%.1f), orient=%c",
                            d.score, d.box.x1, d.box.y1, d.box.x2, d.box.y2, d.orient);
            }
        }

        // (5) xây candidate (slot,det,score)
        struct Cand
        {
            int slot;
            int det;
            double score;
            bool inside;
            double iou;
            bool orient_ok;
        };
        std::vector<Cand> cands;
        cands.reserve(slots.size() * std::max<size_t>(1, dets.size()));
        for (int s = 0; s < (int)slots.size(); ++s)
        {
            const auto &ws = slots[s];
            for (int d = 0; d < (int)dets.size(); ++d)
            {
                const auto &dr = dets[d];
                cv::Point2f center((float)((dr.box.x1 + dr.box.x2) / 2.0),
                                   (float)((dr.box.y1 + dr.box.y2) / 2.0));
                
                // Check center and 4 corners for better sensitivity
                bool inside = (cv::pointPolygonTest(ws.poly, center, false) >= 0);
                if (!inside) {
                    cv::Point2f cpts[4] = {
                        {(float)dr.box.x1, (float)dr.box.y1},
                        {(float)dr.box.x2, (float)dr.box.y1},
                        {(float)dr.box.x1, (float)dr.box.y2},
                        {(float)dr.box.x2, (float)dr.box.y2}
                    };
                    for (int k = 0; k < 4; ++k) {
                        if (cv::pointPolygonTest(ws.poly, cpts[k], false) >= 0) {
                            inside = true;
                            break;
                        }
                    }
                }
                
                double iou = IoU(ws.aabb, dr.box);
                
                
                // Stricter threshold: reject low IoU detections
                // Even if inside, reject if IoU is too low (likely tray background noise)
                if (iou < 0.3) 
                    continue;

                // Score logic: inside gets huge bonus (1.0). 
                // BUT if IoU is decent (> 0.3), also give significant bonus (0.8) 
                // to prioritize it over noise.
                double score = iou;
                if (inside) 
                    score += 1.0;
                else if (iou > 0.3)
                    score += 0.8;

                bool orient_ok = (dr.orient == ws.orient);
                if (orient_ok)
                    score += 0.1;
                cands.push_back({s, d, score, inside, iou, orient_ok});
            }
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand &a, const Cand &b)
                  { return a.score > b.score; });

        // (6) one-to-one assignment (greedy)
        std::vector<int> slot_assigned(slots.size(), -1); // slot -> det
        std::vector<int> det_assigned(dets.size(), -1);   // det  -> slot
        for (const auto &c : cands)
        {
            if (slot_assigned[c.slot] != -1)
                continue;
            if (det_assigned[c.det] != -1)
                continue;
            slot_assigned[c.slot] = c.det;
            det_assigned[c.det] = c.slot;

            if (debug_logs_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Assign: Slot %d <-> Det %d | score=%.3f inside=%d IoU=%.3f orient_ok=%d",
                            c.slot + 1, c.det + 1, c.score, c.inside ? 1 : 0, c.iou, c.orient_ok ? 1 : 0);
            }
        }

        // (7) trạng thái tức thời & cập nhật ổn định
        std::vector<SlotStableState> instant(slots.size(), SlotStableState::EMPTY);
        for (int s = 0; s < (int)slots.size(); ++s)
        {
            if (slot_assigned[s] == -1)
            {
                instant[s] = SlotStableState::EMPTY;
            }
            else
            {
                int d = slot_assigned[s];
                // For user's case: treat misoriented as occupied (object is present)
                // Slot 5 (index 4) should be occupied if anything detected
                if (s == 4 || dets[d].orient == slots[s].orient) {
                    instant[s] = SlotStableState::OCC_OK;
                } else {
                    // For other slots: still mark as occupied even if misoriented
                    instant[s] = SlotStableState::OCC_OK;
                }
            }
        }

        // Debounce: cập nhật streak & state ổn định
        for (int s = 0; s < (int)slots.size(); ++s)
        {
            if (instant[s] == SlotStableState::EMPTY)
            {
                empty_streak_[s]++;
                occ_ok_streak_[s] = 0;
                mis_streak_[s] = 0;
                if (empty_streak_[s] >= confirm_frames_)
                    stable_state_[s] = SlotStableState::EMPTY;
            }
            else if (instant[s] == SlotStableState::OCC_OK)
            {
                occ_ok_streak_[s]++;
                empty_streak_[s] = 0;
                mis_streak_[s] = 0;
                if (occ_ok_streak_[s] >= confirm_frames_)
                    stable_state_[s] = SlotStableState::OCC_OK;
            }
            else
            { // MIS
                mis_streak_[s]++;
                empty_streak_[s] = 0;
                occ_ok_streak_[s] = 0;
                if (mis_streak_[s] >= confirm_frames_)
                    stable_state_[s] = SlotStableState::MIS;
            }
        }

        // (8) publish theo trạng thái ổn định
        std::vector<int> empty_slots, mis_slots;
        for (int s = 0; s < (int)slots.size(); ++s)
        {
            if (stable_state_[s] == SlotStableState::EMPTY)
                empty_slots.push_back(s + 1);
            else if (stable_state_[s] == SlotStableState::MIS)
                mis_slots.push_back(s + 1);
        }

        std_msgs::msg::Int32MultiArray out_empty;
        out_empty.data = empty_slots;
        std_msgs::msg::Int32MultiArray out_mis;
        out_mis.data = mis_slots;
        // pub_empty_->publish(out_empty);
        // pub_misoriented_->publish(out_mis);
        int selected = select_contiguous_empty(empty_slots, (int)slots.size());
        slot_id = selected;

        if (debug_logs_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Stable -> Empty=%s | Misoriented=%s | Selected empty=%d",
                        vecToStr(empty_slots).c_str(), vecToStr(mis_slots).c_str(), selected);
        }
    }

    static std::string vecToStr(const std::vector<int> &v)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < v.size(); ++i)
        {
            oss << v[i] << (i + 1 < v.size() ? ", " : "");
        }
        oss << "]";
        return oss.str();
    }


    void moveAndConfirm(size_t idx)
    {
        RCLCPP_INFO(get_logger(), "[moveAndConfirm] Moving to point %zu", idx + 1);
        auto req = std::make_shared<JointMovJ::Request>();
        auto &ang = joint_sequences_[idx];
        req->j1 = ang[0];
        req->j2 = ang[1];
        req->j3 = ang[2];
        req->j4 = ang[3];
        req->j5 = ang[4];
        req->j6 = ang[5];
        req->param_value.clear();
        callService<JointMovJ>(joint_client_, req, "JointMovJ");

        waitUntilJointReached(ang);
    }

    bool waitUntilJointReached(const std::vector<double> &target, double timeout_sec = 15.0)
    {
        rclcpp::Time start = now();
        while ((now() - start).seconds() < timeout_sec)
        {
            auto req = std::make_shared<GetAngle::Request>();
            auto res = callService<GetAngle>(angle_client_, req, "GetAngle");
            if (res)
            {
                std::vector<double> angles;
                std::string cleaned = res->angle;
                // RCLCPP_INFO(get_logger(), "[GetAngle] Angle from robot %s", cleaned.c_str());
                cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '{'), cleaned.end());
                cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '}'), cleaned.end());

                std::istringstream ss(cleaned);
                std::string token;
                while (std::getline(ss, token, ','))
                {
                    try
                    {
                        angles.push_back(std::stod(token));
                    }
                    catch (const std::invalid_argument &e)
                    {
                        RCLCPP_ERROR(get_logger(), "[GetAngle] Invalid angle token: '%s'", token.c_str());
                        return false;
                    }
                }
                if (angles.size() >= 6)
                {
                    bool close = true;
                    for (size_t i = 0; i < 6; ++i)
                    {
                        if (std::abs(angles[i] - target[i]) > 1.0)
                        {
                            close = false;
                            break;
                        }
                    }
                    if (close)
                    {
                        RCLCPP_INFO(get_logger(), "[Reach] Joint angles matched target.");
                        return true;
                    }
                }
            }
            rclcpp::sleep_for(500ms);
        }
        RCLCPP_WARN(get_logger(), "[Reach] Timeout waiting for joint angles.");
        return false;
    }

    void sendDigitalOutput(int index, int status)
    {
        RCLCPP_INFO(this->get_logger(), "[DO] Setting DO[%d] = %d", index, status);

        auto req = std::make_shared<DO::Request>();
        req->index = index;
        req->status = status;

        callService<DO>(do_client_, req, "DO");
    }

    void setLinearSpeed(int speed)
    {
        auto req = std::make_shared<SpeedL::Request>();
        req->r = speed;
        callService<SpeedL>(speedl_client_, req, "SpeedL");
    }

    void setJointSpeed(int speed)
    {
        auto req = std::make_shared<SpeedJ::Request>();
        req->r = speed;
        callService<SpeedJ>(speedj_client_, req, "SpeedJ");
    }

    void setLinearAccel(int percent)
    {
        auto req = std::make_shared<AccL::Request>();
        req->r = std::clamp(percent, 1, 100);
        callService<AccL>(accl_client_, req, "AccL");
    }

    void setJointAccel(int percent)
    {
        auto req = std::make_shared<AccJ::Request>();
        req->r = std::clamp(percent, 1, 100);
        callService<AccJ>(accj_client_, req, "AccJ");
    }

    bool moveRelAndWaitUntilReached(double dx, double dy, double dz, double timeout_sec = 10.0, double tol = 5.0)
    {
        auto before = getCurrentPose();
        if (before.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "[RelMovL] Failed to get current pose.");
            return false;
        }

        // Calculate target pose
        double target_x = before[0] + dx;
        double target_y = before[1] + dy;
        double target_z = before[2] + dz;

        // Send command RelMovL
        auto req = std::make_shared<RelMovL::Request>();
        req->offset1 = dx;
        req->offset2 = dy;
        req->offset3 = dz;
        req->offset4 = 0.0;
        req->offset5 = 0.0;
        req->offset6 = 0.0;
        req->param_value.clear();

        callService<RelMovL>(relmovl_client_, req, "RelMovL");

        // Monitoring realtime until to target
        rclcpp::Time start = this->now();
        while ((this->now() - start).seconds() < timeout_sec)
        {
            auto current = getCurrentPose();
            if (current.size() < 3)
                continue;

            double dx_err = std::abs(current[0] - target_x);
            double dy_err = std::abs(current[1] - target_y);
            double dz_err = std::abs(current[2] - target_z);

            double dist_err = std::sqrt(dx_err * dx_err + dy_err * dy_err + dz_err * dz_err);
            RCLCPP_INFO(this->get_logger(), "[RelMovL] Err to target: %.2f mm", dist_err);

            if (dist_err <= tol)
            {
                RCLCPP_INFO(this->get_logger(), "[RelMovL] Robot reached target pose.");
                return true;
            }

            rclcpp::sleep_for(200ms);
        }

        RCLCPP_WARN(this->get_logger(), "[RelMovL] Timeout. Robot did not reach target pose.");
        return false;
    }

    std::vector<double> getCurrentPose()
    {
        std::vector<double> pose;

        auto req = std::make_shared<GetPose::Request>();
        req->user = 0;
        req->tool = 0;

        auto res = callService<GetPose>(pose_client_, req, "GetPose");
        if (res)
        {
            // RCLCPP_INFO(get_logger(), "[GetPose] raw pose string: %s", res->pose.c_str());

            std::string pose_str = res->pose;

            pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '{'), pose_str.end());
            pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '}'), pose_str.end());
            std::stringstream ss(pose_str);
            std::string token;
            while (std::getline(ss, token, ','))
            {
                try
                {
                    pose.push_back(std::stod(token));
                }
                catch (...)
                {
                    RCLCPP_WARN(get_logger(), "[GetPose] Failed to parse token: %s", token.c_str());
                }
            }

            // if (pose.size() >= 3)
            // {
            //     RCLCPP_INFO(get_logger(), "[GetPose] Parsed pose: X=%.1f Y=%.1f Z=%.1f", pose[0], pose[1], pose[2]);
            // }
            // else
            // {
            //     RCLCPP_WARN(get_logger(), "[GetPose] Incomplete pose data.");
            // }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "[GetPose] Service call failed.");
        }

        return pose;
    }

    int getSlotIdFromPosition(int row, int col, int rows = 4, int cols = 2)
    {
        if (row >= 0 && row < rows && col >= 0 && col < cols)
        {
            return row * cols + col + 1;
        }
        return -1; // Invalid
    }

    int findPreferredEmptySlot(const bool occupied[4][2], const bool blocked[4][2])
    {
        std::vector<std::pair<int, int>> preferred_order = {
            {0, 1}, {1, 1}, {2, 1}, {3, 1}, // 2 → 4 → 6 → 8
            {0, 0},
            {1, 0},
            {2, 0},
            {3, 0} // 1 → 3 → 5 → 7
        };

        for (const auto &[r, c] : preferred_order)
        {
            if (!occupied[r][c] && !blocked[r][c])
            {
                slot_id_full = 0;
                return r * 2 + c + 1; // slot_id từ 1 đến 8
            }
        }
        slot_id_full = 100;
        return -1; // Không tìm thấy slot trống
    }

    void publishEmptySlots(const bool occupied[4][2],
                           const bool blocked[4][2],
                           float tray_cx, float tray_cy,
                           float tray_w, float tray_h,
                           int rows, int cols, bool tray)
    {
        // bool tray = false;
        vision_msgs::msg::Detection2DArray msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "empty_slot_frame";

        rows = 4;
        cols = 2;

        float cell_w = tray_w / cols;
        float cell_h = tray_h / rows;

        if (!tray)
        {
            empty_slot_pub_->publish(msg);
            return;
        }

        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                if (!occupied[r][c] && !blocked[r][c])
                {
                    float cx = tray_cx - tray_w / 2 + (c + 0.5f) * cell_w;
                    float cy = tray_cy - tray_h / 2 + (r + 0.5f) * cell_h;

                    vision_msgs::msg::Detection2D det;
                    float shrink_px = 10.0;
                    det.id = std::to_string(r * 2 + c + 1);
                    det.bbox.center.position.x = cx;
                    det.bbox.center.position.y = cy;
                    if (!tray)
                    {
                        det.bbox.size_x = 0.0;
                        det.bbox.size_y = 0.0;
                    }
                    else if (std::isnan(cx) || std::isnan(cy) || cell_w <= 0 || cell_h <= 0)
                    {
                        det.bbox.size_x = 0.0;
                        det.bbox.size_y = 0.0;
                    }
                    else
                    {
                        det.bbox.size_x = std::max(cell_w - shrink_px, 1.0f);
                        det.bbox.size_y = std::max(cell_h - shrink_px, 1.0f);
                    }
                    // Add class_id = "empty"
                    vision_msgs::msg::ObjectHypothesisWithPose hypo;
                    hypo.hypothesis.class_id = "empty";
                    hypo.hypothesis.score = 1.0;
                    det.results.push_back(hypo);

                    msg.detections.push_back(det);
                }
            }
        }

        empty_slot_pub_->publish(msg);
    }
    void callEnable()
    {
        RCLCPP_INFO(get_logger(), "Start!");
        writeToPLC("192.168.27.6", 185, 0, {0, 0});
        writeToPLC("192.168.27.6", 185, 4, {0, 0});
        writeToPLC("192.168.27.6", 185, 8, {0, 0});
        // auto req = std::make_shared<EnableRobot::Request>();
        // req->load = 3.5;
        // callService<EnableRobot>(enable_client_, req, "EnableRobot");
        if (!checkPLCConnection(1000))
        {
            RCLCPP_WARN(get_logger(), "PLC not reachable, please check cable/IP.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "PLC connection OK!");
        }
    }

    void run_next_step()
    {
        /*
        if (current_step_ < steps_.size())
        {
            const auto &step = steps_[current_step_];
            RCLCPP_INFO(this->get_logger(), "Running step: %s", step.first.c_str());
            step.second(); // call function
        }
        if (current_step_ >= steps_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All steps completed.");
            return;
        }

        auto [name, func] = steps_[current_step_];
        RCLCPP_INFO(this->get_logger(), "Running step: %s", name.c_str());

        std::thread([func]()
                    {
                        func();
                    })
            .detach();
        */
        timer_->cancel();
        std::thread([this]()
                    {
                  callEnable();
                  runMotionPlan(); })
            .detach();
    }

    void takeCartridgeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        take_cartridge = msg->data;
        // if (take_cartridge)
        // {
        //     rclcpp::sleep_for(2000ms);
        //     take_cartridge = false;
        // }
    }
    void writeToPLC(const std::string &ip, int db, int offset,
                    const std::vector<int32_t> &vals)
    {
        PLCWrite w;
        w.plc_ip = ip;
        w.db_number = db;
        w.offset = offset;

        using V = typename decltype(w.values)::value_type; // thường int16_t
        w.values.reserve(vals.size());
        for (auto v32 : vals)
        {
            long v = static_cast<long>(v32);
            if (v < std::numeric_limits<V>::min())
                v = std::numeric_limits<V>::min();
            if (v > std::numeric_limits<V>::max())
                v = std::numeric_limits<V>::max();
            w.values.push_back(static_cast<V>(v));
        }

        write_pub_->publish(w);

        std::ostringstream oss;
        oss << "[WRITE] ip=" << w.plc_ip << " db=" << w.db_number << " off=" << w.offset << " values=[";
        for (size_t i = 0; i < w.values.size(); ++i)
            oss << static_cast<long>(w.values[i]) << (i + 1 < w.values.size() ? ", " : "");
        oss << "]";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    void publishReadRequest()
    {
        PLCRead msg;
        msg.plc_ip = plc_ip_read_;
        msg.db_number = db_number_;
        msg.offset = read_offset_;
        msg.size = read_size_;
        read_pub_->publish(msg);
        // RCLCPP_INFO(get_logger(), "Published PLCRead: ip=%s db=%d off=%d size=%d",
        //             msg.plc_ip.c_str(), msg.db_number, msg.offset, msg.size);
    }

    void onReadResponse(const PLCResponse::SharedPtr msg)
        {
            // In log giá trị đọc được
            std::ostringstream oss;
            oss << "[READ RESP] count=" << msg->values.size() << " values=[";
            for (size_t i = 0; i < msg->values.size(); ++i)
            {
                oss << msg->values[i] << (i + 1 < msg->values.size() ? ", " : "");
            }
            oss << "]";
            RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

            // ==============================
            //  ✔ CHECK SIZE TRƯỚC KHI TRUY CẬP
            // ==============================
            if (msg->values.size() < 6)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[PLC ERROR] Invalid response size=%ld (need >=6). Skipping cycle...",
                    msg->values.size());

                return;  // QUAN TRỌNG: KHÔNG XỬ LÝ TIẾP
            }

            // ==============================
            //  ✔ SAFE ACCESS - GUARANTEED
            // ==============================
            cmdCheckTakeCartridge = msg->values[0];
            valueTakeCartridge    = msg->values[1];
            cmdCheckTray          = msg->values[4];
            valueCheckTray        = msg->values[5];

            RCLCPP_INFO(
                this->get_logger(),
                "[PLC CMD] Take: %d (v: %d), Tray: %d (v: %d)",
                cmdCheckTakeCartridge, valueTakeCartridge,
                cmdCheckTray, valueCheckTray);
        }


    void onWriteValues(const std_msgs::msg::Int32MultiArray::SharedPtr arr)
    {
        if (arr->data.empty())
        {
            RCLCPP_WARN(get_logger(), "Write ignored: empty values.");
            return;
        }

        PLCWrite w;
        w.plc_ip = plc_ip_write_;
        w.db_number = db_number_;
        w.offset = write_offset_;

        using WriteValueT = typename decltype(w.values)::value_type;
        const long min_v = static_cast<long>(std::numeric_limits<WriteValueT>::min());
        const long max_v = static_cast<long>(std::numeric_limits<WriteValueT>::max());

        w.values.reserve(arr->data.size());
        for (int32_t v32 : arr->data)
        {
            long v = static_cast<long>(v32);
            if (v < min_v)
                v = min_v;
            if (v > max_v)
                v = max_v;
            w.values.push_back(static_cast<WriteValueT>(v));
        }

        write_pub_->publish(w);

        std::ostringstream oss;
        oss << "Published PLCWrite: ip=" << w.plc_ip
            << " db=" << w.db_number
            << " off=" << w.offset
            << " values=[";
        for (size_t i = 0; i < w.values.size(); ++i)
        {
            oss << static_cast<long>(w.values[i]) << (i + 1 < w.values.size() ? ", " : "");
        }
        oss << "]";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    bool checkPLCConnection(int timeout_ms = 1000)
    {
        PLCRead req;
        req.plc_ip = plc_ip_read_;
        req.db_number = db_number_;
        req.offset = 0;
        req.size = 40;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            new_resp_ = false;
        }
        read_pub_->publish(req);

        std::unique_lock<std::mutex> lk(mtx_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this]
                          { return new_resp_; }))
        {
            RCLCPP_ERROR(get_logger(), "[checkPLCConnection] No response from PLC %s within %d ms",
                         plc_ip_read_.c_str(), timeout_ms);
            return false;
        }
        new_resp_ = false;

        RCLCPP_INFO(get_logger(), "[checkPLCConnection] PLC %s is reachable", plc_ip_read_.c_str());
        return true;
    }
};

int main(int argc, char **argv)
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RobotLogicNode>());
    // rclcpp::shutdown();
    // return 0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotLogicNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}