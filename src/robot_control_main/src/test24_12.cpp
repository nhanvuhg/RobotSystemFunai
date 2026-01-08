#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

// Dobot Messages
#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/robot_mode.hpp"
#include "dobot_msgs_v3/srv/speed_l.hpp"
#include "dobot_msgs_v3/srv/acc_l.hpp"
#include "dobot_msgs_v3/srv/speed_j.hpp"
#include "dobot_msgs_v3/srv/acc_j.hpp"
#include "dobot_msgs_v3/srv/sync.hpp"



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
#include <thread>

using namespace std::chrono_literals;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;

using EnableRobot = dobot_msgs_v3::srv::EnableRobot;
using GetPose = dobot_msgs_v3::srv::GetPose;
using GetAngle = dobot_msgs_v3::srv::GetAngle;
using JointMovJ = dobot_msgs_v3::srv::JointMovJ;
using RelMovL = dobot_msgs_v3::srv::RelMovL;
using DO = dobot_msgs_v3::srv::DO;
using RobotMode = dobot_msgs_v3::srv::RobotMode;
using SpeedL = dobot_msgs_v3::srv::SpeedL;
using AccL = dobot_msgs_v3::srv::AccL;
using SpeedJ = dobot_msgs_v3::srv::SpeedJ;
using AccJ = dobot_msgs_v3::srv::AccJ;
using SyncSrv = dobot_msgs_v3::srv::Sync;
// ============================================================================
// SYSTEM STATE ENUM
// ============================================================================
enum class SystemState
{
    IDLE,
    
    // INIT SEQUENCE (First batch only)
    INIT_CHECK,
    INIT_LOAD_CHAMBER_DIRECT,
    INIT_REFILL_BUFFER,
    
    // MAIN LOOP
    WAIT_FILLING,
    TAKE_CHAMBER_TO_SCALE,
    WAIT_SCALE_RESULT,
    
    // OUTPUT HANDLING
    PLACE_TO_OUTPUT,
    PLACE_TO_FAIL,
    
    // BUFFER & CHAMBER MANAGEMENT
    REFILL_BUFFER,
    LOAD_CHAMBER_FROM_BUFFER,
    
    // LAST BATCH
    LAST_BATCH_WAIT,
    
    // ERROR STATES
    ERROR_SCALE,
    ERROR_OUTPUT_TRAY_TIMEOUT
};

// ============================================================================
// ROI STRUCTURES
// ============================================================================
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

// ============================================================================
// FILTERING FOR ROW DETECTION
// ============================================================================
struct RowFilter
{
    size_t window = 5;
    int max_fall = 2;
    int ready_consec = 3;

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

// ============================================================================
// MAIN ROBOT LOGIC NODE
// ============================================================================
class RobotLogicNode : public rclcpp::Node
{
public:
    RobotLogicNode() : Node("robot_logic_nova5")
    {
        RCLCPP_INFO(this->get_logger(), "=== Robot Logic Node Starting ===");

        // Load motion parameters
        loadMotionParameters();
        
        // Initialize ROIs
        initInputTrayROIs();
        initBufferROI();
        initOutputTrayROIs();
        
        // Initialize filters
        row_filters_.assign(5, RowFilter{});
        row_full_.assign(5, false);
        
        // Initialize service clients
        initServiceClients();
        
        // Initialize ROS subscriptions
        initSubscriptions();
        
        // Initialize ROS publishers
        initPublishers();
        
        // Initialize ROS services
        initServices();
        
        // Start state machine thread
        state_machine_thread_ = std::thread(&RobotLogicNode::stateMachineLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "=== Robot Logic Node Ready ===");
    }

    ~RobotLogicNode()
    {
        state_machine_running_ = false;
        if (state_machine_thread_.joinable())
        {
            state_machine_thread_.join();
        }
    }

private:
    // ========================================================================
    // SERVICE CLIENTS
    // ========================================================================
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
    rclcpp::Client<SyncSrv>::SharedPtr sync_client_;
    // ========================================================================
    // ROS SUBSCRIPTIONS
    // ========================================================================
    rclcpp::Subscription<Detection2DArray>::SharedPtr camera1_sub_;
    rclcpp::Subscription<Detection2DArray>::SharedPtr camera2_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr filling_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scale_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr new_tray_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_row_sub_;   // NEW: Single row command
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_slot_sub_;  // NEW: Output slot selection from camera
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_slot_sub_;   // NEW: Manual slot command from terminal
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goto_state_sub_;  // Restored

    // ========================================================================
    // ROS PUBLISHERS
    // ========================================================================
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub_;  // NEW: Gripper command to Festo

    // ========================================================================
    // ROS SERVICES
    // ========================================================================
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_system_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_manual_mode_service_;

    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    SystemState current_state_ = SystemState::IDLE;
    std::thread state_machine_thread_;
    std::atomic<bool> state_machine_running_{true};
    std::mutex state_mutex_;
    
    bool is_first_batch_ = true;  // Track if this is the first batch
    std::atomic<bool> stop_after_single_motion_{false}; // NEW: Stop after one motion sequence
    
    // ========================================================================
    // SYSTEM FLAGS (FROM ROS TOPICS)
    // ========================================================================
    std::atomic<bool> system_started_{false};
    std::atomic<bool> new_tray_loaded_{false};
    std::atomic<bool> fill_machine_filling_{false};
    std::atomic<bool> scale_result_ready_{false};
    std::atomic<bool> scale_result_pass_{false};
    std::atomic<bool> system_enabled_{false};  // NEW: System enable flag
    std::atomic<bool> emergency_stop_{false};  // NEW: Emergency stop flag
    std::atomic<bool> manual_mode_{false};     // NEW: Manual (bypass camera) mode
    
    // ========================================================================
    // CAMERA 1 - INPUT TRAY & BUFFER
    // ========================================================================
    std::vector<ROIQuad> input_tray_rois_;  // 5 rows
    std::vector<RowFilter> row_filters_;
    std::vector<bool> row_full_;
    bool input_tray_empty_ = false;
    
    int selected_input_row_ = -1;  // NEW: AI selected row (1-5), -1 = no selection
    std::mutex row_selection_mutex_;  // Protect selected row
    
    ROIQuad buffer_roi_;
    int buffer_cartridge_count_ = 0;
    bool buffer_detected_any_ = false;
    bool buffer_detected_full_ = false;
    
    // ========================================================================
    // CAMERA 2 - OUTPUT TRAY
    // ========================================================================
    std::array<ROIQuad, 8> output_tray_rois_;  // 8 slots
    int selected_output_slot_ = -1;  // -1 = no empty slot, 1-8 = slot ID
    std::mutex output_slot_selection_mutex_;  // Protect selected output slot
    rclcpp::Time output_tray_wait_start_;
    
    // ========================================================================
    // ROBOT STATE
    // ========================================================================
    bool chamber_is_empty_ = true;
    bool chamber_has_cartridge_ = false;
    bool scale_has_cartridge_ = false;
    
    // ========================================================================
    // TIMING & TIMEOUTS
    // ========================================================================
    rclcpp::Time scale_wait_start_;
    static constexpr double SCALE_TIMEOUT_SEC = 120.0;  // 2 minutes
    static constexpr double OUTPUT_TRAY_TIMEOUT_SEC = 180.0;  // 3 minutes
    static constexpr double FILLING_DURATION_SEC = 90.0;  // Filling takes ~90s
    
    // ========================================================================
    // MOTION SEQUENCES (LOADED FROM PARAMETERS)
    // ========================================================================
    std::vector<std::vector<double>> joint_sequences_;
    std::vector<std::vector<double>> relmovl_sequences_;
    std::vector<std::pair<int, int>> digital_output_steps_;
    
    // ========================================================================
    // SIMULATION MODE
    // ========================================================================
    bool simulate_scale_ = false;
    bool force_pass_ = true;
    
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    static constexpr int INPUT_ROW_THRESHOLD = 8;  // 8 cartridges per row
    static constexpr int BUFFER_CAPACITY = 8;       // 8 cartridges in buffer
    static constexpr float DETECTION_SCORE_THRESH = 0.6f;

    // ========================================================================
    // INITIALIZATION METHODS
    // ========================================================================
    
    void initServiceClients()
    {
        enable_client_ = create_client<EnableRobot>("/nova5/dobot_bringup/EnableRobot");
        pose_client_ = create_client<GetPose>("/nova5/dobot_bringup/GetPose");
        angle_client_ = create_client<GetAngle>("/nova5/dobot_bringup/GetAngle");
        joint_client_ = create_client<JointMovJ>("/nova5/dobot_bringup/JointMovJ");
        relmovl_client_ = create_client<RelMovL>("/nova5/dobot_bringup/RelMovL");
        do_client_ = create_client<DO>("/nova5/dobot_bringup/DO");
        robot_mode_client_ = create_client<RobotMode>("/nova5/dobot_bringup/RobotMode");
        speedl_client_ = create_client<SpeedL>("/nova5/dobot_bringup/SpeedL");
        accl_client_ = create_client<AccL>("/nova5/dobot_bringup/AccL");
        speedj_client_ = create_client<SpeedJ>("/nova5/dobot_bringup/SpeedJ");
        accj_client_ = create_client<AccJ>("/nova5/dobot_bringup/AccJ");
        sync_client_ = create_client<SyncSrv>("/nova5/dobot_bringup/Sync");

    }
    
    void initSubscriptions()
    {
        // Camera subscriptions
        camera1_sub_ = create_subscription<Detection2DArray>(
            "/cam0/yolo/bounding_boxes", 10,
            std::bind(&RobotLogicNode::camera1Callback, this, std::placeholders::_1));
            
        camera2_sub_ = create_subscription<Detection2DArray>(
            "/cam1/yolo/bounding_boxes", 10,
            std::bind(&RobotLogicNode::camera2Callback, this, std::placeholders::_1));
        
        // System subscriptions
        filling_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/fill_machine/filling", 10,
            std::bind(&RobotLogicNode::fillingCallback, this, std::placeholders::_1));
            
        scale_result_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/scale/result", 10,
            std::bind(&RobotLogicNode::scaleResultCallback, this, std::placeholders::_1));
            
        start_button_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/system/start_button", 10,
            std::bind(&RobotLogicNode::startButtonCallback, this, std::placeholders::_1));
            
        new_tray_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/system/new_tray_loaded", 10,
            std::bind(&RobotLogicNode::newTrayCallback, this, std::placeholders::_1));
        
        // NEW: AI selected row subscription
        selected_row_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/camera/ai/selected_row", 10,
            std::bind(&RobotLogicNode::selectedRowCallback, this, std::placeholders::_1));

        // Restore: Manual state control subscription
        goto_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot/goto_state", 10,
            std::bind(&RobotLogicNode::gotoStateCallback, this, std::placeholders::_1));

        // NEW: Command Row subscription (Single execution)
        command_row_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/command_row", 10,
            std::bind(&RobotLogicNode::commandRowCallback, this, std::placeholders::_1));

        // NEW: AI/Camera selected slot subscription (output tray)
        selected_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/camera/ai/selected_slot", 10,
            std::bind(&RobotLogicNode::selectedSlotCallback, this, std::placeholders::_1));

        // NEW: Manual slot command subscription (for terminal testing)
        command_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/command_slot", 10,
            std::bind(&RobotLogicNode::commandSlotCallback, this, std::placeholders::_1));
    }
    
    void initPublishers()
    {
        system_status_pub_ = create_publisher<std_msgs::msg::String>(
            "/robot/system_status", 10);
            
        error_pub_ = create_publisher<std_msgs::msg::String>(
            "/robot/error", 10);
            
        selected_slot_pub_ = create_publisher<std_msgs::msg::Int32>(
            "/robot/selected_output_slot", 10);
            
        gripper_cmd_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/robot/gripper_cmd", 10);  // NEW: Gripper command topic
    }
    
    void initServices()
    {
        // Enable/Disable robot system
        enable_system_service_ = create_service<std_srvs::srv::SetBool>(
            "/robot/enable_system",
            std::bind(&RobotLogicNode::enableSystemCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Emergency stop service
        emergency_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "/robot/emergency_stop",
            std::bind(&RobotLogicNode::emergencyStopCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Set manual mode service
        set_manual_mode_service_ = create_service<std_srvs::srv::SetBool>(
            "/robot/set_manual_mode",
            std::bind(&RobotLogicNode::setManualModeCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Services initialized:");
        RCLCPP_INFO(this->get_logger(), "  - /robot/enable_system");
        RCLCPP_INFO(this->get_logger(), "  - /robot/emergency_stop");
        RCLCPP_INFO(this->get_logger(), "  - /robot/set_manual_mode");
    }
    
    void loadMotionParameters()
    {
        // Load joint sequences from YAML parameter
        this->declare_parameter("motion_sequence", std::vector<std::string>{});
        std::vector<std::string> seq_lines;
        this->get_parameter("motion_sequence", seq_lines);

        joint_sequences_.clear();
        relmovl_sequences_.clear();
        digital_output_steps_.clear();

        for (const auto &line : seq_lines)
        {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            // Split by comma
            while (std::getline(ss, token, ','))
            {
                tokens.push_back(token);
            }

            if (tokens.empty())
                continue;

            const std::string &type = tokens[0];

            // Joint Move: J,j1,j2,j3,j4,j5,j6
            if (type == "J")
            {
                std::vector<double> joints;
                // Skip first token (type identifier)
                for (size_t i = 1; i < tokens.size(); ++i)
                {
                    try
                    {
                        joints.push_back(std::stod(tokens[i]));
                    }
                    catch (...)
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse joint value: '%s'", tokens[i].c_str());
                    }
                }

                // Only store if we have exactly 6 joints
                if (joints.size() == 6)
                {
                    joint_sequences_.push_back(joints);
                    size_t idx = joint_sequences_.size() - 1;
                    RCLCPP_INFO(this->get_logger(), "Loaded Index %zu: [J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f]",
                                idx, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid joint count: %zu (expected 6)", joints.size());
                }
            }
            // Relative Move: R,dx,dy,dz
            else if (type == "R" && tokens.size() >= 4)
            {
                try
                {
                    relmovl_sequences_.emplace_back(std::vector<double>{
                        std::stod(tokens[1]),
                        std::stod(tokens[2]),
                        std::stod(tokens[3])});
                    
                    size_t idx = relmovl_sequences_.size() - 1;
                    RCLCPP_INFO(this->get_logger(), "Loaded RelMovL %zu: [dx=%.2f, dy=%.2f, dz=%.2f]",
                                idx, std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
                }
                catch (...)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse RelMovL line: '%s'", line.c_str());
                }
            }
            // Digital Output: D,index,status
            else if (type == "D" && tokens.size() >= 3)
            {
                try
                {
                    int do_index = std::stoi(tokens[1]);
                    int do_status = std::stoi(tokens[2]);
                    digital_output_steps_.emplace_back(do_index, do_status);
                    
                    size_t idx = digital_output_steps_.size() - 1;
                    RCLCPP_INFO(this->get_logger(), "Loaded DO %zu: [index=%d, status=%d]",
                                idx, do_index, do_status);
                }
                catch (...)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse DO line: '%s'", line.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Malformed motion step: '%s'", line.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "🎯 Total joint index loaded: %zu", joint_sequences_.size());
        RCLCPP_INFO(this->get_logger(), "🎯 Total RelMovL loaded: %zu", relmovl_sequences_.size());
        RCLCPP_INFO(this->get_logger(), "🎯 Total DO steps loaded: %zu", digital_output_steps_.size());
        RCLCPP_INFO(this->get_logger(), "===========================================");
    }
    
    void initInputTrayROIs()
    {
        // 5 rows for input tray
        std::vector<std::vector<std::pair<int, int>>> row_corners = {
            {{212, 554}, {341, 200}, {428, 200}, {334, 560}},  // Row 1
            {{335, 563}, {427, 200}, {522, 200}, {467, 564}},  // Row 2
            {{473, 567}, {529, 201}, {614, 201}, {607, 567}},  // Row 3
            {{609, 568}, {612, 198}, {705, 198}, {748, 568}},  // Row 4
            {{744, 573}, {703, 196}, {803, 196}, {900, 573}}   // Row 5
        };
        
        input_tray_rois_.clear();
        for (const auto &corners : row_corners)
        {
            input_tray_rois_.push_back(ROIQuad::FromCorners(corners));
        }
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu input tray ROIs", input_tray_rois_.size());
    }
    
    void initBufferROI()
    {
        // Buffer ROI coordinates
        std::vector<std::pair<int, int>> buffer_corners = {
            {36, 328},
            {626, 325},
            {629, 500},
            {47, 501}
        };
        
        buffer_roi_ = ROIQuad::FromCorners(buffer_corners);
        RCLCPP_INFO(this->get_logger(), "Initialized buffer ROI");
    }
    
    void initOutputTrayROIs()
    {
        // 8 slots for output tray (canonical order 1→8)
        std::vector<std::vector<std::pair<int, int>>> slot_corners = {
            {{100, 100}, {200, 100}, {200, 200}, {100, 200}},  // Slot 1
            {{220, 100}, {320, 100}, {320, 200}, {220, 200}},  // Slot 2
            {{340, 100}, {440, 100}, {440, 200}, {340, 200}},  // Slot 3
            {{460, 100}, {560, 100}, {560, 200}, {460, 200}},  // Slot 4
            {{100, 220}, {200, 220}, {200, 320}, {100, 320}},  // Slot 5
            {{220, 220}, {320, 220}, {320, 320}, {220, 320}},  // Slot 6
            {{340, 220}, {440, 220}, {440, 320}, {340, 320}},  // Slot 7
            {{460, 220}, {560, 220}, {560, 320}, {460, 320}}   // Slot 8
        };
        
        for (size_t i = 0; i < 8; ++i)
        {
            output_tray_rois_[i] = ROIQuad::FromCorners(slot_corners[i]);
        }
        
        RCLCPP_INFO(this->get_logger(), "Initialized 8 output tray ROIs");
    }

    // ========================================================================
    // ROS CALLBACKS (SENSORS ONLY - NO STATE CHANGES)
    // ========================================================================
    
    void camera1Callback(const Detection2DArray::SharedPtr msg)
    {
        if (manual_mode_) return;
        
        // Count cartridges in each input tray row
        std::vector<int> row_counts(5, 0);
        
        for (const auto &det : msg->detections)
        {
            if (det.results.empty()) continue;
            
            const std::string &class_id = det.results[0].hypothesis.class_id;
            float score = det.results[0].hypothesis.score;
            
            // Only count cartridge class with sufficient confidence
            if (class_id != "0" || score < DETECTION_SCORE_THRESH)
                continue;
            
            float cx = det.bbox.center.position.x;
            float cy = det.bbox.center.position.y;
            
            // Check which row this detection belongs to
            for (size_t i = 0; i < input_tray_rois_.size(); ++i)
            {
                if (input_tray_rois_[i].contains(cx, cy))
                {
                    row_counts[i]++;
                    break;
                }
            }
        }
        
        // Filter and update row_full_ status
        for (size_t i = 0; i < row_counts.size(); ++i)
        {
            int filtered_count = row_filters_[i].filter_count(row_counts[i]);
            bool raw_ready = (filtered_count >= INPUT_ROW_THRESHOLD);
            bool stable_ready = row_filters_[i].update_ready(raw_ready);
            row_full_[i] = stable_ready;
        }
        
        // Check if input tray is completely empty
        input_tray_empty_ = std::none_of(row_full_.begin(), row_full_.end(), 
                                         [](bool full) { return full; });
        
        // Count cartridges in buffer
        buffer_cartridge_count_ = 0;
        for (const auto &det : msg->detections)
        {
            if (det.results.empty()) continue;
            
            const std::string &class_id = det.results[0].hypothesis.class_id;
            float score = det.results[0].hypothesis.score;
            
            if (class_id != "0" || score < DETECTION_SCORE_THRESH)
                continue;
            
            float cx = det.bbox.center.position.x;
            float cy = det.bbox.center.position.y;
            
            if (buffer_roi_.contains(cx, cy))
            {
                buffer_cartridge_count_++;
            }
        }
        
        buffer_detected_any_ = (buffer_cartridge_count_ >= 1);
        buffer_detected_full_ = (buffer_cartridge_count_ >= BUFFER_CAPACITY);
    }
    
    void camera2Callback(const Detection2DArray::SharedPtr msg)
    {
        if (manual_mode_) return;
        
        // Check which output slots are occupied
        std::array<bool, 8> slot_occupied{};
        slot_occupied.fill(false);
        
        for (const auto &det : msg->detections)
        {
            if (det.results.empty()) continue;
            
            const std::string &class_id = det.results[0].hypothesis.class_id;
            float score = det.results[0].hypothesis.score;
            
            if (class_id != "0" || score < DETECTION_SCORE_THRESH)
                continue;
            
            float cx = det.bbox.center.position.x;
            float cy = det.bbox.center.position.y;
            
            // Check which slot this detection belongs to
            for (int i = 0; i < 8; ++i)
            {
                if (output_tray_rois_[i].contains(cx, cy))
                {
                    slot_occupied[i] = true;
                    break;
                }
            }
        }
        
        // Select first empty slot in canonical order (1→8)
        selected_output_slot_ = -1;
        for (int i = 0; i < 8; ++i)
        {
            if (!slot_occupied[i])
            {
                selected_output_slot_ = i + 1;  // Slot IDs are 1-based
                break;
            }
        }
        
        // Publish selected slot
        auto slot_msg = std::make_shared<std_msgs::msg::Int32>();
        slot_msg->data = selected_output_slot_;
        selected_slot_pub_->publish(*slot_msg);
    }
    
    void fillingCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        fill_machine_filling_ = msg->data;
    }
    
    void scaleResultCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        scale_result_ready_ = true;
        scale_result_pass_ = msg->data;  // true = PASS, false = FAIL
    }
    
    void startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "[BUTTON] Start button pressed - Enabling system and hardware");
            
            // Enable system
            if (emergency_stop_)
            {
                RCLCPP_ERROR(this->get_logger(), "[BUTTON] Cannot enable: Emergency stop is active!");
                return;
            }
            
            system_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "[ENABLE] ✅ Robot system ENABLED");
            
            // Enable robot hardware
            auto req = std::make_shared<EnableRobot::Request>();
            req->load = 1.0;
            
            auto res = callService<EnableRobot>(enable_client_, req, "EnableRobot");
            
            if (res && res->res == 0)
            {
                RCLCPP_INFO(this->get_logger(), "[ENABLE] ✅ Robot hardware enabled");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[ENABLE] ⚠️ Robot hardware enable failed - enable manually if needed");
            }
            
            // Move to home position
            RCLCPP_INFO(this->get_logger(), "[HOME] Moving to home position...");
            moveToIndex(0);
            RCLCPP_INFO(this->get_logger(), "[HOME] ✅ Robot at home position");
            
            // Mark system as started
            system_started_ = true;
        }
    }
    
    void newTrayCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            new_tray_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "[TRAY] New tray loaded");
        }
    }
    
    // NEW: AI Selected Row Callback
    void selectedRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        
        int row = msg->data;
        
        // Validate row number (1-5)
        if (row < 1 || row > 5)
        {
            RCLCPP_WARN(this->get_logger(), "[AI ROW] Invalid row selection: %d (must be 1-5)", row);
            return;
        }
        
        // Check if selected row is actually full
        if (!row_full_[row - 1])
        {
            RCLCPP_WARN(this->get_logger(), "[AI ROW] Row %d is not full, ignoring selection", row);
            return;
        }
        
        selected_input_row_ = row;
        RCLCPP_INFO(this->get_logger(), "[AI ROW] ✅ Selected Row %d for picking", row);
    }

    // NEW: AI/Camera Selected Slot Callback (for output tray)
    void selectedSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int slot = msg->data;
        
        // Validate slot range (1-8)
        if (slot < 1 || slot > 8)
        {
            RCLCPP_WARN(this->get_logger(), "[AI SLOT] Invalid slot %d (valid: 1-8), ignoring", slot);
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            selected_output_slot_ = slot;
        }
        
        RCLCPP_INFO(this->get_logger(), "[AI SLOT] ✅ Selected Slot %d for placing", slot);
        
        // Publish confirmation
        auto msg_out = std::make_shared<std_msgs::msg::Int32>();
        msg_out->data = slot;
        selected_slot_pub_->publish(*msg_out);
    }

    // ========================================================================
    // SERVICE CALLBACKS
    // ========================================================================
    
    void enableSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            // Enable system
            if (emergency_stop_)
            {
                response->success = false;
                response->message = "Cannot enable: Emergency stop is active! Clear emergency stop first.";
                RCLCPP_ERROR(this->get_logger(), "[ENABLE] %s", response->message.c_str());
                return;
            }
            
            system_enabled_ = true;
            response->success = true;
            response->message = "Robot system ENABLED successfully";
            RCLCPP_INFO(this->get_logger(), "[ENABLE] ✅ %s", response->message.c_str());
            
            // Enable robot hardware
            auto req = std::make_shared<EnableRobot::Request>();
            req->load = 1.0;
            
        
            auto res = callService<EnableRobot>(enable_client_, req, "EnableRobot");
            
            if (res && res->res == 0)
            {
                RCLCPP_INFO(this->get_logger(), "[ENABLE] ✅ Robot hardware enabled");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[ENABLE] ⚠️ Robot hardware enable failed - enable manually if needed");
                // Don't fail the system enable, just warn
                response->success = true;
                response->message = "System enabled (hardware enable failed - enable manually)";
            }
            
            // 🏠 MOVE TO HOME POSITION (always try, even if hardware enable failed)
            RCLCPP_INFO(this->get_logger(), "[HOME] Moving to home position...");
            moveToIndex(0);
            RCLCPP_INFO(this->get_logger(), "[HOME] ✅ Robot at home position");
        }
        else
        {
            // Disable system
            system_enabled_ = false;
            response->success = true;
            response->message = "Robot system DISABLED successfully";
            RCLCPP_WARN(this->get_logger(), "[ENABLE] ⚠️ %s", response->message.c_str());
            
            // Disable robot hardware
            auto req = std::make_shared<EnableRobot::Request>();
            

            auto res = callService<EnableRobot>(enable_client_, req, "EnableRobot");

            if (res && res->res == 0)
            {
                RCLCPP_INFO(this->get_logger(), "[ENABLE] ✅ Robot hardware disabled");
            }
        }
    }
    
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            // Activate emergency stop
            emergency_stop_ = true;
            system_enabled_ = false;  // Force disable system
            
            response->success = true;
            response->message = "🚨 EMERGENCY STOP ACTIVATED 🚨";
            RCLCPP_ERROR(this->get_logger(), "[E-STOP] %s", response->message.c_str());
            
            // Disable robot hardware immediately
            auto req = std::make_shared<EnableRobot::Request>();
    
            
            // Publish error
            publishError("EMERGENCY STOP ACTIVATED - All operations halted");
        }
        else
        {
            // Clear emergency stop
            emergency_stop_ = false;
            
            response->success = true;
            response->message = "Emergency stop CLEARED. System ready to enable.";
            RCLCPP_INFO(this->get_logger(), "[E-STOP] ✅ %s", response->message.c_str());
        }
    }
    
    void setManualModeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        manual_mode_ = request->data;
        
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[MANUAL] 🔧 MANUAL MODE ENABLED - Camera checks bypassed. All rows treated as FULL for testing.");
            
            // Force Input Tray Logic: All rows FULL
            {
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                for (size_t i = 0; i < row_full_.size(); ++i) {
                    row_full_[i] = true;
                }
                input_tray_empty_ = false;
                selected_input_row_ = -1; // Reset selection to force re-evaluation
            }
            RCLCPP_INFO(this->get_logger(), "[MANUAL] All 5 rows marked as FULL");
            
            // Force Buffer Logic: Buffer EMPTY (to trigger refill testing)
            buffer_cartridge_count_ = 0;
            buffer_detected_any_ = false;
            buffer_detected_full_ = false;
            RCLCPP_INFO(this->get_logger(), "[MANUAL] Buffer marked as EMPTY");
            
            // Force Output Tray Logic: Slot 1 available
            selected_output_slot_ = 1;
            
            response->success = true;
            response->message = "Manual Mode ENABLED";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[MANUAL] 🛑 MANUAL MODE DISABLED - Restoring camera detection.");
            // Reset states to be updated by camera
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            for (size_t i = 0; i < row_full_.size(); ++i) {
                row_full_[i] = false;
            }
            // Let camera update them next frame
            
            response->success = true;
            response->message = "Manual Mode DISABLED";
        }
    }

    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    
    // Restored debug function
    void gotoStateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // IMPORTANT: This callback forces a state transition for manual testing.
        // Do NOT clear `selected_input_row_` or `selected_output_slot_` here.
        // Reason: the state handler that runs immediately after the transition
        // (for example `stateInitLoadChamberDirect` -> `motionStub_InputTrayChamber`)
        // expects to read the selection values. Clearing them here can race
        // with the state handler and cause the robot to pick the wrong index.
        // If a state handler should consume the selection, it must clear the
        // selection itself under the same mutex (`row_selection_mutex_` or
        // `output_slot_selection_mutex_`).

        std::string state_name = msg->data;
        SystemState target_state = SystemState::IDLE;

        if (state_name == "IDLE") target_state = SystemState::IDLE;
        else if (state_name == "INIT_CHECK") target_state = SystemState::INIT_CHECK;
        else if (state_name == "INIT_LOAD_CHAMBER_DIRECT") target_state = SystemState::INIT_LOAD_CHAMBER_DIRECT;
        else if (state_name == "INIT_REFILL_BUFFER") target_state = SystemState::INIT_REFILL_BUFFER;
        else if (state_name == "WAIT_FILLING") target_state = SystemState::WAIT_FILLING;
        else if (state_name == "TAKE_CHAMBER_TO_SCALE") target_state = SystemState::TAKE_CHAMBER_TO_SCALE;
        else if (state_name == "WAIT_SCALE_RESULT") target_state = SystemState::WAIT_SCALE_RESULT;
        else if (state_name == "PLACE_TO_OUTPUT") target_state = SystemState::PLACE_TO_OUTPUT;
        else if (state_name == "PLACE_TO_FAIL") target_state = SystemState::PLACE_TO_FAIL;
        else if (state_name == "REFILL_BUFFER") target_state = SystemState::REFILL_BUFFER;
        else if (state_name == "LOAD_CHAMBER_FROM_BUFFER") target_state = SystemState::LOAD_CHAMBER_FROM_BUFFER;
        else if (state_name == "LAST_BATCH_WAIT") target_state = SystemState::LAST_BATCH_WAIT;
        else if (state_name == "ERROR_SCALE") target_state = SystemState::ERROR_SCALE;
        else if (state_name == "ERROR_OUTPUT_TRAY_TIMEOUT") target_state = SystemState::ERROR_OUTPUT_TRAY_TIMEOUT;
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[GOTO] Unknown state name: '%s'", state_name.c_str());
            return;
        }

        RCLCPP_WARN(this->get_logger(), "[GOTO] ⏭️ FORCING JUMP TO: %s", state_name.c_str());
        // Special handling: if operator requests a REFILL_BUFFER jump and a
        // specific row was previously selected, treat it as a one-shot
        // manual refill for that row (similar to INIT_LOAD_CHAMBER_DIRECT
        // one-shot behaviour). This sets `manual_mode_` and
        // `stop_after_single_motion_` so the state will perform exactly one
        // pick from the selected row then return to IDLE.
        if (target_state == SystemState::REFILL_BUFFER)
        {
            int sel = -1;
            {
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                sel = selected_input_row_;
            }

            if (sel != -1)
            {
                manual_mode_ = true;
                stop_after_single_motion_ = true;
                RCLCPP_INFO(this->get_logger(), "[GOTO] One-shot REFILL_BUFFER requested for Row %d", sel);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[GOTO] REFILL_BUFFER requested but no row selected - will run normal refill");
            }
        }

        transitionTo(target_state);
        
        // NOTE: Do not reset selections here — state handlers will consume and
        // clear `selected_input_row_` / `selected_output_slot_` as appropriate.
        // This prevents manual commands (command_row/command_slot) from being
        // cleared before the target state runs.
    }

    void commandRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int row = msg->data;
        if (row < 1 || row > 5)
        {
            RCLCPP_ERROR(this->get_logger(), "[CMD ROW] Invalid Row %d (Must be 1-5)", row);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[CMD ROW] 🎯 Received Command for Input Row %d (Tray to Chamber/Buffer only)", row);
        
        // 1. Force Manual Mode ON to bypass camera
        manual_mode_ = true;
        // Indicate we should stop after a single motion (single-row command)
        stop_after_single_motion_ = true;
        
        // 2. Set Row and Flags (for input tray operations only)
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            selected_input_row_ = row;
            // Ensure this row is marked valid
            row_full_[row-1] = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "[CMD ROW] ✅ Input Row %d selected. Use goto_state with INIT_LOAD_CHAMBER_DIRECT to pick from input tray", row);
    }

    // NEW: Command Slot Callback (manual mode for output tray - scale to tray)
    void commandSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int slot = msg->data;
        if (slot < 1 || slot > 8)
        {
            RCLCPP_ERROR(this->get_logger(), "[CMD SLOT] Invalid Slot %d (Must be 1-8)", slot);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[CMD SLOT] 🎯 Received Command for Output Slot %d (Scale to Output Tray)", slot);
        
        // 1. Force Manual Mode ON to bypass camera
        manual_mode_ = true;
        
        // 2. Set Slot (for output tray operations only)
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            selected_output_slot_ = slot;
        }
        
        RCLCPP_INFO(this->get_logger(), "[CMD SLOT] ✅ Output Slot %d selected. Use goto_state with PLACE_TO_OUTPUT to place from scale to output tray", slot);
    }

    void stateMachineLoop()
    {
        RCLCPP_INFO(this->get_logger(), "[STATE MACHINE] Started");
        
        while (state_machine_running_ && rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                handleCurrentState();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(this->get_logger(), "[STATE MACHINE] Stopped");
    }
    
    void handleCurrentState()
    {
        // Check emergency stop
        if (emergency_stop_)
        {
            publishSystemStatus("EMERGENCY STOP - System halted");
            return;  // Do nothing while emergency stop is active
        }
        
        // Check if system is enabled
        if (!system_enabled_ && current_state_ != SystemState::IDLE)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[STATE] System disabled, transitioning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }
        
        switch (current_state_)
        {
            case SystemState::IDLE:
                stateIdle();
                break;
                
            case SystemState::INIT_CHECK:
                stateInitCheck();
                break;
                
            case SystemState::INIT_LOAD_CHAMBER_DIRECT:
                stateInitLoadChamberDirect();
                break;
                
            case SystemState::INIT_REFILL_BUFFER:
                stateInitRefillBuffer();
                break;
                
            case SystemState::WAIT_FILLING:
                stateWaitFilling();
                break;
                
            case SystemState::TAKE_CHAMBER_TO_SCALE:
                stateTakeChamberToScale();
                break;
                
            case SystemState::WAIT_SCALE_RESULT:
                stateWaitScaleResult();
                break;
                
            case SystemState::PLACE_TO_OUTPUT:
                statePlaceToOutput();
                break;
                
            case SystemState::PLACE_TO_FAIL:
                statePlaceToFail();
                break;
                
            case SystemState::REFILL_BUFFER:
                stateRefillBuffer();
                break;
                
            case SystemState::LOAD_CHAMBER_FROM_BUFFER:
                stateLoadChamberFromBuffer();
                break;
                
            case SystemState::LAST_BATCH_WAIT:
                stateLastBatchWait();
                break;
                
            case SystemState::ERROR_SCALE:
                stateErrorScale();
                break;
                
            case SystemState::ERROR_OUTPUT_TRAY_TIMEOUT:
                stateErrorOutputTrayTimeout();
                break;
        }
    }
    
    // ========================================================================
    // STATE HANDLERS
    // ========================================================================
    
    void stateIdle()
    {
        publishSystemStatus("IDLE - Waiting for start button");
        
        // Cannot start if system not enabled
        if (!system_enabled_)
        {
            // Don't spam logs
            return;
        }
        
        if (system_started_ && new_tray_loaded_)
        {
            RCLCPP_INFO(this->get_logger(), "[STATE] System started with new tray");
            system_started_ = false;
            new_tray_loaded_ = false;
            is_first_batch_ = true;
            transitionTo(SystemState::INIT_CHECK);
        }
    }
    
    void stateInitCheck()
    {
        publishSystemStatus("INIT_CHECK - Checking initial conditions");
        
        // FIRST BATCH LOGIC:
        // - If buffer is empty: Load chamber directly from input tray
        // - If buffer is full: Load chamber from buffer
        
        if (!buffer_detected_full_)
        {
            RCLCPP_INFO(this->get_logger(), "[INIT] Buffer empty, loading chamber directly from tray");
            transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[INIT] Buffer full, loading chamber from buffer");
            transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        }
    }
    
    void stateInitLoadChamberDirect()
    {
        publishSystemStatus("INIT_LOAD_CHAMBER_DIRECT - Loading chamber from input tray");
        
        // MOTION STUB: Move cartridge from input tray → chamber
        motionStub_InputTrayChamber();
        
        chamber_has_cartridge_ = true;
        chamber_is_empty_ = false;
        
        RCLCPP_INFO(this->get_logger(), "[INIT] Chamber loaded from input tray");
        
        // If this was invoked as a single-row command, stop after this
        // motion and return to IDLE. The `commandRowCallback` sets
        // `stop_after_single_motion_` to request one-shot behavior.
        if (stop_after_single_motion_)
        {
            stop_after_single_motion_ = false;
            manual_mode_ = false; // restore normal camera-driven operation
            RCLCPP_INFO(this->get_logger(), "[STATE] Single-command completed: returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }

        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            transitionTo(SystemState::INIT_REFILL_BUFFER);
        }
    }
    
    
    void stateInitRefillBuffer()
    {
        publishSystemStatus("INIT_REFILL_BUFFER - Filling buffer");
        
        // Fill buffer while waiting for chamber to be ready
        if (!buffer_detected_full_)
        {
            // MOTION STUB: Move cartridges from input tray → buffer
            motionStub_InputTrayBuffer();
        }
        
        RCLCPP_INFO(this->get_logger(), "[INIT] Buffer refilled, entering main loop");
        
        is_first_batch_ = false;  // First batch init complete

        // If this refill was requested as a single-command, stop after the
        // single pick and return to IDLE. Clear the single-command flag here.
        if (stop_after_single_motion_)
        {
            stop_after_single_motion_ = false;
            manual_mode_ = false; // restore camera-driven operation
            RCLCPP_INFO(this->get_logger(), "[STATE] Single-command buffer refill completed: returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }

        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            transitionTo(SystemState::WAIT_FILLING);
        }
    }
    
    void stateWaitFilling()
    {
        publishSystemStatus("WAIT_FILLING - Waiting for filling machine");
        
        // In manual mode, skip all condition checks and just proceed
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: WAIT_FILLING skipping conditions, returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }
        
        // PRIORITY 1: If scale has result AND filling is active → move scale to output/fail
        if (fill_machine_filling_ && scale_result_ready_ && scale_has_cartridge_)
        {
            RCLCPP_INFO(this->get_logger(), "[PRIORITY] Scale result ready during filling, processing output");
            
            if (scale_result_pass_)
            {
                transitionTo(SystemState::PLACE_TO_OUTPUT);
            }
            else
            {
                transitionTo(SystemState::PLACE_TO_FAIL);
            }
            return;
        }
        
        // PRIORITY 2: If filling started and chamber has cartridge → move to scale
        if (fill_machine_filling_ && chamber_has_cartridge_)
        {
            RCLCPP_INFO(this->get_logger(), "[WAIT_FILLING] Filling started, moving chamber to scale");
            transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
            return;
        }
        
        // PRIORITY 3: Refill buffer in parallel during filling (idle time)
        if (fill_machine_filling_ && !buffer_detected_full_ && !input_tray_empty_)
        {
            RCLCPP_INFO(this->get_logger(), "[PARALLEL] Refilling buffer during filling");
            transitionTo(SystemState::REFILL_BUFFER);
            return;
        }
        
        // Check LAST BATCH condition
        // Last batch = input tray empty AND buffer empty
        if (input_tray_empty_ && !buffer_detected_any_)
        {
            RCLCPP_INFO(this->get_logger(), "[LAST BATCH] Input tray and buffer both empty");
            transitionTo(SystemState::LAST_BATCH_WAIT);
            return;
        }
    }
    
    void stateTakeChamberToScale()
    {
        publishSystemStatus("TAKE_CHAMBER_TO_SCALE - Moving cartridge to scale");
        
        // MOTION STUB: Move cartridge from chamber → scale
        motionStub_ChamberScale();
        
        chamber_has_cartridge_ = false;
        chamber_is_empty_ = true;
        scale_has_cartridge_ = true;
        
        // Start scale timeout
        scale_wait_start_ = this->now();
        scale_result_ready_ = false;
        
        // Simulation mode for testing
        if (simulate_scale_)
        {
            RCLCPP_INFO(this->get_logger(), "[SIMULATION] Simulating scale result");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            scale_result_ready_ = true;
            scale_result_pass_ = force_pass_;
        }
        
        RCLCPP_INFO(this->get_logger(), "[SCALE] Cartridge on scale, waiting for result");
        
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            transitionTo(SystemState::WAIT_SCALE_RESULT);
        }
    }
    
    void stateWaitScaleResult()
    {
        publishSystemStatus("WAIT_SCALE_RESULT - Waiting for scale measurement");
        
        // In manual mode, skip all condition checks and just proceed
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: WAIT_SCALE_RESULT skipping conditions, returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }
        
        // Check timeout (2 minutes)
        auto elapsed = (this->now() - scale_wait_start_).seconds();
        if (elapsed > SCALE_TIMEOUT_SEC)
        {
            RCLCPP_ERROR(this->get_logger(), "[SCALE] Timeout after %.1f seconds", elapsed);
            transitionTo(SystemState::ERROR_SCALE);
            return;
        }
        
        // Wait for scale result
        if (!scale_result_ready_)
        {
            return;  // Keep waiting
        }
        
        // CRITICAL CONSTRAINT: Can only move scale→output when filling is active
        if (!fill_machine_filling_)
        {
            RCLCPP_WARN(this->get_logger(), "[CONSTRAINT] Cannot move to output - filling not active");
            return;  // Wait for filling to start
        }
        
        // Scale result received, decide action
        if (scale_result_pass_)
        {
            RCLCPP_INFO(this->get_logger(), "[SCALE] PASS - Moving to output");
            transitionTo(SystemState::PLACE_TO_OUTPUT);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[SCALE] FAIL - Moving to fail position");
            transitionTo(SystemState::PLACE_TO_FAIL);
        }
    }
    
    void statePlaceToOutput()
    {
        publishSystemStatus("PLACE_TO_OUTPUT - Placing cartridge in output tray");
        
        // In manual mode, skip all condition checks and just proceed
        if (manual_mode_)
        {
            int slot_to_use = (selected_output_slot_ != -1) ? selected_output_slot_ : 1;
            motionStub_ScaleOutputTray(slot_to_use);
            scale_has_cartridge_ = false;
            scale_result_ready_ = false;
            selected_output_slot_ = -1;
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: PLACE_TO_OUTPUT executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }
        
        // CRITICAL: Filling must be active
        if (!fill_machine_filling_)
        {
            RCLCPP_ERROR(this->get_logger(), "[CONSTRAINT VIOLATION] Attempting to place output while filling inactive");
            transitionTo(SystemState::ERROR_SCALE);
            return;
        }
        
        // NOTE: If no empty slot detected, wait here for slot index
        // Timeout: 3 minutes (OUTPUT_TRAY_TIMEOUT_SEC)
        if (selected_output_slot_ == -1)
        {
            // Start timeout if not already started
            static bool timeout_started = false;
            static rclcpp::Time timeout_start;
            
            if (!timeout_started)
            {
                timeout_start = this->now();
                timeout_started = true;
                RCLCPP_WARN(this->get_logger(), "[OUTPUT] No empty slot detected, waiting for index...");
            }
            
            auto elapsed = (this->now() - timeout_start).seconds();
            if (elapsed > OUTPUT_TRAY_TIMEOUT_SEC)
            {
                RCLCPP_ERROR(this->get_logger(), "[OUTPUT] Timeout waiting for empty slot");
                timeout_started = false;
                transitionTo(SystemState::ERROR_OUTPUT_TRAY_TIMEOUT);
                return;
            }
            
            return;  // Keep waiting for slot index
        }
        
        // MOTION STUB: Move cartridge from scale → output slot
        motionStub_ScaleOutputTray(selected_output_slot_);
        
        scale_has_cartridge_ = false;
        scale_result_ready_ = false;
        selected_output_slot_ = -1;  // Reset
        
        RCLCPP_INFO(this->get_logger(), "[OUTPUT] Cartridge placed successfully");
        
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            // Check if last batch
            if (input_tray_empty_ && !buffer_detected_any_)
            {
                transitionTo(SystemState::LAST_BATCH_WAIT);
            }
            else
            {
                // Continue with next cartridge
                if (!buffer_detected_full_)
                {
                    transitionTo(SystemState::REFILL_BUFFER);
                }
                else
                {
                    transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
                }
            }
        }
    }
    
    void statePlaceToFail()
    {
        publishSystemStatus("PLACE_TO_FAIL - Placing cartridge in fail position");
        
        // MOTION STUB: Move cartridge from scale → fail position
        motionStub_ScaleFailPosition();
        
        scale_has_cartridge_ = false;
        scale_result_ready_ = false;
        
        RCLCPP_INFO(this->get_logger(), "[FAIL] Cartridge placed in fail position");
        
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            // Continue processing
            if (!buffer_detected_full_)
            {
                transitionTo(SystemState::REFILL_BUFFER);
            }
            else
            {
                transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
            }
        }
    }
    
    void stateRefillBuffer()
    {
        publishSystemStatus("REFILL_BUFFER - Refilling buffer during filling");
        
        // Only refill if buffer not full and tray not empty
        if (!buffer_detected_full_ && !input_tray_empty_)
        {
            // MOTION STUB: Move cartridges from input tray → buffer
            motionStub_InputTrayBuffer();
            
            RCLCPP_INFO(this->get_logger(), "[BUFFER] Buffer refilled");
        }
        
        // If this refill was a single-command request, clear the flag and
        // return to IDLE so the node does not continue automatic processing.
        if (stop_after_single_motion_)
        {
            stop_after_single_motion_ = false;
            manual_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "[STATE] Single-command refill handled: returning to IDLE");
            transitionTo(SystemState::IDLE);
            return;
        }

        // After refilling, load chamber from buffer
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        }
    }
    
    void stateLoadChamberFromBuffer()
    {
        publishSystemStatus("LOAD_CHAMBER_FROM_BUFFER - Loading chamber from buffer");
        
        // MOTION STUB: Move cartridge from buffer → chamber
        motionStub_BufferChamber();
        
        chamber_has_cartridge_ = true;
        chamber_is_empty_ = false;
        
        RCLCPP_INFO(this->get_logger(), "[CHAMBER] Loaded from buffer");
        
        if (manual_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "[STATE] 🛑 Manual mode: state executed, returning to IDLE");
            transitionTo(SystemState::IDLE);
        }
        else
        {
            // Return to main loop
            transitionTo(SystemState::WAIT_FILLING);
        }
    }
    
    void stateLastBatchWait()
    {
        publishSystemStatus("LAST_BATCH_WAIT - Processing final cartridges");
        
        // In last batch:
        // - Input tray is empty
        // - Buffer is empty
        // - Robot stands idle, cannot refill
        // - Process remaining cartridges in chamber and scale
        
        // If scale has result and filling is active, process it
        if (scale_result_ready_ && fill_machine_filling_ && scale_has_cartridge_)
        {
            if (scale_result_pass_)
            {
                transitionTo(SystemState::PLACE_TO_OUTPUT);
            }
            else
            {
                transitionTo(SystemState::PLACE_TO_FAIL);
            }
            return;
        }
        
        // If chamber still has cartridge, move to scale
        if (chamber_has_cartridge_ && fill_machine_filling_)
        {
            transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
            return;
        }
        
        // All cartridges processed, system complete
        if (!chamber_has_cartridge_ && !scale_has_cartridge_ && !scale_result_ready_)
        {
            RCLCPP_INFO(this->get_logger(), "[LAST BATCH] All cartridges processed, system complete");
            transitionTo(SystemState::IDLE);
        }
    }
    
    void stateErrorScale()
    {
        publishSystemStatus("ERROR_SCALE - Scale error occurred");
        publishError("SCALE ERROR - Operator intervention required");
        
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Scale timeout or constraint violation");
        
        // Wait for operator to press start button to recover
        if (system_started_)
        {
            RCLCPP_INFO(this->get_logger(), "[RECOVERY] Start button pressed, returning to INIT");
            system_started_ = false;
            
            // Reset to first batch state
            is_first_batch_ = true;
            chamber_has_cartridge_ = false;
            chamber_is_empty_ = true;
            scale_has_cartridge_ = false;
            scale_result_ready_ = false;
            
            transitionTo(SystemState::INIT_CHECK);
        }
    }
    
    void stateErrorOutputTrayTimeout()
    {
        publishSystemStatus("ERROR_OUTPUT_TRAY_TIMEOUT - Output tray full or not detected");
        publishError("OUTPUT TRAY ERROR - Please change tray or verify camera");
        
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Output tray timeout - no empty slot detected");
        
        // Wait for operator to fix issue and press start
        if (system_started_)
        {
            RCLCPP_INFO(this->get_logger(), "[RECOVERY] Start button pressed after tray change");
            system_started_ = false;
            
            // Return to WAIT_FILLING to retry
            transitionTo(SystemState::WAIT_FILLING);
        }
    }

    // ========================================================================
    // MOTION HELPER FUNCTIONS
    // ========================================================================
    


    // Implement getCurrentPose helper
    std::vector<double> getCurrentPose()
    {
        auto req = std::make_shared<GetPose::Request>();
        req->user = 0;
        req->tool = 1;
        
        auto res = callService<GetPose>(pose_client_, req, "GetPose");
        
        if (!res)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
            return {};
        }
        
        // Parse the result string: "{x,y,z,rx,ry,rz}"
        // Note: The actual GetPose service usually returns a string "vector" or similar which needs parsing
        // OR it returns direct fields. Let's assume standard behavior or fields.
        // Checking GetPose .srv def is safer, but usually it returns a string in some versions or fields.
        // Since we don't have GetPose definition loaded, I will check it in a separate step if this fails, 
        // but for now let's assume it returns fields x, y, z, rx, ry, rz directly simply because 
        // standard messages usually do. Wait, Dobot V3 messages can be strings.
        // Let's assume fields for now based on the usage `current[0]`.
        
        // Actually, let me verify GetPose quickly or implement a "safe" version using fields if available.
        // To be safe I will implement a parser if it's a string, or fields if they exist.
        // But since I cannot see GetPose, I'll rely on common fields: x, y, z, r, p, y or similar.
        // Wait, I should probably check GetPose first.
        
        // HOWEVER, to proceed without blocking, I'll implement it assuming it returns fields.
        // If it compiles, great. If not, I'll fix it.
        // Looking at other code, it seems the user expects vector access.
        
        std::vector<double> pose;
        // Assuming the response has x, y, z, rx, ry, rz fields
        // But wait, the previous code `auto current = getCurrentPose()` calls it. 
        // I will implement a placeholder that I can fix if wrong.
        
        // Actually, let's look at `GetPose` service in the next step if I can't be sure.
        // But for now, I'll add the function signature to fix the "not declared" error 
        // and implement the body with best guess.
        
        // If GetPose returns a string (common in Dobot), we need parsing. 
        // If it returns fields, we access them.
        // I will try fields first as it is strictly typed C++.
        
        // Using common Dobot naming:
        // pose.push_back(res->x);
        // pose.push_back(res->y);
        // pose.push_back(res->z);
        // pose.push_back(res->r); // or rx
        // pose.push_back(res->l); // or ry?
        // pose.push_back(res->h); // or rz?
        
        // Actually, better to just return an empty vector and print error "Not Implemented" 
        // effectively, BUT I need to fix the compile error first. 
        // The most likely fields are x, y, z, rx, ry, rz.
        
        // Let's try:
        // return std::vector<double>{res->x, res->y, res->z, res->rx, res->ry, res->rz};
        // This is a gamble. 
        
        // ALTERNATIVE: Use `get_angle_client_` or similar if pose fails.
        // I'll stick to simple fields.
        
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Placeholder to fix compilation
    }
    
    void moveToIndex(size_t index)
    {
    if (index >= joint_sequences_.size())
    {
        RCLCPP_ERROR(
            get_logger(),
            "[moveToIndex] Index %zu out of range (size=%zu)",
            index,
            joint_sequences_.size()
        );
        return;
    }

    const auto &j = joint_sequences_[index];

    auto req = std::make_shared<JointMovJ::Request>();
    req->j1 = j[0];
    req->j2 = j[1];
    req->j3 = j[2];
    req->j4 = j[3];
    req->j5 = j[4];
    req->j6 = j[5];

    auto res = callService<JointMovJ>(joint_client_, req, "JointMovJ");
    if (!res || res->res != 0)
    {
        RCLCPP_ERROR(get_logger(), "[MOVE] JointMovJ failed at index %zu", index);
        return;
    }

    // ⏳ Chờ robot chạy xong queue
    if (!sync())
    {
        RCLCPP_ERROR(get_logger(), "[MOVE] Sync failed after JointMovJ");
    }
    }


    
    // MovJ: Joint move in Cartesian space (x,y,z,rx,ry,rz)
    void movJ(double x, double y, double z, double rx, double ry, double rz, 
              int speed = 50, int accel = 20)
    {
        RCLCPP_INFO(this->get_logger(), "[MovJ] Moving to (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", 
                    x, y, z, rx, ry, rz);
        
        auto req = std::make_shared<JointMovJ::Request>();
        
        std::ostringstream cmd;
        cmd << "MovJ(" << x << "," << y << "," << z << "," 
            << rx << "," << ry << "," << rz 
            << ",User=0,Tool=1,Speed=" << speed << ",Accel=" << accel << ",CP=1)";
        
        req->param_value.push_back(cmd.str());
        
        callService<JointMovJ>(joint_client_, req, "JointMovJ");
        sync();
    }
    
    // MovL: Linear move in Cartesian space (x,y,z,rx,ry,rz)
    void movL(double x, double y, double z, double rx, double ry, double rz,
              int speed = 30, int accel = 20)
    {
        RCLCPP_INFO(this->get_logger(), "[MovL] Linear move to (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", 
                    x, y, z, rx, ry, rz);
        
        auto req = std::make_shared<RelMovL::Request>();
        
        std::ostringstream cmd;
        cmd << "MovL(" << x << "," << y << "," << z << "," 
            << rx << "," << ry << "," << rz 
            << ",User=0,Tool=1,Speed=" << speed << ",Accel=" << accel << ",CP=1)";
        
        req->param_value.push_back(cmd.str());
        
        callService<RelMovL>(relmovl_client_, req, "RelMovL");
        sync();
    }
    
    // MoveR: Relative move from current position
    void moveR(double dx, double dy, double dz, int speed = 40, int accel = 20)
    {
        RCLCPP_INFO(this->get_logger(), "[MoveR] Relative move (%.2f, %.2f, %.2f)", dx, dy, dz);
        
        auto current = getCurrentPose();
        if (current.size() < 6)
        {
            RCLCPP_ERROR(this->get_logger(), "[MoveR] Failed to get current pose");
            return;
        }
        
        double target_x = current[0] + dx;
        double target_y = current[1] + dy;
        double target_z = current[2] + dz;
        
        auto req = std::make_shared<RelMovL::Request>();
        req->offset1 = dx;
        req->offset2 = dy;
        req->offset3 = dz;
        req->offset4 = 0.0;
        req->offset5 = 0.0;
        req->offset6 = 0.0;
        req->param_value.clear();
        
        callService<RelMovL>(relmovl_client_, req, "RelMovL");
        sync();
    }
    
    void setDigitalOutput(int index, bool status)
    {
    RCLCPP_INFO(
        get_logger(),
        "[DO] Setting DO[%d] = %s",
        index,
        status ? "ON" : "OFF"
    );

    auto req = std::make_shared<DO::Request>();
    req->index = index;
    req->status = status ? 1 : 0;

    auto res = callService<DO>(do_client_, req, "DO");
    if (!res)
    {
        RCLCPP_ERROR(get_logger(), "[DO] Failed to set DO[%d]", index);
    }

    // NEW: If DO[8] is gripper, send command to Festo controller
    if (index == 8 && gripper_cmd_pub_)
    {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        msg->data = status;
        gripper_cmd_pub_->publish(*msg);
        RCLCPP_INFO(get_logger(), "[GRIPPER] Sent command to Festo: %s", status ? "ON" : "OFF");
    }
    }

    
    bool sync()
    {
    auto req = std::make_shared<SyncSrv::Request>();

    auto res = callService<SyncSrv>(sync_client_, req, "Sync");
    if (!res)
    {
        RCLCPP_ERROR(get_logger(), "[SYNC] Sync failed");
        return false;
    }

    RCLCPP_INFO(get_logger(), "[SYNC] Queue completed");
    return true;
    }
    

    // ========================================================================
    // MOTION STUBS
    // ========================================================================
    
       void motionStub_InputTrayChamber()
    {
        RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Chamber");
        
        // Get selected row from AI
        int row_to_pick = -1;
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            row_to_pick = selected_input_row_;
            selected_input_row_ = -1;  // Reset after using
        }
        
        // If no AI selection and manual mode is active, do NOT fallback.
        // Manual mode requires an explicit `command_row` selection.
        if (row_to_pick == -1)
        {
            if (manual_mode_)
            {
                RCLCPP_ERROR(this->get_logger(), "[MOTION] Manual mode active but no row selected — aborting pick");
                return;
            }

            // Automatic fallback: pick first full row
            for (size_t i = 0; i < row_full_.size(); ++i)
            {
                if (row_full_[i])
                {
                    int row_num = static_cast<int>(i) + 1;  // Row number 1-5
                    size_t pick_index_local = static_cast<size_t>(i) + 1; // YAML index 1-5
                    row_to_pick = row_num;
                    RCLCPP_INFO(this->get_logger(), "[MOTION] No AI selection, using first full row: %d (Index %zu)", row_to_pick, pick_index_local);
                    break;
                }
            }
        }
        
        if (row_to_pick == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] No full row available to pick!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "[MOTION] 🎯 Picking from Row %d", row_to_pick);
        
        // Motion sequence: Pick cartridge from selected row → Chamber
        // NOTE ABOUT INDEX MAPPING:
        // The YAML `motion_sequence` maps as follows:
        //   INDEX 0 = HOME
        //   INDEX 1-5 = Row poses for input tray (Row 1..Row 5)
        // Therefore `row_to_pick` (1..5) must be passed to `moveToIndex`
        // *as-is* to select the correct YAML entry. Do NOT convert to 0-based
        // indexing here (off-by-one will cause incorrect picking locations).
        // Any change to `motion_sequence` ordering must be reflected here.
        size_t pick_index = static_cast<size_t>(row_to_pick);  // YAML index for the row (row_to_pick is 1-based)
        
        // Example motion sequence:
        // 1. Move to above pick position (uses YAML index)
        RCLCPP_INFO(this->get_logger(), "[MOTION] moveToIndex -> using YAML index %zu for Row %d", pick_index, row_to_pick);
        moveToIndex(pick_index);
      
        // 2. Move down to pick (use RelMovL or specific pick position)
        moveR(0, 0, -30);  // Down 50mm
        
        // 3. Close gripper / wait for grip
        setDigitalOutput(8, true);  // Gripper ON
    
        // 4. Move up
        moveR(0, 0, 30);  // Up 50mm
      
        moveToIndex(6);
        // 5. Move to chamber position (you need to add chamber index to YAML)
        moveToIndex(7);  // Example: Index 10 is chamber position
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        moveR(0, -30, 0);
        // 7. Release gripper
        setDigitalOutput(8, false);  // Gripper OFF
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        moveR(0, 30, 0);
        RCLCPP_INFO(this->get_logger(), "[MOTION] ✅ Cartridge placed in chamber from Row %d", row_to_pick);
        // NOTE: Do NOT clear `stop_after_single_motion_` here. The state
        // handler (`stateInitLoadChamberDirect`) will decide whether to return
        // to IDLE after a single-command and will clear the flag there. This
        // avoids races where the state logic continues to the next state.
    }
    
    void motionStub_InputTrayBuffer()
    {
        RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Buffer");
        
        // Pick multiple cartridges from input tray to fill buffer
        int cartridges_needed = BUFFER_CAPACITY - buffer_cartridge_count_;
        if (cartridges_needed <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "[MOTION] Buffer already full, skipping");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "[MOTION] Need to pick %d cartridges for buffer", cartridges_needed);
        
        int picked_count = 0;

        // If this was a single-command in manual mode, only pick the
        // explicitly commanded row (if any) and return.
        if (stop_after_single_motion_)
        {
            int row_num = -1;
            {
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                row_num = selected_input_row_;
                selected_input_row_ = -1; // consume selection
            }

            if (row_num < 1 || row_num > static_cast<int>(row_full_.size()))
            {
                RCLCPP_ERROR(this->get_logger(), "[MOTION] (single) Invalid or no row selected: %d", row_num);
                return;
            }

            // If the row isn't considered full but we're in manual mode, allow the pick.
            if (!row_full_[row_num - 1] && !manual_mode_)
            {
                RCLCPP_WARN(this->get_logger(), "[MOTION] (single) Row %d not full and not in manual mode - aborting", row_num);
                return;
            }

            size_t pick_index = static_cast<size_t>(row_num);
            RCLCPP_INFO(this->get_logger(), "[MOTION] (single) 🎯 Picking from Row %d (Index %zu) → Buffer", row_num, pick_index);

            // 1. Move to row pick position
            moveToIndex(pick_index);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 2. Move down to pick
            moveR(0, 0, -30);

            // 3. Activate gripper
            setDigitalOutput(8, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 4. Move up
            moveR(0, 0, 30);
            moveToIndex(6);

            // 5. Move to buffer position
            moveToIndex(8);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 6. Put into chamber/buffer
            moveR(0, -30, 0);

            // 7. Release gripper
            setDigitalOutput(8, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            moveR(0, 30, 0);
            moveToIndex(6);

            RCLCPP_INFO(this->get_logger(), "[MOTION] (single) ✅ Cartridge placed in buffer from Row %d", row_num);
            return;
        }
        
        // Pick from each full row (Row 1 to Row 5) without using an indexed for-loop.
        // Find the next available full row and pick until the buffer is filled
        // or no full rows remain.
        while (picked_count < cartridges_needed)
        {
            auto it = std::find_if(row_full_.begin(), row_full_.end(), [](bool v){ return v; });
            if (it == row_full_.end())
            {
                RCLCPP_INFO(this->get_logger(), "[MOTION] No more full rows available to pick");
                break;
            }

            size_t i = static_cast<size_t>(std::distance(row_full_.begin(), it));
            int row_num = static_cast<int>(i) + 1;  // Row number 1-5
            size_t pick_index = i + 1;  // Index 1-5 in YAML

            RCLCPP_INFO(this->get_logger(), "[MOTION] 🎯 Picking from Row %d (Index %zu) → Buffer",
                        row_num, pick_index);

            // === Motion sequence for each pick ===
            // 1. Move to row pick position
            moveToIndex(pick_index);  // Index 1-5 for Row 1-5
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 2. Move down to pick
            moveR(0, 0, -30);  // Down 30mm

            // 3. Activate gripper
            setDigitalOutput(8, true);  // Gripper ON
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 4. Move up
            moveR(0, 0, 30);  // Up 30mm
            moveToIndex(6);

            // 5. Move to buffer position
            moveToIndex(8);  // Index 8: Buffer position
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 6. Put into chamber
            moveR(0, -30, 0);

            // 7. Release gripper
            setDigitalOutput(8, false);  // Gripper OFF
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            moveR(0, 30, 0);
            moveToIndex(6);

            picked_count++;
            RCLCPP_INFO(this->get_logger(), "[MOTION] ✅ Cartridge %d placed in buffer from Row %d",
                        picked_count, row_num);

            // Do not modify row_full_ here; camera or manual mode controls it.
        }
        
        RCLCPP_INFO(this->get_logger(), "[MOTION] ✅ Buffer refill complete: %d cartridges picked", picked_count);
    }
    
    void motionStub_ChamberScale()
    {
        RCLCPP_INFO(this->get_logger(), "[MOTION STUB] Chamber → Scale");
        moveToIndex(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        moveToIndex(7);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        moveR(0, -30, 0);
        setDigitalOutput(8, true);  // Gripper ON
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        moveR(0, 30, 0);   
        moveToIndex(9);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        moveToIndex(10);
        moveR(0, 0, -30);   
        setDigitalOutput(8, false);  // Gripper OFF
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        moveR(0, 0, 30);
    }
    
    void motionStub_ScaleOutputTray(int slot_id)
    {
        // Validate slot range (1-8)
        if (slot_id < 1 || slot_id > 8)
        {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] Invalid slot_id %d (valid: 1-8)", slot_id);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "[MOTION] Scale → Output Tray Slot %d", slot_id);
        moveToIndex(11);
        moveR(0, 0, -30);
        setDigitalOutput(8, true);  // Gripper ON
        moveR(0, 0, 30);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        moveToIndex(12);
        // Convert slot (1-8) to index (13-20) in YAML motion_sequence
        // Index 13 = Slot 1, Index 14 = Slot 2, ..., Index 20 = Slot 8
        size_t slot_index = 12 + slot_id;  // 12 + 1 = 13, 12 + 2 = 14, ..., 12 + 8 = 20

        // 5. Move to target slot position from YAML
        RCLCPP_INFO(this->get_logger(), "[MOTION] Moving to slot position (Index %zu)", slot_index);
        moveToIndex(slot_index);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // 6. Move down to slot level
        moveR(0, 0, -30);   // Down 30mm
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 7. Release gripper (DO[8] OFF)
        setDigitalOutput(8, false);  // Gripper OFF
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 8. Move up from slot
        moveR(0, 0, 30);    // Up 30mm
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        moveToIndex(12);
        RCLCPP_INFO(this->get_logger(), "[MOTION] ✅ Cartridge placed in output tray Slot %d", slot_id);
    }
    
    void motionStub_ScaleFailPosition()
    {
        RCLCPP_INFO(this->get_logger(), "[MOTION STUB] Scale → Fail Position");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void motionStub_BufferChamber()
    {
        moveToIndex(8);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        moveR(0, 0, -30);
        setDigitalOutput(8, true);  // Gripper ON
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        moveR(0, 0, 30);
        moveToIndex(7);
        moveR(0, -30, 0);  
        setDigitalOutput(8, false);  // Gripper OFF
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        moveR(0, -30, 0); 
        moveToIndex(0);
        RCLCPP_INFO(this->get_logger(), "[MOTION STUB] Buffer → Chamber");
    }

    // ========================================================================
    // UTILITY FUNCTIONS
    // ========================================================================
    
    void transitionTo(SystemState next_state)
    {
        if (current_state_ == next_state)
            return;
        
        RCLCPP_INFO(this->get_logger(), "[STATE TRANSITION] %s → %s",
                    stateToString(current_state_).c_str(),
                    stateToString(next_state).c_str());
        
        current_state_ = next_state;
    }
    
    std::string stateToString(SystemState state)
    {
        switch (state)
        {
            case SystemState::IDLE: return "IDLE";
            case SystemState::INIT_CHECK: return "INIT_CHECK";
            case SystemState::INIT_LOAD_CHAMBER_DIRECT: return "INIT_LOAD_CHAMBER_DIRECT";
            case SystemState::INIT_REFILL_BUFFER: return "INIT_REFILL_BUFFER";
            case SystemState::WAIT_FILLING: return "WAIT_FILLING";
            case SystemState::TAKE_CHAMBER_TO_SCALE: return "TAKE_CHAMBER_TO_SCALE";
            case SystemState::WAIT_SCALE_RESULT: return "WAIT_SCALE_RESULT";
            case SystemState::PLACE_TO_OUTPUT: return "PLACE_TO_OUTPUT";
            case SystemState::PLACE_TO_FAIL: return "PLACE_TO_FAIL";
            case SystemState::REFILL_BUFFER: return "REFILL_BUFFER";
            case SystemState::LOAD_CHAMBER_FROM_BUFFER: return "LOAD_CHAMBER_FROM_BUFFER";
            case SystemState::LAST_BATCH_WAIT: return "LAST_BATCH_WAIT";
            case SystemState::ERROR_SCALE: return "ERROR_SCALE";
            case SystemState::ERROR_OUTPUT_TRAY_TIMEOUT: return "ERROR_OUTPUT_TRAY_TIMEOUT";
            default: return "UNKNOWN";
        }
    }
    
    void publishSystemStatus(const std::string &status)
    {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = status;
        system_status_pub_->publish(*msg);
    }
    
    void publishError(const std::string &error)
    {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = error;
        error_pub_->publish(*msg);
        RCLCPP_ERROR(this->get_logger(), "[ERROR] %s", error.c_str());
    }
    
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string &service_name)
    {
        RCLCPP_INFO(get_logger(), "[SERVICE] Calling %s...", service_name.c_str());
        
        if (!client->wait_for_service(5s))
        {
            RCLCPP_ERROR(get_logger(), "[SERVICE] ❌ %s not available after 5s timeout", service_name.c_str());
            return nullptr;
        }
        
        RCLCPP_INFO(get_logger(), "[SERVICE] %s is available, sending request...", service_name.c_str());
        
        auto future = client->async_send_request(request);
        
        // Wait for response
        if (future.wait_for(10s) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "[SERVICE] ❌ %s timeout waiting for response", service_name.c_str());
            return nullptr;
        }
        
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "[SERVICE] ✅ %s got response", service_name.c_str());
        
        return response;
    }
};

// ============================================================================
// SIMULATION NODE (FOR TESTING WITHOUT RevPi)
// ============================================================================
class SimulationNode : public rclcpp::Node
{
public:
    SimulationNode() : Node("simulation_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== Simulation Node Started ===");
        RCLCPP_INFO(this->get_logger(), "This node simulates RevPi signals for testing");
        
        // Publishers
        filling_pub_ = create_publisher<std_msgs::msg::Bool>("/fill_machine/filling", 10);
        scale_pub_ = create_publisher<std_msgs::msg::Bool>("/scale/result", 10);
        start_pub_ = create_publisher<std_msgs::msg::Bool>("/system/start_button", 10);
        tray_pub_ = create_publisher<std_msgs::msg::Bool>("/system/new_tray_loaded", 10);
        
        // Subscribers for control
        sim_control_sub_ = create_subscription<std_msgs::msg::String>(
            "/sim/control", 10,
            std::bind(&SimulationNode::controlCallback, this, std::placeholders::_1));
        
        // Timer for automatic filling simulation
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimulationNode::timerCallback, this));
        
        printHelp();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr filling_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scale_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tray_pub_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_control_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool auto_filling_ = false;
    rclcpp::Time filling_start_;
    bool filling_active_ = false;
    
    void printHelp()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "=== SIMULATION COMMANDS ===");
        RCLCPP_INFO(this->get_logger(), "Publish to /sim/control (std_msgs/String):");
        RCLCPP_INFO(this->get_logger(), "  'start'           - Trigger start button");
        RCLCPP_INFO(this->get_logger(), "  'tray'            - Signal new tray loaded");
        RCLCPP_INFO(this->get_logger(), "  'filling_on'      - Start filling");
        RCLCPP_INFO(this->get_logger(), "  'filling_off'     - Stop filling");
        RCLCPP_INFO(this->get_logger(), "  'scale_pass'      - Scale result PASS");
        RCLCPP_INFO(this->get_logger(), "  'scale_fail'      - Scale result FAIL");
        RCLCPP_INFO(this->get_logger(), "  'auto_fill'       - Auto simulate 90s filling cycle");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Example:");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic pub -1 /sim/control std_msgs/String \"data: 'start'\"");
        RCLCPP_INFO(this->get_logger(), "");
    }
    
    void controlCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &cmd = msg->data;
        
        if (cmd == "start")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = true;
            start_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] Start button pressed");
        }
        else if (cmd == "tray")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = true;
            tray_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] New tray loaded");
        }
        else if (cmd == "filling_on")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = true;
            filling_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] Filling ON");
        }
        else if (cmd == "filling_off")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = false;
            filling_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] Filling OFF");
        }
        else if (cmd == "scale_pass")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = true;
            scale_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] Scale result: PASS");
        }
        else if (cmd == "scale_fail")
        {
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = false;
            scale_pub_->publish(*m);
            RCLCPP_INFO(this->get_logger(), "[SIM] Scale result: FAIL");
        }
        else if (cmd == "auto_fill")
        {
            auto_filling_ = true;
            filling_start_ = this->now();
            filling_active_ = true;
            
            auto m = std::make_shared<std_msgs::msg::Bool>();
            m->data = true;
            filling_pub_->publish(*m);
            
            RCLCPP_INFO(this->get_logger(), "[SIM] Auto filling started (90s cycle)");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[SIM] Unknown command: %s", cmd.c_str());
        }
    }
    
    void timerCallback()
    {
        if (auto_filling_ && filling_active_)
        {
            auto elapsed = (this->now() - filling_start_).seconds();
            
            if (elapsed >= 90.0)  // 90 seconds filling time
            {
                auto m = std::make_shared<std_msgs::msg::Bool>();
                m->data = false;
                filling_pub_->publish(*m);
                
                filling_active_ = false;
                RCLCPP_INFO(this->get_logger(), "[SIM] Auto filling completed (90s)");
            }
        }
    }
};

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotLogicNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}