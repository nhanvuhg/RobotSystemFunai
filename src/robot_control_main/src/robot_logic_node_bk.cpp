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

#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>

// using MoveJ = dobot_msgs_v3::srv::JointMovJ;

using namespace std::chrono_literals;

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
        read_period_ms_ = this->declare_parameter<int>("read_period_ms", 100);

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

        yolov8_sub_cam1_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/cam1/detections_output", 10,
            std::bind(&RobotLogicNode::yolov8CallbackCam1, this, std::placeholders::_1));

        yolov8_sub_cam2_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/cam2/detections_output", 10,
            std::bind(&RobotLogicNode::yolov8CallbackCam2, this, std::placeholders::_1));

        yolov8_sub_camrealsense_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/camrealsense/detections_output", 10,
            std::bind(&RobotLogicNode::yolov8CallbackCamRealsense, this, std::placeholders::_1));

        empty_slot_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/empty_slots", rclcpp::QoS(10));

        signal_take_cartridge_to_fill_machine_ = this->create_subscription<std_msgs::msg::Bool>(
            "/take_cartridge", 10,
            std::bind(&RobotLogicNode::takeCartridgeCallback, this, std::placeholders::_1));

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
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolov8_sub_camrealsense_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr empty_slot_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_take_cartridge_to_fill_machine_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double>> joint_sequences_;
    std::vector<std::vector<double>> relmovl_sequences_;
    std::vector<std::pair<int, int>> digital_output_steps_;

    std::string plc_ip_read_, plc_ip_write_;
    int db_number_{10};
    int read_offset_{0};
    int read_size_{40};
    int write_offset_{0};
    int read_period_ms_{1000};

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

    struct DetectionTPlus
    {
        int count_Tplus_cam1 = 0;
        int count_Tplus_cam2 = 0;
        bool ready() const
        {
            return count_Tplus_cam1 == 4 && count_Tplus_cam2 == 4;
        }
    } detection_Tplus_;

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
    bool TPlush_cartridge_fail;
    int count_check_cartridge_empty_tray = 0;

    using ValueT = typename decltype(PLCResponse().values)::value_type;

    std::vector<ValueT> last_values_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool new_resp_{false};

    int cmdCheckTakeCartridge;
    int valueTakeCartridge;

    int cmdCheckTPlush;
    int valueCheckTPlush;

    int cmdCheckTray;
    int valueCheckTray;

    enum class Step
    {
        step_home_state,
        step_0_IDLE_state,
        step_1_check_take_cartridge_empty,
        step_2_add_cartridge_to_fill_machine,
        step_3_take_cartridge_fill_machine_to_scale,
        step_4_count_and_check_T_plush_cartridge,
        step_5_check_scale_cartridge,
        step_6_take_cartridge_from_scale_to_tray,
        step_7_check_tray_empty_position,
        step_8_put_cartridge_to_empty_position_on_tray,
        step_9_arrange_batch_cartridge_on_tray,
        step_10_take_cartridge_to_position_fail,
        check_cartridge_on_tray_empty,
        enable_robot
    };

    Step current_step_ = Step::step_0_IDLE_state;
    // Step current_step_ = Step::step_4_count_and_check_T_plush_cartridge;

    void runMotionPlan()
    {
        while (1)
        {
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
            case Step::step_4_count_and_check_T_plush_cartridge:
                step_4_count_and_check_T_plush_cartridge();
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
            case Step::step_9_arrange_batch_cartridge_on_tray:
                step_9_arrange_batch_cartridge_on_tray();
                break;
            case Step::step_10_take_cartridge_to_position_fail:
                step_10_take_cartridge_to_position_fail();
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
        // moveAndConfirm(0);
        rclcpp::sleep_for(2000ms);
        current_step_ = Step::step_0_IDLE_state;
    }

    void step_0_IDLE_state()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 0] IDLE...");
        // cmdCheckTakeCartridge = msg->values[0];
        // valueTakeCartridge = msg->values[1];
        if (cmdCheckTakeCartridge == 1)
        {
            // RCLCPP_INFO(this->get_logger(), "Check take cartridge");
            current_step_ = Step::check_cartridge_on_tray_empty;
        }
        if (cmdCheckTPlush == 1)
            current_step_ = Step::step_4_count_and_check_T_plush_cartridge;
        if (cmdCheckTray == 1)
            current_step_ = Step::step_7_check_tray_empty_position;
        // if (take_cartridge)
        // {
        //     take_cartridge = false;
        //     current_step_ = Step::step_1_check_take_cartridge_empty;
        //     row_take_cartirdge = 1;
        //     take_new_cartridge = true;
        //     end_cartridge_final = false;
        // }
    }

    void step_1_check_take_cartridge_empty()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 1] Check row cartridge");
        if (row_take_cartirdge <= 6)
        {
            // sendDigitalOutput(7, 0);
            // if (take_new_cartridge)
            // {
            //     setLinearSpeed(100);
            //     setLinearAccel(50);
            //     moveAndConfirm(1);
            //     rclcpp::sleep_for(1000ms);
            //     setLinearSpeed(80);
            //     setLinearAccel(20);
            //     moveRelAndWaitUntilReached(1.3 * (row_take_cartirdge - 1), -93.2 * (row_take_cartirdge - 1), 0.0);
            //     rclcpp::sleep_for(100ms);
            // }
            // else
            // {
            //     setLinearSpeed(80);
            //     setLinearAccel(20);
            //     moveRelAndWaitUntilReached(1.3, -93.2, 0.0);
            // }
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
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(20);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(0, 0, -101);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(7, 1);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(5);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(0, 0, 30);
        // setLinearSpeed(100);
        // setLinearAccel(30);
        // moveRelAndWaitUntilReached(0, 0, 170);
        // setLinearSpeed(100);
        // setLinearAccel(100);
        // moveAndConfirm(2);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(3); // waiting add cart
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(100);
        // setLinearAccel(30);
        // moveAndConfirm(4); // add cart to fill machine
        // setLinearSpeed(7);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(77, 0, 0);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(7, 0);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(5);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(-65, 0, 0);
        // setLinearSpeed(15);
        // setLinearAccel(15);
        // moveRelAndWaitUntilReached(0, -11, 0);
        // setLinearSpeed(10);
        // setLinearAccel(10);
        // moveRelAndWaitUntilReached(32, 0, 0);
        // setLinearSpeed(100);
        // setLinearAccel(30);
        // moveRelAndWaitUntilReached(-432, 5, 262);
        // rclcpp::sleep_for(1000ms);
        // if (row_take_cartirdge <= 2)
        //     current_step_ = Step::step_3_take_cartridge_fill_machine_to_scale;
        // else if (TPlush_cartridge_fail)
        //     current_step_ = Step::step_6_take_cartridge_from_scale_to_tray;
        // else
        //     current_step_ = Step::step_10_take_cartridge_to_position_fail;
    }

    void step_3_take_cartridge_fill_machine_to_scale()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 3] Take the cartirdge from Fill machine to scale");
        // take_new_cartridge = true;
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(5);
        // setLinearSpeed(7);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(79, 1.5, 0);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(7, 1);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(40);
        // setLinearAccel(20);
        // moveRelAndWaitUntilReached(-66, 0, 0);
        // setLinearSpeed(70);
        // setLinearAccel(35);
        // moveRelAndWaitUntilReached(-250, 0, 170);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(6);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(7);
        // current_step_ = Step::step_4_count_and_check_T_plush_cartridge;
        // setLinearSpeed(10);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(0, 1.7, -64);
        // rclcpp::sleep_for(200ms);
        // sendDigitalOutput(7, 0);
        // rclcpp::sleep_for(200ms);
        // setLinearSpeed(50);
        // setLinearAccel(50);
        // moveRelAndWaitUntilReached(0, 0, 180);
        // rclcpp::sleep_for(200ms);
        // if (row_take_cartirdge >= 6)
        //     current_step_ = Step::step_6_take_cartridge_from_scale_to_tray;
        // else if (row_take_cartirdge <= 2)
        //     current_step_ = Step::step_1_check_take_cartridge_empty;
        // else
        //     current_step_ = Step::step_1_check_take_cartridge_empty;
    }

    void step_4_count_and_check_T_plush_cartridge()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 4] Count and check T_plush product...");
        rclcpp::sleep_for(2000ms);
        if (detection_Tplus_.ready())
        {
            // TPlush_cartridge_fail = true;
            RCLCPP_INFO(this->get_logger(), "[STEP 4] Check T_plush OK");
            writeToPLC("192.168.27.6", 185, 4, {1, 2});
            current_step_ = Step::step_0_IDLE_state;
        }
        else
        {
            // TPlush_cartridge_fail = false;
            RCLCPP_INFO(this->get_logger(), "[STEP 4] Check T_plush Fail");
            writeToPLC("192.168.27.6", 185, 4, {1, 3});
            current_step_ = Step::step_0_IDLE_state;
        }
        // setLinearSpeed(10);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(0, 1.7, -64);
        // rclcpp::sleep_for(200ms);
        // sendDigitalOutput(7, 0);
        // rclcpp::sleep_for(200ms);
        // setLinearSpeed(50);
        // setLinearAccel(50);
        // moveRelAndWaitUntilReached(0, 0, 180);
        // rclcpp::sleep_for(200ms);
        // if (row_take_cartirdge >= 6)
        //     if (TPlush_cartridge_fail)
        //         current_step_ = Step::step_6_take_cartridge_from_scale_to_tray;
        //     else
        //         current_step_ = Step::step_10_take_cartridge_to_position_fail;
        // else
        //     current_step_ = Step::step_1_check_take_cartridge_empty;
    }
    void step_5_check_scale_cartridge()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 5] Count and check T_plush product...");
    }

    void step_6_take_cartridge_from_scale_to_tray()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 6] Count and check T_plush product...");
        // sendDigitalOutput(8, 0);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(8);
        // setLinearSpeed(50);
        // setLinearAccel(20);
        // moveRelAndWaitUntilReached(0, 2, -134.7);
        // setLinearSpeed(10);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(2, 0, -50);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(8, 1);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(80);
        // setLinearAccel(40);
        // moveRelAndWaitUntilReached(0, 0, 150);
        // setLinearSpeed(80);
        // setLinearAccel(40);
        // moveAndConfirm(9);
        // current_step_ = Step::step_7_check_tray_empty_position;
    }

    void step_7_check_tray_empty_position()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 7] Count and check tray empty...");
        rclcpp::sleep_for(2000ms);
        if (slot_id > 0)
            current_step_ = Step::step_8_put_cartridge_to_empty_position_on_tray;
        else
            current_step_ = Step::step_7_check_tray_empty_position;
    }

    void step_8_put_cartridge_to_empty_position_on_tray()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 8] Put the cartridge to the tray empty...");
        slot_id_temp = slot_id;
        // take_new_cartridge = true;
        if (slot_id_full == 100)
            writeToPLC("192.168.27.6", 185, 8, {1, 100});
        else if (slot_id == 2)
            writeToPLC("192.168.27.6", 185, 8, {1, 1});
        else if (slot_id == 4)
            writeToPLC("192.168.27.6", 185, 8, {1, 2});
        else if (slot_id == 6)
            writeToPLC("192.168.27.6", 185, 8, {1, 3});
        else if (slot_id == 8)
            writeToPLC("192.168.27.6", 185, 8, {1, 4});
        else if (slot_id == 1)
            writeToPLC("192.168.27.6", 185, 8, {1, 6});
        else if (slot_id == 3)
            writeToPLC("192.168.27.6", 185, 8, {1, 7});
        else if (slot_id == 5)
            writeToPLC("192.168.27.6", 185, 8, {1, 8});
        else if (slot_id == 7)
            writeToPLC("192.168.27.6", 185, 8, {1, 9});
        current_step_ = Step::step_0_IDLE_state;
        rclcpp::sleep_for(5000ms);
        // setLinearSpeed(10);
        // setLinearAccel(10);
        // moveRelAndWaitUntilReached(0, 0, -118.5);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(8, 0);
        // if (slot_id_temp <= 6)
        // {
        //     setLinearSpeed(4);
        //     setLinearAccel(4);
        //     moveRelAndWaitUntilReached(0, 0, 21.3);
        //     setLinearSpeed(5);
        //     setLinearAccel(5);
        //     moveRelAndWaitUntilReached(10, 0, 0);
        //     sendDigitalOutput(8, 1);
        //     rclcpp::sleep_for(500ms);
        //     setLinearSpeed(5);
        //     setLinearAccel(5);
        //     moveRelAndWaitUntilReached(-30, 0, 0);
        //     rclcpp::sleep_for(500ms);
        //     setLinearSpeed(5);
        //     setLinearAccel(5);
        //     moveRelAndWaitUntilReached(5, 0, 0);
        // }
        // setLinearSpeed(50);
        // setLinearAccel(50);
        // moveRelAndWaitUntilReached(0, 0, 180);
        // sendDigitalOutput(8, 0);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(18);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(19);
        // if (end_cartridge_final)
        // {
        //     current_step_ = Step::step_0_IDLE_state;
        // }
        // else
        // {
        //     if (row_take_cartirdge >= 6)
        //         end_cartridge_final = true;
        //     if (row_take_cartirdge <= 2)
        //         current_step_ = Step::step_1_check_take_cartridge_empty;
        //     else
        //         current_step_ = Step::step_3_take_cartridge_fill_machine_to_scale;
        // }
    }

    void step_9_arrange_batch_cartridge_on_tray()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 9] Count and check T_plush product...");
    }

    void step_10_take_cartridge_to_position_fail()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 10] Take cartridge to position fail...");
        // sendDigitalOutput(8, 0);
        // setLinearSpeed(100);
        // setLinearAccel(50);
        // moveAndConfirm(8);
        // setLinearSpeed(50);
        // setLinearAccel(20);
        // moveRelAndWaitUntilReached(0, 2, -134.7);
        // setLinearSpeed(10);
        // setLinearAccel(5);
        // moveRelAndWaitUntilReached(2, 0, -50);
        // rclcpp::sleep_for(500ms);
        // sendDigitalOutput(8, 1);
        // rclcpp::sleep_for(500ms);
        // setLinearSpeed(80);
        // setLinearAccel(40);
        // moveRelAndWaitUntilReached(0, 0, 150);
        // moveAndConfirm(20);
        // setLinearSpeed(5);
        // setLinearAccel(2);
        // moveRelAndWaitUntilReached(0, 0, -4);
        // rclcpp::sleep_for(200ms);
        // sendDigitalOutput(8, 0);
        // rclcpp::sleep_for(200ms);
        // setLinearSpeed(80);
        // setLinearAccel(40);
        // moveRelAndWaitUntilReached(0, 0, 254);
        // moveAndConfirm(19);
        // if (end_cartridge_final)
        // {
        //     current_step_ = Step::step_0_IDLE_state;
        // }
        // else
        // {
        //     if (row_take_cartirdge >= 6)
        //         end_cartridge_final = true;
        //     if (row_take_cartirdge <= 2)
        //         current_step_ = Step::step_1_check_take_cartridge_empty;
        //     else
        //         current_step_ = Step::step_3_take_cartridge_fill_machine_to_scale;
        // }
    }

    void check_cartridge_on_tray_empty()
    {
        RCLCPP_INFO(this->get_logger(), "[AI check] Check cartridge empty on the tray");
        // rclcpp::sleep_for(2000ms);
        if (detection_state_.ready())
        {
            writeToPLC("192.168.27.6", 185, 0, {1, 2});
            current_step_ = Step::step_0_IDLE_state;
            rclcpp::sleep_for(5000ms);
        }
        else if (++count_check_cartridge_empty_tray < 5)
        {
            current_step_ = Step::check_cartridge_on_tray_empty;
        }
        else
        {
            writeToPLC("192.168.27.6", 185, 0, {1, 3});
            current_step_ = Step::step_0_IDLE_state;
            rclcpp::sleep_for(5000ms);
        }
    }

    // enable_robot
    void enable_robot()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP 9] Count and check T_plush product...");
        callEnable();
        // current_step_ = 0;
    }

    void yolov8CallbackCam1(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        detection_state_.count_cam1 = 0;
        detection_Tplus_.count_Tplus_cam1 = 0;
        std::vector<std::pair<int, int>> corners = {
            {164, 299},
            {518, 297},
            {521, 432},
            {166, 436}};
        int min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        int max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        std::vector<std::pair<int, int>> TPlushcorners = {
            {36, 328},
            {626, 325},
            {629, 500},
            {47, 501}};
        int Tplushmin_x = std::min({TPlushcorners[0].first, TPlushcorners[1].first, TPlushcorners[2].first, TPlushcorners[3].first});
        int Tplushmax_x = std::max({TPlushcorners[0].first, TPlushcorners[1].first, TPlushcorners[2].first, TPlushcorners[3].first});
        int Tplushmin_y = std::min({TPlushcorners[0].second, TPlushcorners[1].second, TPlushcorners[2].second, TPlushcorners[3].second});
        int Tplushmax_y = std::max({TPlushcorners[0].second, TPlushcorners[1].second, TPlushcorners[2].second, TPlushcorners[3].second});

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {

                const std::string &class_id = r.hypothesis.class_id;
                float confidence = r.hypothesis.score;
                if (class_id == "2")
                {
                    if (confidence > 0.5)
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
                else if (class_id == "0")
                {
                    if (confidence > 0.7)
                    {
                        float TPlushcenter_x = det.bbox.center.position.x;
                        float TPlushcenter_y = det.bbox.center.position.y;
                        if (TPlushcenter_x >= Tplushmin_x && TPlushcenter_x <= Tplushmax_x &&
                            TPlushcenter_y >= Tplushmin_y && TPlushcenter_y <= Tplushmax_y)
                        {
                            detection_Tplus_.count_Tplus_cam1++;
                            if (detection_Tplus_.count_Tplus_cam1 > 4)
                                detection_Tplus_.count_Tplus_cam1 = 4;
                        }
                    }
                }
            }
        }
        if (current_step_ == Step::check_cartridge_on_tray_empty)
        {
            if (detection_state_.count_cam1 > 0)
            {
                RCLCPP_INFO(this->get_logger(), "[Cam1] count_cam1 = %d", detection_state_.count_cam1);
            }
        }
        if (current_step_ == Step::step_4_count_and_check_T_plush_cartridge)
        {
            if (detection_Tplus_.count_Tplus_cam1 > 0)
            {
                RCLCPP_INFO(this->get_logger(), "[Cam1] count_Tplus_cam1 = %d", detection_Tplus_.count_Tplus_cam1);
            }
        }
    }

    void yolov8CallbackCam2(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        detection_state_.count_cam2 = 0;
        detection_Tplus_.count_Tplus_cam2 = 0;
        std::vector<std::pair<int, int>> corners = {
            {127, 287},
            {130, 428},
            {481, 278},
            {480, 414}};
        int min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        int min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        int max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        std::vector<std::pair<int, int>> TPlushcorners = {
            {22, 497},
            {33, 340},
            {606, 327},
            {603, 498}};
        int Tplushmin_x = std::min({TPlushcorners[0].first, TPlushcorners[1].first, TPlushcorners[2].first, TPlushcorners[3].first});
        int Tplushmax_x = std::max({TPlushcorners[0].first, TPlushcorners[1].first, TPlushcorners[2].first, TPlushcorners[3].first});
        int Tplushmin_y = std::min({TPlushcorners[0].second, TPlushcorners[1].second, TPlushcorners[2].second, TPlushcorners[3].second});
        int Tplushmax_y = std::max({TPlushcorners[0].second, TPlushcorners[1].second, TPlushcorners[2].second, TPlushcorners[3].second});

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {

                const std::string &class_id = r.hypothesis.class_id;
                float confidence = r.hypothesis.score;
                if (class_id == "2")
                {
                    if (confidence > 0.5)
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
                else if (class_id == "0")
                {
                    if (confidence > 0.7)
                    {
                        float TPlushcenter_x = det.bbox.center.position.x;
                        float TPlushcenter_y = det.bbox.center.position.y;
                        if (TPlushcenter_x >= Tplushmin_x && TPlushcenter_x <= Tplushmax_x &&
                            TPlushcenter_y >= Tplushmin_y && TPlushcenter_y <= Tplushmax_y)
                        {
                            detection_Tplus_.count_Tplus_cam2++;
                            if (detection_Tplus_.count_Tplus_cam2 > 4)
                                detection_Tplus_.count_Tplus_cam2 = 4;
                        }
                    }
                }
            }
        }
        if (current_step_ == Step::check_cartridge_on_tray_empty)
        {
            if (detection_state_.count_cam2 > 0)
            {
                RCLCPP_INFO(this->get_logger(), "[Cam1] count_cam2 = %d", detection_state_.count_cam2);
            }
        }
        if (current_step_ == Step::step_4_count_and_check_T_plush_cartridge)
        {
            if (detection_Tplus_.count_Tplus_cam2 > 0)
            {
                RCLCPP_INFO(this->get_logger(), "[Cam1] count_Tplus_cam2 = %d", detection_Tplus_.count_Tplus_cam2);
            }
        }
    }

    void yolov8CallbackCamRealsense(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        vision_msgs::msg::Detection2D tray_boundingbox;
        bool found_tray = false;

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {
                // RCLCPP_INFO(this->get_logger(), "[CamRealsense] Detected class_id: %s", r.hypothesis.class_id.c_str());
                if (r.hypothesis.class_id == CLASS_TRAY)
                {
                    tray_boundingbox = det;
                    found_tray = true;
                    break;
                }
            }
            if (found_tray)
                break;
        }

        if (!found_tray)
        {
            // RCLCPP_WARN(this->get_logger(), "[TrayDetect] Can't detect the tray on the image");
            return;
        }

        int rows = 4, cols = 2;
        float cell_w = tray_boundingbox.bbox.size_x / cols;
        float cell_h = tray_boundingbox.bbox.size_y / rows;
        bool occupied[4][2] = {{false}};
        bool blocked[4][2] = {{false}};

        float tray_cx = tray_boundingbox.bbox.center.position.x;
        float tray_cy = tray_boundingbox.bbox.center.position.y;

        // RCLCPP_INFO(this->get_logger(), "[Tray] Tray center=(%.1f, %.1f), size=(%.1f x %.1f)", tray_cx, tray_cy, tray_boundingbox.bbox.size_x, tray_boundingbox.bbox.size_y);
        // RCLCPP_INFO(this->get_logger(), "[Tray] Grid: %d rows x %d cols", rows, cols);
        // RCLCPP_INFO(this->get_logger(), "[Tray] Each cell size: %.1f x %.1f", cell_w, cell_h);

        // std::set<std::pair<int, int>> visited;

        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {
                if ((r.hypothesis.class_id == CLASS_LINE_PASS) || (r.hypothesis.class_id == CLASS_LINE_FALL))
                {
                    float x = det.bbox.center.position.x;
                    float y = det.bbox.center.position.y;

                    float rel_x = x - tray_cx + tray_boundingbox.bbox.size_x / 2;
                    float rel_y = y - tray_cy + tray_boundingbox.bbox.size_y / 2;

                    int col = rel_x / cell_w;
                    int row = rel_y / cell_h;

                    if (row >= 0 && row < rows && col >= 0 && col < cols)
                    {
                        occupied[row][col] = true;
                        // RCLCPP_INFO(this->get_logger(), "[Occupied] Product at (%.1f, %.1f) => rel(%.1f, %.1f) => slot[%d][%d]", x, y, rel_x, rel_y, row, col);
                    }
                }
            }
        }

        // Check cartridge error
        for (const auto &det : msg->detections)
        {
            for (const auto &r : det.results)
            {
                if (r.hypothesis.class_id == CLASS_CARTRIDGE)
                {
                    float x = det.bbox.center.position.x;
                    float y = det.bbox.center.position.y;

                    float rel_x = x - tray_cx + tray_boundingbox.bbox.size_x / 2;
                    float rel_y = y - tray_cy + tray_boundingbox.bbox.size_y / 2;
                    int col = rel_x / cell_w;
                    int row = rel_y / cell_h;

                    if (row >= 0 && row < rows && col >= 0 && col < cols)
                    {
                        blocked[row][col] = true;
                        // RCLCPP_WARN(this->get_logger(), "[Blocked] slot[%d][%d] occupied by a reclining object", row, col);
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "[Slot Matrix]");
        for (int r = 0; r < rows; ++r)
        {
            std::stringstream ss;
            for (int c = 0; c < cols; ++c)
            {
                ss << "[";
                if (blocked[r][c])
                    ss << "B"; // Blocked by cartridge reclining
                else if (occupied[r][c])
                    ss << "X"; // have line_pass
                else
                    ss << " "; // Slot empty
                ss << "]";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        publishEmptySlots(occupied,
                          blocked,
                          tray_boundingbox.bbox.center.position.x,
                          tray_boundingbox.bbox.center.position.y,
                          tray_boundingbox.bbox.size_x,
                          tray_boundingbox.bbox.size_y, rows, cols, found_tray);

        slot_id = findPreferredEmptySlot(occupied, blocked);
        if (slot_id > 0)
        {
            RCLCPP_INFO(this->get_logger(), "[SlotSelect] Empty slot selected: %d", slot_id);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[SlotSelect] No empty slot found!");
        }
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
        // if (!checkPLCConnection(1000))
        // {
        //     RCLCPP_WARN(get_logger(), "PLC not reachable, please check cable/IP.");
        // }
        // else
        // {
        //     RCLCPP_INFO(get_logger(), "PLC connection OK!");
        // }
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

        std::ostringstream oss;
        oss << "[READ RESP] count=" << msg->values.size() << " values=[";
        for (size_t i = 0; i < msg->values.size(); ++i)
        {
            oss << msg->values[i] << (i + 1 < msg->values.size() ? ", " : "");
        }
        oss << "]";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        cmdCheckTakeCartridge = msg->values[0];
        valueTakeCartridge = msg->values[1];
        cmdCheckTPlush = msg->values[2];
        valueCheckTPlush = msg->values[3];
        cmdCheckTray = msg->values[4];
        valueCheckTray = msg->values[5];
        RCLCPP_INFO(this->get_logger(), "Check cartridge cmd: %d , value: %d", cmdCheckTakeCartridge, valueTakeCartridge);
        RCLCPP_INFO(this->get_logger(), "Check T Plush cmd: %d , value: %d", cmdCheckTPlush, valueCheckTPlush);
        RCLCPP_INFO(this->get_logger(), "Check Tray cmd: %d , value: %d", cmdCheckTray, valueCheckTray);
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
