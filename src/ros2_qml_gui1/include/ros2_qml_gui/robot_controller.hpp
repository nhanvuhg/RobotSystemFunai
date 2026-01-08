#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class RobotController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString systemStatus READ systemStatus NOTIFY systemStatusChanged)
    Q_PROPERTY(QString errorMessage READ errorMessage NOTIFY errorMessageChanged)
    Q_PROPERTY(int selectedRow READ selectedRow NOTIFY selectedRowChanged)
    Q_PROPERTY(int selectedSlot READ selectedSlot NOTIFY selectedSlotChanged)

public:
    explicit RobotController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);
    
    QString systemStatus() const { return system_status_; }
    QString errorMessage() const { return error_message_; }
    int selectedRow() const { return selected_row_; }
    int selectedSlot() const { return selected_slot_; }

public slots:
    void enableSystem(bool enable);
    void emergencyStop(bool stop);
    void setManualMode(bool enable);
    void setAiMode(bool enable);
    void switchCamera(int cameraId);
    void selectRow(int row);
    void selectSlot(int slot);
    void gotoState(const QString& state);

signals:
    void systemStatusChanged();
    void errorMessageChanged();
    void selectedRowChanged();
    void selectedSlotChanged();
    void serviceCallResult(bool success, QString message);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_mode_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ai_mode_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_select_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_row_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goto_state_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_slot_sub_;
    
    // State
    QString system_status_;
    QString error_message_;
    int selected_row_;
    int selected_slot_;
    
    void callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
};

#endif // ROBOT_CONTROLLER_HPP
