#pragma once
#include <QObject>
#include <QVariantList>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <dobot_msgs_v3/srv/enable_robot.hpp>
#include <dobot_msgs_v3/srv/get_angle.hpp>
#include <dobot_msgs_v3/srv/joint_mov_j.hpp>

class Nova5Client : public QObject {
  Q_OBJECT
  Q_PROPERTY(QVariantList angles READ angles NOTIFY anglesChanged)
  Q_PROPERTY(bool busy READ busy NOTIFY busyChanged)
public:
  explicit Nova5Client(std::shared_ptr<rclcpp::Node> node, QObject* parent=nullptr);

  Q_INVOKABLE void enableRobot(double load);
  Q_INVOKABLE void getAngles();
  // Q_INVOKABLE void saveAnglesYaml(QString path);
  Q_INVOKABLE void jointMovJ(double j1,double j2,double j3,double j4,double j5,double j6);
  Q_INVOKABLE void saveAnglesYaml(QString path, QVariantList anglesFromUi);

  QVariantList angles() const { return angles_qt_; }
  bool busy() const { return busy_; }

signals:
  void toast(QString msg);
  void anglesChanged();
  void busyChanged();

private:
  void setBusy(bool b){ if(busy_!=b){ busy_=b; emit busyChanged(); } }
  void setAngles(const std::array<double,6>& q){
    angles_qt_.clear(); for(double v:q) angles_qt_.push_back(v); emit anglesChanged();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<dobot_msgs_v3::srv::EnableRobot>::SharedPtr cli_enable_;
  rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedPtr cli_get_angle_;
  rclcpp::Client<dobot_msgs_v3::srv::JointMovJ>::SharedPtr cli_joint_movj_;
  QVariantList angles_qt_;
  bool busy_=false;
};
