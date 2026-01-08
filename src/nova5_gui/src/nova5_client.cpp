#include "nova5_gui/nova5_client.hpp"
#include <thread>
#include <fstream>
#include <regex>

Nova5Client::Nova5Client(std::shared_ptr<rclcpp::Node> node, QObject* parent)
  : QObject(parent), node_(node)
{
  cli_enable_    = node_->create_client<dobot_msgs_v3::srv::EnableRobot>("/nova5/dobot_bringup/EnableRobot");
  cli_get_angle_ = node_->create_client<dobot_msgs_v3::srv::GetAngle>("/nova5/dobot_bringup/GetAngle");
  cli_joint_movj_= node_->create_client<dobot_msgs_v3::srv::JointMovJ>("/nova5/dobot_bringup/JointMovJ");

  angles_qt_.clear();
  for(int i=0;i<6;i++) angles_qt_.push_back(0.0);
}

void Nova5Client::enableRobot(double load){
  setBusy(true);
  auto req = std::make_shared<dobot_msgs_v3::srv::EnableRobot::Request>();
  req->load = load;

  if(!cli_enable_->wait_for_service(std::chrono::seconds(2))){
    emit toast("EnableRobot service not available");
    setBusy(false);
    return;
  }

  auto fut = cli_enable_->async_send_request(req);

  std::thread([this, fut = std::move(fut)]() mutable {
    try{
      fut.get();
      emit toast("EnableRobot OK");
    }catch(...){
      emit toast("EnableRobot FAILED");
    }
    setBusy(false);
  }).detach();
}

void Nova5Client::getAngles(){
  setBusy(true);
  auto req = std::make_shared<dobot_msgs_v3::srv::GetAngle::Request>();

  if(!cli_get_angle_->wait_for_service(std::chrono::seconds(2))){
    emit toast("GetAngle service not available");
    setBusy(false);
    return;
  }

  auto fut = cli_get_angle_->async_send_request(req);

  std::thread([this, fut = std::move(fut)]() mutable {
    try{
      auto resp = fut.get();
      // resp->angle ví dụ: "{132.673477,-0.078284,68.418701,132.360580,0.001147,0.000385}"
      std::array<double,6> q{0,0,0,0,0,0};

      const std::string s = resp->angle;                 // string angle
      std::regex num_re(R"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)");
      std::sregex_iterator it(s.begin(), s.end(), num_re), end;

      int i = 0;
      for (; it != end && i < 6; ++it, ++i) {
        q[i] = std::stod(it->str());                     // nếu muốn hiển thị độ: * 180.0 / M_PI
      }

      if (i < 6) {
        emit toast(QString("Parse GetAngle failed (found %1)").arg(i));
      } else {
        setAngles(q);
        emit toast(QString("Angles DEG: [%1, %2, %3, %4, %5, %6]")
                   .arg(q[0]).arg(q[1]).arg(q[2]).arg(q[3]).arg(q[4]).arg(q[5]));
      }
    } catch(...) {
      emit toast("GetAngle FAILED");
    }
    setBusy(false);
  }).detach();
}



// void Nova5Client::saveAnglesYaml(QString path_qt){
//   YAML::Node root;
//   YAML::Node q;
//   for(auto &v : angles_qt_) q.push_back(v.toDouble());
//   root["q_deg"] = q;

//   try{
//     std::ofstream f(path_qt.toStdString());
//     f << root;
//     emit toast(QString("Saved: %1").arg(path_qt));
//   }catch(...){
//     emit toast("Save YAML FAILED");
//   }
// }

void Nova5Client::saveAnglesYaml(QString path_qt, QVariantList anglesFromUi) {
  YAML::Node root;
  YAML::Node q;

  if (anglesFromUi.size() >= 6) {
    // Ưu tiên giá trị ngay trên UI
    for (int i = 0; i < 6; ++i) q.push_back(anglesFromUi[i].toDouble());
  } else {
    // Fallback về state trong C++
    for (auto &v : angles_qt_) q.push_back(v.toDouble());
  }
  root["q_deg"] = q;

  try {
    std::ofstream f(path_qt.toStdString());
    f << root;
    emit toast(QString("Saved: %1").arg(path_qt));
  } catch(...) {
    emit toast(QString("Save YAML FAILED"));
  }
}

void Nova5Client::jointMovJ(double j1,double j2,double j3,double j4,double j5,double j6){
  setBusy(true);
  auto req = std::make_shared<dobot_msgs_v3::srv::JointMovJ::Request>();
  req->j1 = j1;
  req->j2 = j2;
  req->j3 = j3;
  req->j4 = j4;
  req->j5 = j5;
  req->j6 = j6;

  // ✅ param_value là vector<string>, không phải vector<double>
  req->param_value = std::vector<std::string>();

  if(!cli_joint_movj_->wait_for_service(std::chrono::seconds(2))){
    emit toast("JointMovJ service not available");
    setBusy(false);
    return;
  }

  auto fut = cli_joint_movj_->async_send_request(req);

  std::thread([this, fut = std::move(fut)]() mutable {
    try{
      fut.get();
      emit toast("JointMovJ SENT");
    }catch(...){
      emit toast("JointMovJ FAILED");
    }
    setBusy(false);
  }).detach();
}
