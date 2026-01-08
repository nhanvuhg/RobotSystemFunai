#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <rclcpp/rclcpp.hpp>
#include "nova5_gui/nova5_client.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nova5_gui_node");

  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  auto nova5 = new Nova5Client(node);
  engine.rootContext()->setContextProperty("nova5", nova5);

  engine.load(QUrl("qrc:/qml/Main.qml"));
  if (engine.rootObjects().isEmpty()) {
    rclcpp::shutdown();
    return -1;
  }

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
