#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <QObject>
#include <QQmlApplicationEngine>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QVariantList>
#include <QStringList>
#include "cam_provider.hpp"

class CamNode : public QObject, public rclcpp::Node {
    Q_OBJECT
    Q_PROPERTY(QVariantList cameraList READ cameraList NOTIFY cameraListChanged)

public:
    CamNode(QQmlApplicationEngine &engine);
    void setup(const std::vector<std::string> &topics);

    Q_INVOKABLE QStringList getAvailableImageTopics();
    Q_INVOKABLE void updateCameraTopic(int index, const QString &newTopic);
    Q_INVOKABLE void refreshTopics();

    QVariantList cameraList() const;

signals:
    void cameraListChanged();

private:
    std::vector<CamProvider *> providers_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
    QVariantList cameraList_;
    QQmlApplicationEngine *engine_;
    int maxCameras_ = 4;
};

#endif // CAM_NODE_HPP
