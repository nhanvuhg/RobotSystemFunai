#include "ros2_qml_gui/cam_node.hpp"
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

CamNode::CamNode(QQmlApplicationEngine &engine)
    : QObject(), rclcpp::Node("qml_cam_node"), engine_(&engine) {}

void CamNode::setup(const std::vector<std::string> &topics)
{
    subs_.clear();

    std::vector<std::string> limitedTopics = topics;
    if (limitedTopics.size() > maxCameras_)
        limitedTopics.resize(maxCameras_);

    while (providers_.size() > limitedTopics.size())
    {
        delete providers_.back();
        providers_.pop_back();
    }

    while (providers_.size() < limitedTopics.size())
    {
        QString providerId = QString("cam_%1").arg(providers_.size());
        auto *provider = new CamProvider();
        engine_->addImageProvider(providerId, provider);
        providers_.push_back(provider);
    }

    cameraList_.clear();

    for (size_t i = 0; i < limitedTopics.size(); ++i)
    {
        QString providerId = QString("cam_%1").arg(i);
        CamProvider *provider = providers_[i];

        // Use BEST_EFFORT QoS to match image_transport publisher
        auto qos = rclcpp::QoS(10).best_effort();
        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            limitedTopics[i], qos,
            [this, i](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
            {
                try
                {
                    auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                    // Resize to 640x360 (16:9) to match camera aspect ratio
                    cv::Mat resized;
                    cv::resize(cvimg, resized, cv::Size(640, 360));
                    QImage qimg(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_BGR888);
                    providers_[i]->setImage(qimg.copy());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Image error for cam %lu: %s", i, e.what());
                }
            });

        subs_.push_back(sub);

        QVariantMap cam;
        cam["name"] = QString("Camera %1").arg(i + 1);
        cam["topic"] = QString::fromStdString(limitedTopics[i]);
        cam["providerId"] = providerId;
        cameraList_.append(cam);
    }

    emit cameraListChanged();
}

QStringList CamNode::getAvailableImageTopics()
{
    QStringList result;
    // Topics to exclude from settings dropdown
    QStringList excludePatterns = {"cam2", "/ai/", "image_raw"};

    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &pair : topics_and_types)
    {
        const auto &topic_name = pair.first;
        const auto &types = pair.second;
        QString qTopic = QString::fromStdString(topic_name);

        // Skip excluded topics
        bool excluded = false;
        for (const auto &pattern : excludePatterns)
            if (qTopic.contains(pattern)) { excluded = true; break; }
        if (excluded) continue;

        for (const auto &type : types)
        {
            if (type == "sensor_msgs/msg/Image")
            {
                result << qTopic;
            }
        }
    }
    result.sort();
    return result;
}

void CamNode::updateCameraTopic(int index, const QString &newTopic)
{
    RCLCPP_INFO(this->get_logger(), "[GUI] Updating camera %d to topic: %s", index, newTopic.toStdString().c_str());
    
    if (index < 0 || index >= static_cast<int>(subs_.size())) {
        RCLCPP_ERROR(this->get_logger(), "[GUI] Invalid camera index: %d", index);
        return;
    }

    try {
        // Reset old subscription
        subs_[index].reset();

        // Create new subscription
        // Use BEST_EFFORT QoS to match image_transport publisher
        auto qos = rclcpp::QoS(10).best_effort();
        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            newTopic.toStdString(), qos,
            [this, index](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
            {
                try
                {
                    auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                    // Resize to 640x360 (16:9) to match overlay aspect ratio
                    cv::Mat resized;
                    cv::resize(cvimg, resized, cv::Size(640, 360));
                    QImage qimg(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_BGR888);
                    providers_[index]->setImage(qimg.copy());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Image error: %s", e.what());
                }
            });

        subs_[index] = sub;

        QVariantMap cam = cameraList_[index].toMap();
        cam["topic"] = newTopic;
        cameraList_[index] = cam;

        emit cameraListChanged();
        
        RCLCPP_INFO(this->get_logger(), "[GUI] Successfully updated camera %d subscription", index);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[GUI] Failed to update topic: %s", e.what());
    }
}

void CamNode::refreshTopics()
{
    QStringList qTopics = getAvailableImageTopics();

    if (qTopics.size() > maxCameras_)
        qTopics = qTopics.mid(0, maxCameras_);

    std::vector<std::string> topics;
    for (const QString &qstr : qTopics)
        topics.push_back(qstr.toStdString());

    setup(topics);
}

QVariantList CamNode::cameraList() const
{
    return cameraList_;
}
