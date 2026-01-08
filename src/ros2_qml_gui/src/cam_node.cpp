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

        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            limitedTopics[i], 10,
            [this, i](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
            {
                try
                {
                    auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                    // auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                    // cv::Mat cvimg = cv_ptr->image;
                    QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
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
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &pair : topics_and_types)
    {
        const auto &topic_name = pair.first;
        const auto &types = pair.second;
        for (const auto &type : types)
        {
            if (type == "sensor_msgs/msg/Image")
            {
                result << QString::fromStdString(topic_name);
            }
        }
    }
    return result;
}

void CamNode::updateCameraTopic(int index, const QString &newTopic)
{
    if (index < 0 || index >= static_cast<int>(subs_.size()))
        return;

    subs_[index].reset();

    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        newTopic.toStdString(), 10,
        [this, index](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
        {
            try
            {
                auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
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
