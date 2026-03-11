#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>

using vision_msgs::msg::Detection2DArray;

static inline void draw_detections(cv::Mat &img, const Detection2DArray &dets, double sx, double sy)
{
    for (const auto &det : dets.detections) {
        if (det.bbox.size_x <= 0 || det.bbox.size_y <= 0) continue;
        double cx = det.bbox.center.position.x * sx;
        double cy = det.bbox.center.position.y * sy;
        double w  = det.bbox.size_x * sx;
        double h  = det.bbox.size_y * sy;
        int x1 = static_cast<int>(cx - w/2.0);
        int y1 = static_cast<int>(cy - h/2.0);
        int x2 = static_cast<int>(cx + w/2.0);
        int y2 = static_cast<int>(cy + h/2.0);
        x1 = std::max(0, std::min(x1, img.cols-1));
        y1 = std::max(0, std::min(y1, img.rows-1));
        x2 = std::max(0, std::min(x2, img.cols-1));
        y2 = std::max(0, std::min(y2, img.rows-1));
        
        cv::rectangle(img, cv::Rect(cv::Point(x1,y1), cv::Point(x2,y2)), cv::Scalar(0,255,0), 1);
        std::string label = "";
        if (!det.results.empty()) {
            const auto &h = det.results[0].hypothesis;
            label = h.class_id + " " + cv::format("(%.2f)", h.score);
        }
        if (!label.empty()) {
            int base = 0; cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.25, 1, &base);
            int tx = std::max(0, x1), ty = std::max(ts.height+1, y1);
            cv::rectangle(img, cv::Rect(tx, ty-ts.height-1, ts.width+2, ts.height+2), cv::Scalar(0,255,0), cv::FILLED);
            cv::putText(img, label, cv::Point(tx+1, ty-1), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(0,0,0), 1);
        }
    }
}

static inline bool rosimg_to_cvmat(const sensor_msgs::msg::Image &msg, cv::Mat &out_bgr)
{
    const std::string enc = msg.encoding;
    if (enc == "bgr8") {
        cv::Mat ref(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()), msg.step);
        out_bgr = ref.clone();
        return true;
    } else if (enc == "rgb8") {
        cv::Mat ref_rgb(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()), msg.step);
        cv::cvtColor(ref_rgb, out_bgr, cv::COLOR_RGB2BGR);
        return true;
    } else if (enc == "mono8") {
        cv::Mat ref_gray(msg.height, msg.width, CV_8UC1, const_cast<unsigned char*>(msg.data.data()), msg.step);
        cv::cvtColor(ref_gray, out_bgr, cv::COLOR_GRAY2BGR);
        return true;
    } else if (enc == "yuv422_yuy2" || enc == "yuyv" || enc == "yuv422") {
        cv::Mat ref_yuyv(msg.height, msg.width, CV_8UC2, const_cast<unsigned char*>(msg.data.data()), msg.step);
        cv::cvtColor(ref_yuyv, out_bgr, cv::COLOR_YUV2BGR_YUY2);
        return true;
    } else if (enc == "uyvy" || enc == "yuv422_uyvy") {
        cv::Mat ref_uyvy(msg.height, msg.width, CV_8UC2, const_cast<unsigned char*>(msg.data.data()), msg.step);
        cv::cvtColor(ref_uyvy, out_bgr, cv::COLOR_YUV2BGR_UYVY);
        return true;
    } else {
        return false;
    }
}

static inline sensor_msgs::msg::Image cvmat_to_rosimg(const cv::Mat &bgr, const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::Image out;
    out.header = header;
    out.height = bgr.rows;
    out.width  = bgr.cols;
    out.encoding = "bgr8";
    out.is_bigendian = false;
    out.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * bgr.channels());
    size_t size = static_cast<size_t>(out.step) * out.height;
    out.data.resize(size);
    std::memcpy(out.data.data(), bgr.data, size);
    return out;
}

class OverlayForOneCam : public rclcpp::Node {
public:
    using ImageMsg = sensor_msgs::msg::Image;
    using BoxesMsg = Detection2DArray;

    OverlayForOneCam(const std::string &ns, const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
    : Node("overlay_"+ns, opts), ns_(ns)
    {
        image_topic_ = declare_parameter<std::string>("image_topic", "/"+ns_+"/image_raw");
        boxes_topic_ = declare_parameter<std::string>("boxes_topic", "/"+ns_+"/yolo/bounding_boxes");
        output_topic_= declare_parameter<std::string>("output_topic", "/"+ns_+"/image_overlay");
        out_w_       = declare_parameter<int>("output_width",  640);
        out_h_       = declare_parameter<int>("output_height", 360);

        image_sub_.reset(new message_filters::Subscriber<ImageMsg>(this, image_topic_));
        boxes_sub_.reset(new message_filters::Subscriber<BoxesMsg>(this, boxes_topic_));

        using Policy = message_filters::sync_policies::ApproximateTime<ImageMsg, BoxesMsg>;
        sync_.reset(new message_filters::Synchronizer<Policy>(Policy(10), *image_sub_, *boxes_sub_));
        sync_->registerCallback(std::bind(&OverlayForOneCam::callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_ = image_transport::create_publisher(this, output_topic_);
        RCLCPP_INFO(get_logger(), "[%s] Sub: %s, %s | Pub: %s", ns_.c_str(), image_topic_.c_str(), boxes_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const ImageMsg::ConstSharedPtr &img_msg, const BoxesMsg::ConstSharedPtr &boxes_msg)
    {
        cv::Mat bgr;
        if (!rosimg_to_cvmat(*img_msg, bgr)) {
            RCLCPP_WARN(get_logger(), "[%s] Unsupported encoding: %s", ns_.c_str(), img_msg->encoding.c_str());
            return;
        }
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(out_w_, out_h_));
        double sx = static_cast<double>(out_w_) / bgr.cols;
        double sy = static_cast<double>(out_h_) / bgr.rows;
        draw_detections(resized, *boxes_msg, sx, sy);
        auto out_msg = cvmat_to_rosimg(resized, img_msg->header);
        pub_.publish(out_msg);
    }

    std::string ns_, image_topic_, boxes_topic_, output_topic_;
    int out_w_{960}, out_h_{540};
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<BoxesMsg>> boxes_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<ImageMsg, BoxesMsg>>> sync_;
    image_transport::Publisher pub_;
};

class OverlayTwoCams : public rclcpp::Node {
public:
    OverlayTwoCams(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
    : Node("overlay_two_cams", opts)
    {
        rclcpp::NodeOptions child_opts;
        cam0_ = std::make_shared<OverlayForOneCam>("cam0Funai", child_opts);
        cam1_ = std::make_shared<OverlayForOneCam>("cam1Funai", child_opts);
        exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        exec_->add_node(cam0_);
        exec_->add_node(cam1_);
        worker_ = std::thread([this](){ exec_->spin(); });
        RCLCPP_INFO(get_logger(), "OverlayTwoCams started.");
    }
    ~OverlayTwoCams() override {
        if (exec_) exec_->cancel();
        if (worker_.joinable()) worker_.join();
    }
private:
    rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;
    std::shared_ptr<OverlayForOneCam> cam0_, cam1_;
    std::thread worker_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OverlayTwoCams>());
    rclcpp::shutdown();
    return 0;
}
