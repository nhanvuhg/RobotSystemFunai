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
#include <memory>
#include <string>

using vision_msgs::msg::Detection2DArray;

// -pi- CÁC HÀM TIỆN ÍCH (HELPER FUNCTIONS) - GIỮ NGUYÊN ---

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
        
        // Vẽ khung chữ nhật (Màu xanh lá)
        cv::rectangle(img, cv::Rect(cv::Point(x1,y1), cv::Point(x2,y2)), cv::Scalar(0,255,0), 2);
        
        std::string label = "";
        if (!det.results.empty()) {
            const auto &h = det.results[0].hypothesis;
            label = h.class_id + " " + cv::format("(%.2f)", h.score);
        }
        if (!label.empty()) {
            int base = 0; cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
            int tx = std::max(0, x1), ty = std::max(ts.height+4, y1);
            cv::rectangle(img, cv::Rect(tx, ty-ts.height-4, ts.width+6, ts.height+6), cv::Scalar(0,255,0), cv::FILLED);
            cv::putText(img, label, cv::Point(tx+3, ty-3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
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
    } else if (enc == "yuv422_yuy2" || enc == "yuyv") {
        cv::Mat ref_yuyv(msg.height, msg.width, CV_8UC2, const_cast<unsigned char*>(msg.data.data()), msg.step);
        cv::cvtColor(ref_yuyv, out_bgr, cv::COLOR_YUV2BGR_YUY2);
        return true;
    } 
    // Thêm các định dạng khác nếu cần
    return false;
}

static inline sensor_msgs::msg::Image cvmat_to_rosimg(const cv::Mat &bgr, const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::Image out;
    out.header = header;
    out.height = bgr.rows;
    out.width  = bgr.cols;
    out.encoding = "bgr8";
    out.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * bgr.channels());
    out.data.resize(out.step * out.height);
    std::memcpy(out.data.data(), bgr.data, out.data.size());
    return out;
}

// --- CLASS QUẢN LÝ 1 KÊNH (HELPER CLASS, KHÔNG PHẢI NODE) ---
class ChannelVisualizer {
public:
    using ImageMsg = sensor_msgs::msg::Image;
    using BoxesMsg = Detection2DArray;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, BoxesMsg>;

    ChannelVisualizer(rclcpp::Node* node, const std::string& prefix) 
        : node_(node), prefix_(prefix)
    {
        // Khai báo tham số dạng "cam0.image_topic", "cam0.boxes_topic"
        // Điều này khớp hoàn toàn với file launch của bạn
        std::string img_topic = node_->declare_parameter<std::string>(prefix_ + ".image_topic", "/" + prefix_ + "/image_raw");
        std::string box_topic = node_->declare_parameter<std::string>(prefix_ + ".boxes_topic", "/" + prefix_ + "/yolo/bounding_boxes");
        std::string out_topic = node_->declare_parameter<std::string>(prefix_ + ".output_topic", "/" + prefix_ + "/image_overlay");
        
        out_w_ = node_->declare_parameter<int>(prefix_ + ".output_width", 640);
        out_h_ = node_->declare_parameter<int>(prefix_ + ".output_height", 480);

        // Khởi tạo Subscribers & Publisher
        image_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(node_, img_topic);
        boxes_sub_ = std::make_shared<message_filters::Subscriber<BoxesMsg>>(node_, box_topic);
        
        // Synchronizer: Đồng bộ ảnh và box
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_sub_, *boxes_sub_);
        sync_->registerCallback(std::bind(&ChannelVisualizer::callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_ = image_transport::create_publisher(node_, out_topic);

        RCLCPP_INFO(node_->get_logger(), "Configured Overlay [%s]: Img='%s', Box='%s' -> Out='%s'", 
            prefix_.c_str(), img_topic.c_str(), box_topic.c_str(), out_topic.c_str());
    }

private:
    void callback(const ImageMsg::ConstSharedPtr &img_msg, const BoxesMsg::ConstSharedPtr &boxes_msg)
    {
        cv::Mat bgr;
        if (!rosimg_to_cvmat(*img_msg, bgr)) {
            // Chỉ warn 1 lần mỗi giây để đỡ spam log
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                "[%s] Unsupported encoding: %s", prefix_.c_str(), img_msg->encoding.c_str());
            return;
        }

        // Resize nếu cần
        cv::Mat resized;
        if (bgr.cols != out_w_ || bgr.rows != out_h_) {
            cv::resize(bgr, resized, cv::Size(out_w_, out_h_));
        } else {
            resized = bgr; // No copy
        }

        // Tính tỉ lệ để vẽ box cho đúng vị trí sau khi resize
        double sx = static_cast<double>(out_w_) / img_msg->width;
        double sy = static_cast<double>(out_h_) / img_msg->height;

        // Vẽ
        draw_detections(resized, *boxes_msg, sx, sy);

        // Publish
        auto out_msg = cvmat_to_rosimg(resized, img_msg->header);
        pub_.publish(out_msg);
    }

    rclcpp::Node* node_;
    std::string prefix_;
    int out_w_, out_h_;
    
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<BoxesMsg>> boxes_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    image_transport::Publisher pub_;
};

// --- NODE CHÍNH (QUẢN LÝ CẢ 2 KÊNH) ---
class DualCamOverlayNode : public rclcpp::Node {
public:
    DualCamOverlayNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
    : Node("overlay_dual_cam", opts)
    {
        // Tạo 2 visualizer cho 2 prefix "cam0" và "cam1"
        // Nó sẽ tự động đọc tham số cam0.image_topic, cam1.image_topic...
        vis_cam0_ = std::make_shared<ChannelVisualizer>(this, "cam0");
        vis_cam1_ = std::make_shared<ChannelVisualizer>(this, "cam1");
        
        RCLCPP_INFO(get_logger(), "Dual Camera Overlay Node Started.");
    }

private:
    std::shared_ptr<ChannelVisualizer> vis_cam0_;
    std::shared_ptr<ChannelVisualizer> vis_cam1_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualCamOverlayNode>());
    rclcpp::shutdown();
    return 0;
}