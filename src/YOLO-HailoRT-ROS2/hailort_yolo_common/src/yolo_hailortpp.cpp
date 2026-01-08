#include "hailort_yolo_common/hailo_nms_decode.hpp"
#include "hailort_yolo_common/yolo_hailortpp.hpp"
#include "hailort_yolo_common/labels/coco_eighty.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>

#include <regex>

// void filter(HailoROIPtr roi, void * params_void_ptr)
// {
//   if (!roi->has_tensors()) {
//     return;
//   }
//   std::map<uint8_t, std::string> labels_map;
//   labels_map = common::coco_eighty;

//   for (auto tensor : roi->get_tensors()) {
//     if (std::regex_search(tensor->name(), std::regex("nms"))) {
//       auto post = HailoNMSDecode(tensor, labels_map);
//       auto detections = post.decode<float32_t, common::hailo_bbox_float32_t>();
//       hailo_common::add_detections(roi, detections);
//     }
//   }
// }

// (A) Dump danh sách tensor để debug khi cần
static void dump_tensors_for_debug(const HailoROIPtr& roi)
{
    std::stringstream ss;
    ss << "[YOLO-HailoRT] Available tensors:\n";
    const auto &tensors = roi->get_tensors(); // vector<HailoTensorPtr>
    for (const auto &t : tensors) {
        const auto &info = t->vstream_info();
        ss << "  - " << t->name()
           << " | order=" << static_cast<int>(info.format.order)
           << " | HxW="   << info.shape.height << "x" << info.shape.width
           << " | features=" << info.shape.features << "\n";
    }
    std::cerr << ss.str();
}

// (B) Chọn tensor NMS theo thứ tự ưu tiên:
// 1) ENV HAILO_NMS_TENSOR (nếu set) → 2) tên gợi ý đúng với HEF bạn có → 3) scan tên chứa "nms"/"postprocess" & đúng format → 4) scan đúng format NMS
static HailoTensorPtr select_nms_tensor(const HailoROIPtr& roi)
{
    const auto &tensors = roi->get_tensors(); // vector<HailoTensorPtr>

    // 1) ENV
    std::string nms_name;
    if (const char *env = std::getenv("HAILO_NMS_TENSOR")) {
        nms_name = env;
    }
    // 2) Gợi ý tên vstream đúng với file .hef của bạn (đã parse-hef)
    if (nms_name.empty()) {
        nms_name = "yolov8s/yolov8_nms_postprocess";
    }
    // Thử khớp tên trực tiếp
    for (const auto &t : tensors) {
        if (t->name() == nms_name) return t;
    }

    // 3) Fallback: tên có "nms"/"postprocess" & đúng format NMS
    for (const auto &t : tensors) {
        const std::string &nm = t->name();
        bool looks_like_nms = (nm.find("nms") != std::string::npos)
                           || (nm.find("postprocess") != std::string::npos);
        if (looks_like_nms) {
            if (t->vstream_info().format.order == HAILO_FORMAT_ORDER_HAILO_NMS
                || static_cast<int>(t->vstream_info().format.order) == 22) { // 22: BY_CLASS/4.20
                return t;
            }
        }
    }

    // 4) Fallback cuối: chỉ cần đúng format NMS
    for (const auto &t : tensors) {
        if (t->vstream_info().format.order == HAILO_FORMAT_ORDER_HAILO_NMS
            || static_cast<int>(t->vstream_info().format.order) == 22) {
            return t;
        }
    }

    return nullptr;
}

// =================== HÀM FILTER CHÍNH ===================
void filter(HailoROIPtr roi, void *params_void_ptr)
{
    if (!roi || !roi->has_tensors())
        return;

    // labels_map có thể thay bằng labels 4 lớp của bạn; coco_eighty dùng tạm cũng không cản detection
    std::map<uint8_t, std::string> labels_map = common::coco_eighty;

    // 1) Lấy đúng tensor NMS
    auto nms_tensor = select_nms_tensor(roi);
    if (!nms_tensor) {
        std::cerr << "[YOLO-HailoRT] No NMS tensor selected. "
                     "Set env HAILO_NMS_TENSOR or adjust select_nms_tensor().\n";
        dump_tensors_for_debug(roi);
        return; // không crash container
    }

    // 2) Gọi decode CHUẨN bằng HailoNMSDecode (sau khi đã nới điều kiện NMS ở Bước 0)
    try {
        auto post = HailoNMSDecode(nms_tensor, labels_map);

        // Giữ float để tránh lỗi template float32_t tùy SDK
        auto detections = post.decode<float, common::hailo_bbox_float32_t>();

        // 3) Xuất detection → đúng kiểu HailoDetection mà add_detections yêu cầu
        hailo_common::add_detections(roi, detections);

        // (tuỳ chọn) In số lượng cho dễ debug
        std::cerr << "[YOLO-HailoRT] decoded detections: " << detections.size() << "\n";
    }
    catch (const std::exception &e) {
        std::cerr << "[YOLO-HailoRT] HailoNMSDecode failed: " << e.what() << "\n";
        dump_tensors_for_debug(roi);
        // Không throw để container còn sống
    }
}
