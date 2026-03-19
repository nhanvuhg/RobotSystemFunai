# Hướng dẫn Deploy YOLO HailoRT trên ROS 2 (Multi-Camera / Multi-Model)

Tài liệu này ghi chú lại các lỗi kiến trúc nghiêm trọng và cách thiết lập chuẩn khi deploy **nhiều camera cùng chạy suy luận YOLO trên một chip NPU Hailo-8 duy nhất** trong môi trường ROS 2.

---

## 1. Lỗi chia sẻ phần cứng Hailo (Physical Device Exhaustion)

**Triệu chứng:**
Khi chạy 2 node YOLO (`yolo_cam0`, `yolo_cam1`) ở 2 tiến trình (process/container) khác nhau, node thứ hai sẽ báo lỗi `HAILO_OUT_OF_PHYSICAL_DEVICES` (Status 74).

**Nguyên nhân:**
HailoRT `VDevice` quản lý phần cứng PCIe. Mặc định nó độc chiếm (exclusive lock) thiết bị cho tiến trình gọi nó. Không thể có 2 OS process cùng truy cập 1 chip Hailo-8.

**Giải pháp (Bắt buộc):**
- Gộp chung tất cả các node YOLO vào **1 ROS 2 ComposableNodeContainer duy nhất**.
- Khởi tạo thư viện dùng chung một con trỏ `std::shared_ptr<hailort::VDevice>`.

---

## 2. Kích hoạt bộ lập lịch phần cứng (Multi-Network Scheduler)

**Triệu chứng:**
Ngay cả khi đưa vào 1 container, nếu gọi cấu hình (configure) nhiều file HEF khác nhau, hệ thống có thể bị crash, hoặc báo `Device in use`, `Driver timeout`.

**Nguyên nhân:**
Để chip Hailo-8 gánh cùng lúc nhiều mạng Neural (HEF) khác nhau từ nhiều camera, tính năng lập lịch (Scheduler) của NPU phải được bật ở mức Driver.

**Giải pháp (C++ API):**
Trong hàm khởi tạo `VDevice`, bắt buộc truyền tham số `HAILO_SCHEDULING_ALGORITHM_ROUND_ROBIN`:
```cpp
hailo_vdevice_params_t params;
hailo_init_vdevice_params(&params);
params.scheduling_algorithm = HAILO_SCHEDULING_ALGORITHM_ROUND_ROBIN;
auto vdev_exp = hailort::VDevice::create(params);
```

---

## 3. Lỗi nghẽn cổ chai ComponentManager (C++ Deadlock)

**Triệu chứng:**
ROS 2 Node Container bị treo (hang) ngay khi mới khởi động, một hoặc toàn bộ camera không xuất hiện dòng chữ `[model loaded]`, không có bất kỳ báo lỗi nào. `ros2 node list` thấy node bị mất tích.

**Nguyên nhân:**
Khi ROS 2 `launch_ros` đẩy yêu cầu khởi tạo hai node YOLO cùng một lúc vào Container:
1. `MultiThreadedExecutor` sẽ gọi hàm khởi tạo C++ (constructor) của cả 2 node **song song trên 2 luồng**.
2. Cả 2 luồng cùng gọi hàm `vdevice.configure(...)` xuống Driver PCIe đồng thời.
3. Driver PCIe của Hailo KHÔNG thread-safe ở bước Flash cấu hình mạng, dẫn đến khóa cứng toàn bộ thiết bị (Deadlock).

**Giải pháp:**
Phải thiết lập một `std::mutex` tĩnh tại hàm khởi tạo thiết bị để ép các luồng cấu hình phần cứng **lần lượt**:
```cpp
static std::mutex init_mtx;
std::lock_guard<std::mutex> lock(init_mtx);
// Thực hiện vdevice.configure() và VStreamsBuilder::create_vstreams()
```

---

## 4. Xung đột mạng ROS 2 (Service Cross-talk)

**Triệu chứng:**
Người dùng "tắt đi bật lại" script khởi động nhưng hệ thống bị kẹt. Node YOLO cam0 không xuất ra ảnh, nhưng `ros2 node list -a` lại báo có 2 node `/yolo_container` trên mạng.

**Nguyên nhân:**
- Theo mặc định, ROS 2 FastDDS gộp tất cả các thiết bị cùng chung mạng LAN (cùng `ROS_DOMAIN_ID`).
- Nếu có 1 máy tính/robot khác cũng đang chạy node có tên `/yolo_container` (do copy code cùng nguồn), hàm gọi khởi tạo từ `launch_ros` (Service Call `LoadNode`) sẽ **gửi nhầm sang máy tính của robot kia**, khiến robot hiện tại bị bỏ qua lệnh khởi tạo.

**Giải pháp bền vững:**
Bảo vệ tính duy nhất của Node container trong file `launch.py`:
```python
container = ComposableNodeContainer(
    name='yolo_container_funai',       # Tên phải ĐỘC NHẤT cho mỗi robot
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt', # BẮT BUỘC dùng bản multi-thread (_mt)
    # ...
)
```

**Sử dụng `component_container_mt`:**
Bắt buộc dùng `executable='component_container_mt'` thay vì `component_container`. Nếu dùng bản mặc định (single-threaded), tiến trình xử lý ảnh của `cam1` sẽ chặn luồng duy nhất, khiến luồng nhận ảnh của `cam0` không bao giờ được cấp CPU (Data Starvation).
