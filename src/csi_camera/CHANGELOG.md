# Changelog — csi_camera

## 2026-03-16 (v2) — Fix Runtime Camera Crash: PID Tracking + Targeted Kill

### Vấn đề
Camera crash khi **đang chạy** (runtime). Fix watchdog v1 vẫn bị lỗi vì
`stop_camera()` dùng `pkill rpicam-vid` — **kill cả cam0 lẫn cam1 cùng lúc**,
làm camera còn lại ngừng hoạt động.

### Thay đổi — Viết lại `csi_dual_camera_node.cpp`

#### Kiến trúc mới: `fork() + exec()` thay `popen()`
| | Cũ (`popen`) | Mới (`fork+exec`) |
|---|---|---|
| Track PID | ❌ Không biết PID | ✅ Track riêng mỗi camera |
| Kill khi crash | ❌ `pkill` cả 2 | ✅ `kill(pid, SIGKILL)` đúng process |
| Zombie reaping | ❌ Không | ✅ `waitpid()` |
| Khi cam0 crash | cam1 bị kill oan | cam1 vẫn chạy bình thường |

#### `CamProcess` struct — lưu `pid` + `fd` + `fp` riêng từng camera
```cpp
struct CamProcess { pid_t pid; int fd; FILE* fp; };
```

#### `launch_rpicam(cam_id, ...)` — fork/exec với pipe
- `fork()` → child: `dup2(pipe_write, stdout)` → `execlp("rpicam-vid", ...)`
- Parent nhận `pid` và `read fd` → wrap thành `FILE*` cho `fread()`

#### `kill_cam_process(CamProcess&)` — kill đúng process
```cpp
kill(cp.pid, SIGKILL);   // chỉ kill process này
waitpid(cp.pid, ...);    // reap zombie ngay lập tức
fclose(cp.fp);           // giải phóng fd
```

#### `capture_loop(cam_id)` — 1 hàm generic cho cả 2 camera
- `FAIL_THRESHOLD = fps` (= 30 frames ≈ 1 giây) → trigger reconnect
- Reconnect: `kill_cam_process(cp)` → 600ms wait → `launch_rpicam(cam_id)`
- Các camera **độc lập**: cam0 reconnect không ảnh hưởng cam1

#### Cải tiến khác
- `yuv_frame_size_` tính trước `start_camera()` (tránh log `0 bytes`)
- 400ms delay giữa CAM0 và CAM1 tránh I2C/CSI bus contention
- Hợp nhất `capture_loop_cam0` + `capture_loop_cam1` thành 1 hàm

---

## 2026-03-16 (v1) — Fix cam0 Disconnection + Watchdog Reconnect

### Vấn đề
- `cam0` (CSI camera 0) thường xuyên mất kết nối trong khi `cam1` vẫn chạy bình thường, gây chương trình robot hoạt động sai.

### Nguyên nhân gốc rễ
1. **Không có cơ chế reconnect**: Khi `rpicam-vid --camera 0` bị crash/ngắt, thread capture chỉ `sleep(10ms)` và tiếp tục đọc mãi mãi mà không bao giờ restart process.
2. **`select()` timeout quá ngắn (5ms)**: Ở 30fps mỗi frame cần ~33ms, khiến cam0 luôn bị false-disconnect.

### Thay đổi — `src/csi_dual_camera_node.cpp`

#### Watchdog Reconnect (capture_loop_cam0 / capture_loop_cam1)
- Thêm bộ đếm `consecutive_fails` cho từng camera.
- Khi đếm đạt **30 lần liên tiếp** (~1 giây @ 30fps):
  1. Gọi `stop_camera()` để đóng pipe cũ.
  2. Chờ 500ms để hardware ổn định.
  3. Gọi `start_camera()` để khởi động lại `rpicam-vid`.
  4. Nếu thất bại, chờ thêm 2s trước khi thử lần sau.
- Reset `consecutive_fails = 0` mỗi khi đọc được frame hợp lệ.

#### Tăng `select()` Timeout
- Thay đổi từ `5000 µs` (5ms) → `40000 µs` (40ms).
- 40ms ≥ một frame @ 30fps (33ms), loại bỏ false timeout.

```
// Trước:
timeout.tv_usec = 5000;  // 5ms timeout

// Sau:
timeout.tv_usec = 40000; // 40ms timeout (≥ 1 frame @ 30fps = 33ms)
```
