# Slot Detection Logic - Robot Control Main

## Tổng quan

Hệ thống sử dụng camera để detect vị trí các cartridge trên tray và xác định slot nào đang trống để robot đặt cartridge vào.

## Kiến trúc hệ thống

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────────┐
│ CSI Camera  │────▶│ YOLO HailoRT │────▶│ robot_logic_node_   │
│   (Cam1)    │     │  Detection   │     │       dual          │
└─────────────┘     └──────────────┘     └─────────────────────┘
                                                    │
                                                    ▼
                                         ┌──────────────────┐
                                         │  Slot Assignment │
                                         │  & Empty Slot    │
                                         │    Selection     │
                                         └──────────────────┘
```

## Object Classes

| Class ID | Tên | Mô tả |
|----------|-----|-------|
| 0 | Tray | Khay chứa - dùng để tính perspective transform |
| 1 | cartridge_ok | Cartridge đứng đúng hướng |
| 2 | misoriented | Cartridge đặt sai hướng |
| 3 | cartridge_fall | Cartridge bị ngã/rơi |

**Lưu ý**: Chỉ class 1, 2, 3 được dùng để assign vào slot. Class 0 (tray) chỉ dùng để tính homography.

## ROI (Region of Interest) - Slot Definition

### Canonical Tray Corners
Tọa độ 4 góc tray trong không gian chuẩn (canonical):
```cpp
canonical_tray_ = {
    {46.0f, 479.0f},   // Bottom-Left
    {109.0f, 52.0f},   // Top-Left
    {1147.0f, 61.0f},  // Top-Right
    {1128.0f, 520.0f}  // Bottom-Right
};
```

### 9 Canonical Slots
Mỗi slot được định nghĩa bởi 4 góc và orientation (H=Horizontal, V=Vertical):

| Slot | Góc TL | Góc TR | Góc BR | Góc BL | Orient |
|------|--------|--------|--------|--------|--------|
| 1 | (80,505) | (580,505) | (560,400) | (120,410) | H |
| 2 | (120,400) | (580,390) | (565,290) | (155,300) | H |
| 3 | (150,290) | (590,285) | (575,185) | (185,195) | H |
| 4 | (195,185) | (620,185) | (600,90) | (225,105) | H |
| 5 | (580,505) | (700,500) | (700,185) | (600,195) | V |
| 6 | (720,495) | (1105,485) | (1085,385) | (705,390) | H |
| 7 | (705,385) | (1080,380) | (1060,285) | (700,290) | H |
| 8 | (700,285) | (1035,280) | (1015,195) | (700,200) | H |
| 9 | (660,195) | (970,185) | (955,95) | (650,105) | H |

### Perspective Transform
Khi detect tray (class 0), hệ thống tính homography từ canonical → detected:
```cpp
cv::Mat H = cv::getPerspectiveTransform(canonical_tray_, detected_tray);
```

Sau đó warp các slot corners theo H và shrink về tâm (shrink ratio = 0.85):
```cpp
poly = shrinkPolygon(warpedPoly, slot_shrink_ratio_);
```

## Detection Pipeline

### 1. Filter Detections
```cpp
// Chỉ accept class 1, 2, 3 (cartridge types)
if (cid != 1 && cid != 2 && cid != 3)
    continue;

// Filter theo score threshold
if (score < score_thresh_o)  // default: 0.0
    continue;
```

### 2. NMS (Non-Maximum Suppression)
Loại bỏ các detection trùng lặp:
```cpp
nms_iou_thresh_ = 0.45;  // Default threshold
std::vector<int> keep = nmsGreedy(boxes, scores, nms_iou_thresh_);
```

### 3. Slot-Detection Matching
Với mỗi cặp (slot, detection):

#### Kiểm tra Inside
```cpp
cv::Point2f center = detection_center;
double in = cv::pointPolygonTest(slot_polygon, center, false);
bool inside = (in >= 0.0);  // True nếu center nằm trong polygon
```

#### Tính IoU (Intersection over Union)
```cpp
double iou = IoU(slot_aabb, detection_box);
```

#### Filter Logic
```cpp
// Skip nếu KHÔNG inside VÀ IoU thấp hơn threshold
if (!inside && iou < iou_thresh_)  // iou_thresh_ = 0.10
    continue;
```

**Giải thích**:
- Nếu **inside = true**: Luôn accept (center detection nằm trong slot)
- Nếu **inside = false**: Chỉ accept nếu IoU >= 0.10 (có overlap đáng kể)

#### Scoring
```cpp
double score = (inside ? 1.0 + iou : iou);
if (orient_ok) score += 0.1;  // Bonus nếu orientation khớp
```

### 4. Greedy Assignment
Candidates được sort theo score giảm dần, sau đó assign one-to-one:
```cpp
for (const auto &c : candidates) {
    if (slot_assigned[c.slot] != -1) continue;  // Slot đã có
    if (det_assigned[c.det] != -1) continue;    // Detection đã dùng
    slot_assigned[c.slot] = c.det;
    det_assigned[c.det] = c.slot;
}
```

## Debouncing (Temporal Stability)

### Instant vs Stable State
- **Instant state**: Trạng thái tức thời từ frame hiện tại
- **Stable state**: Trạng thái sau khi confirm qua nhiều frame

### State Machine
```
SlotStableState:
  - EMPTY: Slot trống
  - OCC_OK: Có cartridge đúng hướng
  - MIS: Có cartridge sai hướng
```

### Streak Counter
```cpp
confirm_frames_ = 2;  // Số frame cần để confirm

// Ví dụ: Slot 4 từ EMPTY → OCC_OK
Frame 1: instant[3] = OCC_OK → occ_ok_streak_[3] = 1
Frame 2: instant[3] = OCC_OK → occ_ok_streak_[3] = 2 → stable_state_[3] = OCC_OK
```

## Empty Slot Selection

### Instant Override (CRITICAL FIX)
```cpp
// Ưu tiên instant state để tránh va chạm
for (int s = 0; s < slots.size(); ++s) {
    // Nếu slot có object NGAY BÂY GIỜ → không empty
    if (instant[s] != SlotStableState::EMPTY) {
        if (stable_state_[s] == SlotStableState::MIS)
            mis_slots.push_back(s + 1);
        continue;  // Skip, không thêm vào empty_slots
    }
    
    // Chỉ slot INSTANT empty VÀ STABLE empty mới được publish
    if (stable_state_[s] == SlotStableState::EMPTY)
        empty_slots.push_back(s + 1);
}
```

### Contiguous Selection
Robot chỉ chọn slot empty liên tục từ slot 1:
```cpp
// Ví dụ: empty_slots = [4, 5, 6, 7, 8, 9]
// → Slot 1, 2, 3 đã đầy → Selected = 4

// Ví dụ: empty_slots = [2, 4, 5, 6, 7, 8, 9]
// → Slot 1 đầy, slot 2 trống, slot 3 đầy → INVALID (-1)
// → Có lỗ trống (slot 2) trước slot đầy (slot 3)
```

## Thông số hệ thống

### Detection Parameters
| Parameter | Default | Mô tả |
|-----------|---------|-------|
| `score_thresh_o` | 0.0 | Score tối thiểu để accept detection |
| `iou_thresh_` | 0.10 | IoU threshold cho matching |
| `nms_iou_thresh_` | 0.45 | IoU threshold cho NMS |
| `slot_shrink_ratio_` | 0.85 | Tỷ lệ shrink slot polygon |

### Debounce Parameters
| Parameter | Default | Mô tả |
|-----------|---------|-------|
| `confirm_frames_` | 2 | Số frame để confirm state change |

### Debug
| Parameter | Default | Mô tả |
|-----------|---------|-------|
| `debug_logs_` | true | Enable debug logging |
| `use_tray_from_detection_` | true | Dùng detected tray hay canonical |

## Troubleshooting

### Slot bị nhận nhầm là occupied
- Kiểm tra IoU threshold (`iou_thresh_`)
- Kiểm tra NMS threshold - có thể detection bị merge sai
- Xem log "DEBUG Slot4" để check IoU values

### Cartridge fall không được detect
- Đảm bảo class 3 được include trong filter
- Kiểm tra score của detection
- Xem slot polygon có cover đúng vùng không

### Robot chọn sai slot
- Kiểm tra `select_contiguous_empty()` logic
- Verify stable_state_ đã update đúng
- Check instant override có hoạt động

## Changelog

### 2026-02-06
- **FIX**: Loại bỏ class 0 (tray) khỏi slot detection
- **FIX**: Khôi phục IoU filtering logic từ backup
- **FIX**: Thêm instant state override để detect cartridge fall ngay lập tức
- **FIX**: Giảm IoU threshold từ 0.3 xuống 0.1 để detect partial overlap
