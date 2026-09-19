# Chạy trên máy dwduong74 — PX4 v1.17.0

Workspace: `/home/dwduong74/drone/src/dwd_uav`, Ubuntu 22.04 amd64, ROS 2 Humble,
Gazebo Harmonic. PX4 dùng từ `/home/dwduong74/PX4-Autopilot`.

`px4_msgs` được khóa tại commit `86d8239e962f6939e05c3737784f60c02fa884db`
(branch `release/1.17`). Đã đối chiếu toàn bộ 12 message mà adapter sử dụng với
source PX4 trên máy: khớp hoàn toàn. Bộ message riêng nằm ở `install/px4_117`;
không dùng bản message v1.15 của workspace ngoài.

## Mở mô phỏng và các node giao hàng

```bash
cd /home/dwduong74/drone/src/dwd_uav
bash scripts/run_native.sh sim
```

Trên máy 4 nhân này, dùng chế độ giảm tải để PX4 giữ ổn định nhịp IMU khi bay:

```bash
DELIVERY_HEADLESS=1 bash scripts/run_native.sh sim
```

Chế độ này vẫn chạy Gazebo physics, camera và lidar nhưng không mở cửa sổ 3D.
Chế độ có GUI có thể làm EKF mất nhịp nếu máy đang chạy nhiều ứng dụng.

Lệnh chạy Gazebo (GUI và server), PX4 SITL, Micro XRCE-DDS Agent v2.4.2 và ROS
bridge/các node `vision`, `px4_adapter`, `payload`, `mission`. X500 có camera
hướng xuống và hai bãi đáp ArUco. Payload dùng backend giả lập.

Không tự bắt đầu nhiệm vụ. Drone đứng trên bãi đáp cho đến khi gửi goal.
Kiểm tra trạng thái trong terminal khác:

```bash
cd /home/dwduong74/drone/src/dwd_uav
source scripts/env_native.sh
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=0
ros2 topic echo /delivery/flight_state --once
ros2 topic echo /delivery/vision_ready --once
ros2 topic echo /delivery/payload_state --once
```

Khi flight/payload healthy, camera ready, range valid và PX4 đã đáp ứng các
arming check, gửi nhiệm vụ theo tọa độ cấu hình sinh từ world:

```bash
ros2 service call /delivery/start std_srvs/srv/Trigger '{}'
```

Giữ QGroundControl kết nối trước khi thực hiện bay. Không tắt arming check để
che lỗi cấu hình. Một lần khởi động và nhận telemetry không xác nhận chuyến bay
khứ hồi hay độ chính xác hạ cánh; xem [các kiểm chứng còn lại](validation.md).

Ctrl+C dừng toàn bộ stack. Log: `log/gazebo/{server,gui,agent,px4,ros}.log`;
tham số và ULog PX4 riêng ở `build/gazebo_runtime`. Gazebo dùng partition
`dwd_uav_native`, ROS domain mặc định 74, Agent UDP port 8894. Có thể chọn domain
bằng `ROS_DOMAIN_ID=75 bash scripts/run_native.sh sim`; terminal theo dõi dùng cùng domain.
Có thể đặt `DELIVERY_PX4_DIR` tới checkout v1.17.0 khác đã build.

## Build và kiểm thử

Đã kiểm chứng: build native thành công; **44/44 test đạt** sau khi thêm chế độ Bảng C.
Gazebo/PX4 thực xuất telemetry healthy, range hợp lệ, camera/TF ready và payload
giả lập healthy. Chưa gửi goal/arm trong kiểm tra này; chuyến bay khứ hồi vẫn cần nghiệm thu.

```bash
cd /home/dwduong74/drone/src/dwd_uav
bash scripts/run_native.sh deps
bash scripts/run_native.sh build
bash scripts/run_native.sh test
```

`deps` tải/build message đã khóa và Agent v2.4.2. Build dependency dùng `/tmp`
để giảm tải ổ `/home`, install nằm trong repo. Build các node dùng
`build/native`, `install/native`, `log/native`. Dừng sim/demo trước khi test.
Agent v3.0.1 trên hệ thống được giữ nguyên; script ưu tiên Agent v2.4.2 trong
`install/agent_242` và Fast DDS của Humble.

Camera mô phỏng dùng 320×240 ở 10 Hz để giảm tải trên máy này.
Camera/Image, CameraInfo và LaserScan giữ thời điểm thu nhận trong Gazebo,
được chuyển sang đồng hồ DDS bằng `TimesyncStatus.estimated_offset` của PX4
qua node `sim_camera_clock`; frame trùng hoặc cũ bị loại để không che lỗi đóng băng ảnh.

Với v1.17, adapter kiểm tra `estimator_status_flags.cs_yaw_align` để yêu cầu yaw
đã căn chỉnh ban đầu. Cờ `heading_good_for_control` chỉ hoàn tất căn chỉnh từ kế
sau khi bay, nên không dùng nó để chặn cất cánh; PX4 vẫn áp dụng arming check.
Camera/lidar được đặt tương đối với `base_link`; scan GPU ba tia được lấy tia giữa
thành LaserScan một tia cho adapter.

Adapter tự thêm hậu tố topic theo `MESSAGE_VERSION`:
`/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`,
`/fmu/out/battery_status_v1`. Message version 0 không có hậu tố.

`env_native.sh` ưu tiên NumPy 1.21.5 và OpenCV 4.5.4 của Ubuntu, bỏ đường dẫn
Python/ROS overlay cũ. Không cần thay các bản pip của ứng dụng khác trên máy.

## Chỉ chạy các node ROS

```bash
bash scripts/run_native.sh demo
```

Chế độ `demo` chỉ khởi động các node ROS với điều khiển bay tắt, payload giả lập;
không mở Gazebo. Thiếu camera/telemetry thì trạng thái chưa sẵn sàng là bình thường.

## Phần cứng

Hiệu chuẩn camera, cấu hình GPS/serial và flight controller PX4 v1.17 theo README:

```bash
bash scripts/run_native.sh hardware /absolute/path/calibrated_site.yaml
```

Hardware dùng ROS domain do terminal cung cấp (mặc định 0). Agent và các node
phải cùng domain. Gói thay đổi này không nạp firmware lên flight controller.
