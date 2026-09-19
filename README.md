# dwd_uav — ROS 2 Humble, PX4 v1.17 và Gazebo

Repo triển khai giao hàng khứ hồi bằng ROS 2 Action: **cất cánh → tới GPS → tìm
ArUco → đáp → nhả hàng → cất cánh lại → quay về → đáp**. Pi 4 là máy tính đồng
hành; flight controller riêng chạy PX4 và điều khiển động cơ bay.

Trên máy phát triển đã cấu hình, xem [hướng dẫn riêng cho máy này](docs/setup_this_machine.md).
`bash scripts/run_native.sh sim` mở bài giao một kiện; `table-c` mở sa hình ba kiện;
`demo` chỉ chạy các node ROS.

**Đã kiểm chứng:** build native ROS 2 Humble với PX4 v1.17, 44/44 kiểm thử đạt;
Gazebo mở, ROS nhận telemetry/camera/TF/range hợp lệ, và UAV hạ tại pad B với
sai số ground truth 0,021 m rồi nhả payload.
**Chưa nghiệm thu:** chuyến bay ba kiện liên tiếp, firmware trên cơ cấu thật và
benchmark Pi 4. Xem [báo cáo kiểm chứng](docs/validation.md) trước khi thử phần cứng.

## Bắt đầu nhanh trên máy đã cài dependency

```bash
cd /home/dwduong74/drone/src/dwd_uav
bash scripts/run_native.sh build
bash scripts/run_native.sh test
bash scripts/run_native.sh table-c
```

Giữ terminal mô phỏng chạy. Trong terminal thứ hai, source môi trường rồi gửi
mission theo phần dưới. Máy mới cần làm các bước cài ROS/PX4/Gazebo ở mục 4–6
trước khi chạy `build`. Script `deps` cài Agent và build `px4_msgs` đã khóa phiên bản.

## Chạy bài thi Bảng C ba kiện

Chế độ này sinh sa hình gồm bốn dãy nhà, hai cổng lệch nhau, hầm 3×3×3 m,
ba bệ giao và scanner trạm gắp mô phỏng. Giữ terminal đầu chạy:

```bash
cd /home/dwduong74/drone/src/dwd_uav
bash scripts/run_native.sh table-c
```

Sau khi PX4 báo sẵn sàng, mở terminal thứ hai:

```bash
cd /home/dwduong74/drone/src/dwd_uav
source scripts/env_native.sh
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=0
ros2 topic echo /delivery/flight_state --once
ros2 topic echo /delivery/vision_ready --once
ros2 topic echo /delivery/payload_state --once
ros2 service call /competition/start std_srvs/srv/Trigger '{}'
ros2 topic echo /competition/status
```

Chỉ gửi `/competition/start` khi `flight_state.healthy`, `vision_ready.data`,
`payload_state.healthy` và `payload_state.grab_ready` đều là `true`.
Service trả `success=true` nghĩa là goal đã được gửi; kết quả cuối nằm ở
`/competition/status`. `state`: `1=SCAN_PACKAGE`, `2=GRAB_PACKAGE`,
`3=DELIVER`, `5=HOLD`, `6=FINISHED`, `7=FAILED`.

Điều khiển giám sát dùng `1=PAUSE`, `2=RESUME`, `3=RTL`, `4=LAND`,
`5=ABORT_SAFE`:

```bash
ros2 service call /competition/control \
    delivery_interfaces/srv/CompetitionControl '{command: 1}'
```

Toàn bộ gốc GPS, ba điểm gắp, ba điểm trả, marker, tuyến, geofence và vật cản
nằm trong một file `config/table_c.yaml`. Tọa độ sa hình dùng ENU theo mét:
`[x Đông, y Bắc, z cao]`. `position` của điểm trả tự được đổi sang GPS; không
cần nhập lại latitude/longitude.

Có thể sao chép file này, sửa cấu hình và chạy bản riêng:

```bash
cp config/table_c.yaml config/my_course.yaml
bash scripts/run_native.sh table-c config/my_course.yaml
```

Các `route_nodes` phải nằm trong `geofence`; marker phải duy nhất. Vật cản hỗ
trợ `type: box`, `type: gate` và `type: tunnel`. Cả Gazebo lẫn supervisor đọc
cùng file được truyền vào, nên tọa độ mô phỏng và tọa độ mission luôn đồng bộ.
Trên phần cứng, thay scanner mô phỏng bằng camera xuất `/delivery/fiducials` và giữ
`sim_package_station` tắt. Công tắc kill động cơ phải cấu hình trực tiếp trên
RC/PX4, không đi qua ROS.

Mô phỏng đã xác nhận khởi động stack, quét/gắp giả lập và vào chặng bay đầu.
Chưa đạt tiêu chí nghiệm thu ba kiện liên tiếp; xem [validation](docs/validation.md).

## Mục lục

1. [Chọn lộ trình và phiên bản](#1-chọn-lộ-trình-và-phiên-bản)
2. [Chuẩn bị mã nguồn](#2-chuẩn-bị-mã-nguồn)
3. [Kiểm tra nhanh bằng Docker](#3-kiểm-tra-nhanh-bằng-docker)
4. [Cài ROS 2 và build trên Ubuntu](#4-cài-ros-2-và-build-trên-ubuntu)
5. [Cài Micro XRCE-DDS Agent](#5-cài-micro-xrce-dds-agent)
6. [Setup PX4 và mô phỏng](#6-setup-px4-và-mô-phỏng)
7. [Setup Raspberry Pi 4 và thiết bị](#7-setup-raspberry-pi-4-và-thiết-bị)
8. [Cấu hình và khởi chạy trên Pi](#8-cấu-hình-và-khởi-chạy-trên-pi)
9. [Gửi nhiệm vụ, theo dõi và hủy](#9-gửi-nhiệm-vụ-theo-dõi-và-hủy)
10. [Ghi log và kiểm tra hiệu năng](#10-ghi-log-và-kiểm-tra-hiệu-năng)
11. [Lỗi thường gặp](#11-lỗi-thường-gặp)
12. [Cấu trúc repo và tài liệu](#12-cấu-trúc-repo-và-tài-liệu)

## 1. Chọn lộ trình và phiên bản

| Mục đích | Thực hiện |
|---|---|
| Chỉ kiểm tra phần mềm, chưa có drone | Mục 2–3: Docker, không cần camera/FC/MCU |
| Phát triển ROS và chạy SITL trên PC | Mục 2, 4–6, 9–10 |
| Triển khai trên Pi 4 | Mục 2, 4–5, phần firmware ở mục 6, mục 7–10 |

| Thành phần | Phiên bản/cấu hình của repo |
|---|---|
| Máy phát triển native | Ubuntu 22.04 amd64 |
| Raspberry Pi 4 | Ubuntu Server 22.04 **arm64**, nguồn đủ tải và tản nhiệt |
| ROS | ROS 2 Humble; Python 3 hệ thống Ubuntu |
| PX4 | **v1.17.0**, có bổ sung DDS topic land detector |
| `px4_msgs` | Commit `86d8239e962f6939e05c3737784f60c02fa884db` từ `release/1.17` |
| Micro XRCE-DDS Agent | **v2.4.2** |
| Mô phỏng của repo | Gazebo Harmonic + bridge Humble tương ứng, chạy trên PC |
| Camera | USB UVC hướng xuống hoặc driver tương đương xuất Image + CameraInfo |
| Cảm biến/cơ cấu thật | GPS, rangefinder, MCU USB, servo kẹp, cảm biến hàng và kẹp |

Không cần build PX4 hoặc cài Gazebo trên Pi để chạy các node ROS. Bộ điều khiển
bay được build trên máy phát triển rồi nạp lên flight controller.

## 2. Chuẩn bị mã nguồn

Clone repo từ GitHub hoặc sao chép mã nguồn; không sao chép các thư mục sinh ra
`build`, `install`, `log`, `__pycache__` giữa các máy. Đặc biệt, không dùng bản
`install` build trên amd64 cho Pi arm64.

```bash
git clone https://github.com/dwduong74/dwd_uav.git
cd dwd_uav
```

Các lệnh Bash bên dưới giả sử thư mục nằm tại:

```bash
cd "$HOME/dwd_uav"
```

Nếu dùng WSL với workspace Windows hiện tại, có thể vào trực tiếp:

```bash
cd /mnt/c/project/UAV_HCM/dwd_uav
```

Giữ đường dẫn thực tế của bạn trong các terminal. Trên WSL, build trong filesystem
Linux thường nhanh hơn build qua `/mnt/c`. Đây là các lệnh **Bash trên Linux/WSL**,
không dán nguyên vào PowerShell. Chưa có URL Git chính thức để dùng `git clone`.

## 3. Kiểm tra nhanh bằng Docker

Cần Docker Engine đang chạy và tài khoản có quyền sử dụng Docker. Chạy tại root repo:

```bash
docker info
docker build -t delivery-test -f Dockerfile.test .
docker run --rm -v "$PWD:/workspace" delivery-test bash scripts/test_container.sh
```

Kết quả mong đợi: hai package `delivery_interfaces`, `delivery_ros` build thành
công và cuối log có `Ran 44 tests ... OK`. Build đầu tải image và biên dịch
`px4_msgs`, có thể mất nhiều phút.

Container dùng `build/container`, `install/container` và `log/container`.
Image kiểm thử chứa ROS, OpenCV và message PX4; **không chứa PX4 SITL/Gazebo**.
Kiểm thử chu trình dùng mô hình động học giả lập, không xác nhận chất lượng bay.

Nếu Docker báo permission denied với socket trên Linux, cấu hình quyền Docker
cho tài khoản theo hướng dẫn Docker của máy, rồi đăng nhập lại.

## 4. Cài ROS 2 và build trên Ubuntu

### 4.1. Kiểm tra hệ điều hành

```bash
cat /etc/os-release
dpkg --print-architecture
```

Native setup yêu cầu Ubuntu 22.04; Pi phải trả về `arm64`. Nếu máy Windows/WSL
đang dùng Ubuntu khác, chọn container ở mục 3 hoặc môi trường Ubuntu 22.04 riêng.

### 4.2. Cài ROS 2 Humble

Với máy mới, làm phần **Set locale** và **Setup Sources** trong
[hướng dẫn ROS 2 Humble chính thức](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
để bật Universe, locale UTF-8 và repository ROS. Sau đó:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

Kết quả phải là `humble`. Trên PC cần RViz có thể cài thêm `ros-humble-desktop`;
Pi chạy headless chỉ cần `ros-humble-ros-base`. Cách thêm apt repository cập nhật
được duy trì trong [nguồn tài liệu ROS](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst).

### 4.3. Cài dependency của repo

```bash
sudo apt install git build-essential cmake \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-opencv python3-numpy python3-yaml python3-serial \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-diagnostic-msgs \
  ros-humble-rosbag2-storage-default-plugins
```

Dùng Python/OpenCV của Ubuntu trong môi trường ROS. Không kích hoạt Conda hoặc
venv có OpenCV khác khi build/chạy `cv_bridge`.

### 4.4. Import `px4_msgs` và build

Tại root repo, trong terminal đã source Humble:

```bash
vcs import src < dependencies.repos
# Chỉ chạy lệnh sau nếu rosdep chưa được khởi tạo trên máy:
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --executor sequential
source install/setup.bash
```

Nếu `rosdep init` báo đã có cấu hình, bỏ qua bước init và tiếp tục `rosdep update`.
Không import đè thủ công một bản `px4_msgs main`; manifest đã khóa commit phù hợp.
Nếu `src/px4_msgs` đã tồn tại, kiểm tra thay vì xóa thư mục:

```bash
git -C src/px4_msgs rev-parse HEAD
```

Kiểm tra kết quả:

```bash
ros2 pkg prefix delivery_ros
ros2 interface show delivery_interfaces/msg/MissionStatus
ros2 interface show delivery_interfaces/action/ExecuteDelivery
python3 -m unittest discover -s tests -v
```

### 4.5. Mỗi terminal mới

Thực hiện trước mọi lệnh ROS của repo:

```bash
cd "$HOME/dwd_uav"  # thay bằng đường dẫn repo thực tế
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

Dùng cùng domain trên các node và `UXRCE_DDS_DOM_ID` của PX4. Các hướng dẫn native
sử dụng `install/setup.bash`; bản build Docker riêng nằm ở `install/container`.

## 5. Cài Micro XRCE-DDS Agent

Cài trên **PC chạy SITL** hoặc **Pi kết nối flight controller**. Build trong thư
mục riêng để colcon không nhận nhầm mã nguồn Agent là package của workspace:

```bash
mkdir -p "$HOME/uav-tools"
cd "$HOME/uav-tools"
git clone --branch v2.4.2 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build
cmake --build Micro-XRCE-DDS-Agent/build -j2
sudo cmake --install Micro-XRCE-DDS-Agent/build
sudo ldconfig
command -v MicroXRCEAgent
```

Đây là [tag v2.4.2 của Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/releases/tag/v2.4.2).
Chỉ chạy **một Agent cho mỗi kênh kết nối**:

```bash
# PC/SITL, UDP:
MicroXRCEAgent udp4 -p 8888
```

Hoặc trên Pi/UART:

```bash
MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```

Để terminal này chạy liên tục. Topic `/fmu/out/*` xuất hiện sau khi client PX4
kết nối. Agent không tự khởi động mission.

## 6. Setup PX4 và mô phỏng

### 6.1. Checkout PX4 đúng phiên bản trên PC

```bash
mkdir -p "$HOME/uav-tools"
cd "$HOME/uav-tools"
git clone --branch v1.17.0 --recursive https://github.com/PX4/PX4-Autopilot.git
```

Cài toolchain theo [PX4 Ubuntu development environment](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu).
Vì repo chọn Harmonic, bỏ phần simulator của script PX4 để cài Gazebo riêng:

```bash
bash "$HOME/uav-tools/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-sim-tools
```

Khởi động lại nếu trình cài yêu cầu. Trên máy chỉ dùng SITL có thể thêm
`--no-nuttx`; máy cần build firmware flight controller giữ toolchain NuttX.

### 6.2. Bật topic xác nhận landed — bắt buộc

Trở về repo, chạy:

```bash
cd "$HOME/dwd_uav"
python3 scripts/enable_land_topic.py "$HOME/uav-tools/PX4-Autopilot"
```

PX4 v1.17 đã xuất bản `/fmu/out/vehicle_land_detected` mặc định. Script kiểm tra
và bổ sung topic nếu checkout đã tùy chỉnh làm thiếu nó; chỉ phải build lại khi
`dds_topics.yaml` thay đổi. Adapter tự thêm hậu tố `_vN` theo `MESSAGE_VERSION`,
ví dụ `/fmu/out/vehicle_status_v1` và `/fmu/out/battery_status_v1`.

Đối với flight controller thật, build target đúng model board và nạp file `.px4`
tương ứng qua QGroundControl. Repo chưa chốt model FC nên không có lệnh flash
chung cho mọi board. Không nạp binary SITL lên flight controller.

### 6.3. Cài Gazebo và bridge trên PC

Cài `gz-harmonic` theo [hướng dẫn Ubuntu của Gazebo](https://gazebosim.org/docs/harmonic/install_ubuntu/),
rồi cài bridge từ repository OSRF vừa cấu hình:

```bash
sudo apt install ros-humble-ros-gzharmonic
```

Humble mặc định ghép với Fortress; bridge `ros-humble-ros-gzharmonic` xung đột với
nhóm `ros-humble-ros-gz*` mặc định. Không cài chồng hai bộ. Chi tiết tại
[bảng tương thích Gazebo/ROS](https://gazebosim.org/docs/harmonic/ros_installation/).
Phần Gazebo đầu-cuối của repo vẫn đang chờ kiểm chứng thực tế.

### 6.4. Build PX4 và sinh world

```bash
cd "$HOME/uav-tools/PX4-Autopilot"
make px4_sitl
cd "$HOME/dwd_uav"
python3 scripts/prepare_sitl.py "$HOME/uav-tools/PX4-Autopilot"
```

Bộ sinh tạo `build/sitl_assets`: x500 với camera hướng xuống, lidar, hai bãi
ArUco và `sitl.yaml`. Marker giao hàng ID 0 nằm 3 m phía đông; marker nhà ID 1
ở điểm xuất phát. Kích thước marker đen là 0,4 m.

### 6.5. Chạy ba terminal riêng

**Terminal A — Agent:**

```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal B — PX4/Gazebo, tại root repo:**

```bash
bash scripts/run_sitl.sh "$HOME/uav-tools/PX4-Autopilot" "$PWD/build/sitl_assets"
```

**Terminal C — đã source ROS và workspace theo mục 4.5:**

```bash
GZ_PARTITION=dwd_uav_native ros2 launch delivery_ros sitl.launch.py config:="$PWD/build/sitl_assets/sitl.yaml"
```

Mở QGroundControl, chờ GPS/estimator sẵn sàng. Xem mục 9 để kiểm tra trạng thái
và gửi mission lấy GPS từ cấu hình sinh tự động. Không dùng GPS ví dụ ngoài
world này. Sau một lượt giao, cần khởi động lại payload sim để nạp lại hàng giả lập.

Xem thêm [quy trình SITL và các giới hạn](docs/sitl.md). Profile `sitl.yaml` bật
điều khiển, dùng payload giả lập và range từ ROS; **không dùng profile này trên drone thật**.

## 7. Setup Raspberry Pi 4 và thiết bị

### 7.1. Hệ điều hành và quyền thiết bị

Cài Ubuntu Server 22.04 arm64, cấu hình mạng/SSH, hoàn thành mục 2, 4 và 5 trên Pi.
Sau đó:

```bash
sudo usermod -aG dialout,video "$USER"
```

Đăng xuất rồi đăng nhập lại. Kiểm tra thiết bị:

```bash
groups
ls -l /dev/serial0
ls -l /dev/serial/by-id/
ls -l /dev/video*
```

Thiết bị phải tồn tại và tài khoản chạy node phải có quyền truy cập. Không dùng
`chmod 777` cho toàn bộ cổng thiết bị.

### 7.2. UART giữa Pi và PX4

- Bật UART bằng `enable_uart=1` trong `/boot/firmware/config.txt` trên Ubuntu Pi.
- Tắt serial console trên cổng sử dụng; nếu chỉnh `/boot/firmware/cmdline.txt`,
  bỏ riêng token `console=serial0,...` tương ứng và giữ nguyên các tham số khác
  trên một dòng. Reboot sau khi thay đổi boot configuration.
- Kiểm tra `/dev/serial0` trỏ đúng UART đã đấu dây; không giả định cùng tên cổng
  với USB nối MCU payload.
- Nối TX Pi ↔ RX FC, RX Pi ↔ TX FC và GND theo pinout hãng, mức logic 3,3 V.
- Dùng nguồn riêng đủ tải cho Pi; không lấy điện cấp Pi từ chân TELEM khi chưa
  xác minh khả năng cấp dòng.

Trong QGroundControl: chọn cổng thật bằng `UXRCE_DDS_CFG`, đặt baud của cổng đó
là 921600, tắt MAVLink trên **cùng cổng** và đặt `UXRCE_DDS_DOM_ID=0`. Reboot FC
nếu tham số yêu cầu. Giữ một đường RC/QGroundControl độc lập để vận hành.

Xác minh Offboard-loss qua `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` và auto-disarm sau
hạ cánh theo airframe. Mission chờ PX4 xác nhận disarmed, không force-disarm.

### 7.3. Camera và calibration

Driver camera phải xuất:

| Topic | Kiểu |
|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

CameraInfo cần intrinsics, distortion hợp lệ, cùng frame_id và độ phân giải
với Image. Repo không cung cấp calibration chung cho mọi camera. Khởi đầu ở
640×480, 10 FPS; dùng driver phù hợp camera, lưu calibration và cấu hình driver
nạp lại khi khởi động.

Cần TF `base_link → camera_optical_frame` theo phép đo vị trí và hướng lắp camera.
Adapter tự phát `map → base_link`. Kiểm tra bằng:

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image_raw
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

Không lấy transform trong launch SITL dùng cho camera thật nếu lắp khác hướng.
Đặt `vision.calibrated: true` chỉ sau khi xác minh calibration và TF. ArUco dùng
`DICT_5X5_250`; đo cạnh **ô marker đen**, không tính viền trắng, để đặt `marker_size`.

### 7.4. Rangefinder

Kết nối và cấu hình rangefinder theo loại cảm biến trên PX4. Trong
`/fmu/out/vehicle_local_position`, cần `dist_bottom_valid: true`, khoảng cách hợp
lệ và `dist_bottom_sensor_bitfield` có bit RANGE. Không dùng riêng độ cao GPS
để thay xác nhận khoảng cách tới mặt đất.

### 7.5. MCU nhả hàng

Firmware tham chiếu tại [payload_controller.ino](firmware/payload_controller/payload_controller.ino).
Dùng Arduino IDE mở sketch, chọn đúng Nano/ATmega328P và bootloader của board,
cài thư viện Servo nếu thiếu, kiểm tra pin/góc servo rồi compile/upload.
Firmware này chưa được nghiệm thu trên phần cứng thực tế.

| Kết nối tham chiếu | Chân Nano |
|---|---|
| Công tắc kẹp đóng | D2, đóng về GND |
| Cảm biến có hàng | D3, đóng về GND |
| Tín hiệu servo | D9 |
| Pi ↔ MCU | USB serial, 115200 baud |

Nguồn servo riêng đủ tải, chung GND. Đặt `payload.port` theo `/dev/serial/by-id/...`.
MCU có watchdog và chống lặp lệnh theo UUID; xem [giao thức và phục hồi lỗi](docs/payload.md).
Thử bàn không gắn cánh; ngắt kết nối phải dừng chuyển động cơ cấu.

## 8. Cấu hình và khởi chạy trên Pi

### 8.1. Tạo file cấu hình site

Tại root repo:

```bash
mkdir -p "$HOME/.config/drone-delivery"
cp src/delivery_ros/config/pi4.yaml "$HOME/.config/drone-delivery/site.yaml"
nano "$HOME/.config/drone-delivery/site.yaml"
```

Các giá trị cần sửa:

| Node/tham số | Ý nghĩa |
|---|---|
| `mission.latitude`, `longitude` | GPS điểm giao; `0.0` mặc định là placeholder |
| `mission.relative_altitude` | Độ cao bay theo mét so với điểm xuất phát |
| `mission.delivery_marker_id`, `home_marker_id` | Hai ID khác nhau, trong 0–249 |
| `mission.radius`, `spacing` | Vùng và bước tìm marker, mét |
| `mission.min_battery` | Ngưỡng tỷ lệ pin 0–1, mặc định 0,3 |
| `mission.max_distance` | Khoảng cách tối đa tới điểm giao, mặc định 200 m |
| `mission.enable_control` | Cho phép nhận goal điều khiển |
| `px4_adapter.enable_control` | Cho phép phát lệnh xuống PX4 |
| `vision.calibrated`, `marker_size` | Xác nhận hiệu chuẩn và cạnh marker theo mét |
| `payload.backend`, `port` | Phần cứng dùng `serial` và cổng USB thực tế |

Giữ hai cờ `enable_control: false` trong lần kiểm tra kết nối đầu. Khi đủ điều
kiện thử mới đổi **cả hai** sang true và khởi động lại launch. Không dùng
`ros2 param set` để bật giữa chừng: các giá trị này được đọc lúc node khởi tạo.

### 8.2. Thứ tự khởi chạy

1. Bật flight controller, Pi, camera và MCU; kiểm tra RC/QGroundControl.
2. Chạy Agent UART theo mục 5 trong terminal riêng.
3. Chạy camera driver và static TF đã hiệu chuẩn.
4. Trong terminal đã source ROS và workspace:

```bash
ros2 launch delivery_ros delivery.launch.py \
  config:="$HOME/.config/drone-delivery/site.yaml"
```

Nếu camera dùng tên topic khác:

```bash
ros2 launch delivery_ros delivery.launch.py \
  config:="$HOME/.config/drone-delivery/site.yaml" \
  image_topic:=/image_raw camera_info_topic:=/camera_info
```

Launch khởi động các node `mission`, `px4_adapter`, `vision`, `payload`,
`competition`, `safety_monitor` và `vio_bridge`; không tự
chạy Agent hoặc camera driver, và không tự bắt đầu nhiệm vụ khi boot.

Sau khi chạy thủ công ổn định mới dùng [service mẫu](deploy/delivery.service):
sửa `User`, `WorkingDirectory`, `ExecStart` theo máy, bảo đảm Agent/camera/TF được
quản lý riêng. Service mẫu không tự restart và không tự gửi goal.

## 9. Gửi nhiệm vụ, theo dõi và hủy

### 9.1. Kiểm tra trước khi gửi goal

Trong terminal mới đã làm mục 4.5:

```bash
ros2 node list
ros2 action list -t
ros2 topic echo /delivery/flight_state --once
ros2 topic echo /delivery/payload_state --once
ros2 topic echo /delivery/vision_ready --once
```

Mong đợi: flight `healthy: true`, `landed: true`, `armed: false`, `range_valid: true`;
payload healthy, closed, present đều true; vision ready true. Điều khiển phải được
bật trong cấu hình của cả mission và adapter để goal được thực hiện.

### 9.2. Chạy GPS đã cấu hình

```bash
ros2 service call /delivery/start std_srvs/srv/Trigger '{}'
```

Service chỉ xác nhận **đã gửi yêu cầu**. Chấp nhận và kết quả thực tế được phản
ánh qua topic/action. Với SITL, lệnh này dùng GPS từ `build/sitl_assets/sitl.yaml`.

Trong mô phỏng được tạo bởi `run_native.sh sim`, điểm A là vị trí UAV khi nhận
goal; điểm B là pad ArUco `0` cách A 3 m theo hướng Đông và độ cao hành trình là
2 m. Chuỗi điều khiển tới B là `PREFLIGHT → TAKEOFF → TRANSIT → SEARCH →
APPROACH → DESCEND → LAND → RELEASE`.

#### Vì sao service chỉ gửi goal

Bay A → B là tác vụ dài, cần feedback, hủy và kết quả cuối nên phần thực thi là
ROS 2 Action `/delivery/execute`. Service `/delivery/start` dùng `Trigger` chỉ làm
cổng khởi động: tạo `ExecuteDelivery.Goal`, gọi `send_goal_async()` rồi trả về
ngay. Không giữ callback service cho tới lúc UAV hạ cánh.

Luồng giao tiếp:

```text
/delivery/start (std_srvs/Trigger)
        │ gửi goal mặc định
        ▼
/delivery/execute (ExecuteDelivery action)
        │ feedback/result
        └──► /delivery/mission_status (MissionStatus)
```

Mẫu callback tối thiểu:

```python
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from delivery_interfaces.action import ExecuteDelivery

self.client = ActionClient(self, ExecuteDelivery, '/delivery/execute')
self.start_srv = self.create_service(Trigger, '/delivery/start', self.start)

def start(self, request, response):
    if not self.client.server_is_ready():
        response.success = False
        response.message = 'Mission action server chưa sẵn sàng'
        return response

    goal = ExecuteDelivery.Goal()
    goal.latitude = 47.397971057728974
    goal.longitude = 8.546203597337506
    goal.relative_altitude = 2.0
    goal.delivery_marker_id = 0
    goal.home_marker_id = 1
    self.client.send_goal_async(goal, feedback_callback=self.on_feedback)

    response.success = True
    response.message = 'Goal đã gửi; theo dõi /delivery/mission_status'
    return response
```

Code đang chạy nằm trong
[`src/delivery_ros/delivery_ros/mission.py`](src/delivery_ros/delivery_ros/mission.py).
Nếu cần truyền tọa độ qua service thay vì action trực tiếp, tạo
`StartDelivery.srv`:

```text
float64 latitude
float64 longitude
float32 relative_altitude
int32 delivery_marker_id
int32 home_marker_id
---
bool accepted
string message
```

Callback của service tùy biến vẫn chỉ chuyển các trường request thành
`ExecuteDelivery.Goal`; state machine bay không nên đặt trong callback service.

### 9.3. Gửi action trực tiếp với feedback

Ví dụ cú pháp — thay GPS bằng điểm thử đã chuẩn bị, không dùng nguyên tọa độ mẫu:

```bash
ros2 action send_goal /delivery/execute delivery_interfaces/action/ExecuteDelivery \
  '{latitude: 47.397971057728974, longitude: 8.546203597337506, relative_altitude: 2.0, delivery_marker_id: 0, home_marker_id: 1}' \
  --feedback
```

Theo dõi từ terminal khác, kể cả kết nối sau khi mission đã chạy:

```bash
ros2 topic echo /delivery/mission_status --qos-durability transient_local
```

`MissionStatus` chứa UUID goal, sequence, execution_state, phase, progress,
elapsed, payload_released, error_code và detail. Topic, feedback và result dùng
cùng snapshot. Feedback 2 Hz và khi đổi pha/lỗi; progress là mốc hoàn thành,
không phải ETA, chỉ bằng 1 khi hoàn thành cả chặng về.

### 9.4. Hủy nhiệm vụ

```bash
ros2 service call /delivery/stop std_srvs/srv/Trigger '{}'
```

Khi đang bay, trạng thái giữ `CANCELING` trong lúc xử lý Return/Land. Khi đang
ở đất, hệ thống ngăn các bước nhả/cất cánh chưa bắt đầu; nếu thao tác nhả đã bắt
đầu, chờ kết quả hủy cơ cấu trước khi kết thúc. Lỗi xử lý hủy trả `FAILED`.

RC takeover nhường quyền cho người vận hành. Tắt terminal/Ctrl+C không tương
đương yêu cầu Return; mất tiến trình làm PX4 thực hiện failsafe Offboard-loss.
Bãi giao phải được kiểm soát vì chu trình có thể tự cất cánh lại sau khi nhả hàng;
cảm biến hàng/kẹp không phát hiện người đứng quanh drone.

## 10. Ghi log và kiểm tra hiệu năng

Tại root repo, với ROS/workspace đã source:

```bash
bash scripts/record.sh
```

Rosbag ghi trạng thái, ý định điều khiển, telemetry, TF và camera. Chuẩn bị đủ
chỗ trống vì ảnh raw chiếm nhiều dung lượng. Lưu thêm ULog từ PX4 để đối chiếu.

Đo tải Pi khi hệ thống đang được vận hành trong bài thử:

```bash
python3 scripts/benchmark_pi.py --seconds 1800 --output benchmark.json
```

Script chỉ quan sát, không arm/gửi mission. Đánh giá độ trễ ảnh p95 ≤200 ms,
khoảng ngắt heartbeat <0,5 s và throttling. Cần `vcgencmd` trên Pi để đọc cờ
throttling; thiếu dữ liệu thì không đánh dấu đạt. Xem [kiểm chứng](docs/validation.md).

## 11. Lỗi thường gặp

| Triệu chứng | Kiểm tra và cách xử lý |
|---|---|
| `ros2: command not found` | Cài Humble; source `/opt/ros/humble/setup.bash` trong terminal hiện tại |
| Không tìm thấy `delivery_ros` hoặc custom action | Build thành công rồi source đúng `install/setup.bash`; không dùng install từ kiến trúc khác |
| Không có `/fmu/out/*` | Agent/client, UART/UDP, domain, baud và firmware; chỉ một Agent trên một kênh |
| Có PX4 topics nhưng `healthy: false` | Thiếu land detector, GPS chưa fix, dữ liệu pin/odometry cũ; xem `/delivery/flight_state` |
| Thiếu `vehicle_land_detected` | Chạy script bổ sung DDS rồi build/nạp lại đúng firmware; khởi động lại Agent/FC |
| Permission denied trên serial/video | Cổng thiết bị và nhóm `dialout,video`; đăng nhập lại sau `usermod` |
| Payload node thoát ngay | `payload.port` còn placeholder, MCU chưa cắm hoặc cổng không mở được |
| `vision_ready: false` | CameraInfo, frame/dimensions, ảnh có timestamp mới, TF và `calibrated` |
| Thấy marker nhưng không căn tâm | Đúng dictionary/ID/kích thước, reprojection error, nhiều ảnh mới và TF tại thời điểm chụp |
| `range_valid: false` | Rangefinder PX4/cờ RANGE; SITL kiểm tra bridge `/camera/scan` |
| Goal bị từ chối | `enable_control` của mission, telemetry, tham số goal hoặc đang có goal khác |
| Goal nhận rồi báo preflight thất bại | Camera/range/payload, pin, landed/disarmed, GPS trong khoảng cách cho phép |
| ACK timeout/command failed | QGroundControl prearm checks, trạng thái mode, time sync và liên kết DDS |
| Đáp rồi không nhả | Chờ landed **và** disarmed ≥2 s; kiểm tra auto-disarm PX4 và interlock MCU |
| Nhả rồi không cất cánh lại | Cảm biến hàng phải báo trống, kẹp đóng, pin/camera/range còn đủ điều kiện |
| `ros_gz_bridge` lỗi thư viện | Kiểm tra đang dùng bộ bridge Harmonic đúng với Humble, không trộn Fortress |
| Build bị kill trên Pi | Dùng `colcon build --executor sequential`, giảm tác vụ khác và kiểm tra RAM/nhiệt độ |

## 12. Cấu trúc repo và tài liệu

```text
src/delivery_interfaces/    MissionStatus, action và message giữa các node
src/delivery_ros/           Engine, PX4 adapter, vision, payload, launch/config
firmware/                  Sketch MCU nhả hàng tham chiếu
scripts/                   Setup DDS/SITL, ghi log, benchmark, kiểm thử container
tests/                     Kiểm thử logic, ROS action, adapter, vision, assets
deploy/                    Service mẫu cho Pi
.github/workflows/         CI build/test ROS Humble
docs/                      Hướng dẫn và bản ghi kiểm chứng
```

- [SITL chi tiết](docs/sitl.md)
- [Payload MCU và giao thức serial](docs/payload.md)
- [Kết quả và các bước chưa nghiệm thu](docs/validation.md)
- [MissionStatus](src/delivery_interfaces/msg/MissionStatus.msg)
- [ExecuteDelivery](src/delivery_interfaces/action/ExecuteDelivery.action)

Tham khảo chức năng từ các repo `drone-delivery`, `gps-denied-drone-survey`,
`AEAC-Controls-2022` và `drone-winch-gripper` trong workspace. Repo mới chưa có
VIO/SLAM, tránh vật cản, nhận diện người, bánh xe hoặc tời. Chưa chọn giấy phép
phát hành (`UNLICENSED`); cần cập nhật maintainer/email trước công bố.
