# 基于海康 MVS SDK 的 ROS 2 相机驱动

> **一句话说明**：一个把海康（Hikrobot）USB3 Vision 工业相机接入 ROS 2 的 `hikrobot_camera` 功能包，实现设备发现、图像发布、参数控制与断线重连。

| 项目 | 内容 |
|---|---|
| 功能包名 | `hikrobot_camera` |
| 目标相机 | Hikrobot MV-CS016-10UC（USB3 Vision） |
| 核心依赖 | 海康 MVS SDK（`/opt/MVS`）、ROS 2 |
| 图像话题 | `/image_raw`（`sensor_msgs/Image`，`bayer_rggb8`） |
| 参数节点名 | `/hikrobot_camera` |

---

## 目录

- [1. 环境说明 ⚠️](#1-环境说明-️)
- [2. 依赖安装](#2-依赖安装)
  - [2.1 MVS SDK](#21-mvs-sdk)
  - [2.2 usbfs_memory_mb ⭐](#22-usbfs_memory_mb-)
  - [2.3 ROS 依赖](#23-ros-依赖)
- [3. 编译](#3-编译)
- [4. 运行](#4-运行)
- [5. 可配置参数 ⭐](#5-可配置参数-)
  - [5.1 参数文件](#51-参数文件)
  - [5.2 运行时改参数](#52-运行时改参数)
- [6. 功能说明](#6-功能说明)
- [7. 已知问题 / 未完成](#7-已知问题--未完成)
- [8. 参考文献](#8-参考文献)

---

## 1. 环境说明 ⚠️

| 项目 | 版本 |
|---|---|
| 作业要求 | Ubuntu 22.04 + ROS 2 Humble |
| 本机实测 | Ubuntu 24.04 + ROS 2 Jazzy |

**差异影响：**

- 命令中的 `humble` 需替换为 `jazzy`。
- **环境变量加载**：Humble 通常用 `source /opt/ros/humble/setup.bash`；本机使用 `source /opt/ros/jazzy/setup.zsh`（根据实际 shell 选择 `setup.bash` 或 `setup.zsh`）。
- ⚠️ 若助教按 Humble 命令运行，请先确认 ROS 发行版（`ls /opt/ros`）并对应替换。

```bash
# 确认本机 ROS 发行版
ls /opt/ros
```

---

## 2. 依赖安装

### 2.1 MVS SDK

| 项目 | 内容 |
|---|---|
| 下载来源 | 海康机器人官网 |
| 安装版本 | 4.8.2.x |
| 安装路径 | `/opt/MVS` |
| udev 规则 | `80-drivers-SDK-2bdf.rules`，其中 `2bdf` 为海康厂商 ID |

> 说明：udev 规则用于让普通用户免 `sudo` 访问相机设备节点。

### 2.2 usbfs_memory_mb ⭐

Linux 默认 `usbfs_memory_mb` 为 **16 MB**，USB3 Vision 相机会出现**丢帧或开流失败**。需设置为 **2000 MB**，且重启后失效（内核参数，非持久化）。

通过 systemd 服务设置，写入 `/etc/systemd/system/usbfs_memory.service`：

```ini
# /etc/systemd/system/usbfs_memory.service
[Unit]
Description=Set usbfs_memory_mb

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo 2000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

启用并启动：

```bash
sudo systemctl enable usbfs_memory.service
sudo systemctl start usbfs_memory.service
```

验证：

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
# 应输出 2000
```

### 2.3 ROS 依赖

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

> ⚠️ 指定 `--rosdistro jazzy`，避免默认按当前 distro 解析导致依赖不匹配。

---

## 3. 编译

```bash
cd ~/assignment3-ROS2  # 工作空间根目录（本仓库）
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select hikrobot_camera
```

**注意事项：**

- 需要**先 source ROS 2 环境**（Jazzy 对应 `setup.zsh` 或 `setup.bash`）。
- MVS 库**不在系统缓存中**（`/opt/MVS/lib/64`），通过 `target_*` 系列命令让链接器找到：

  ```cmake
  target_include_directories(camera_node PRIVATE include /opt/MVS/include)
  target_link_directories(camera_node PRIVATE /opt/MVS/lib/64)
  target_link_libraries(camera_node MvCameraControl)
  ```

- 使用 `target_*` 而非 `link_directories`：作用域限定在 `camera_node` 目标，避免污染其他目标。

---

## 4. 运行

```bash
cd ~/assignment3-ROS2
source install/setup.zsh
ros2 launch hikrobot_camera camera.launch.py
```

带参数文件启动：

```bash
ros2 launch hikrobot_camera camera.launch.py \
    params_file:=$(ros2 pkg prefix hikrobot_camera)/share/hikrobot_camera/config/camera.yaml
```

查看图像：

```bash
rviz2
# 或
rqt_image_view
```

话题：`/image_raw`

---

## 5. 可配置参数 ⭐

| 参数 | 类型 | 单位 | 范围 | 说明 |
|---|---|---|---|---|
| `exposure_time` | `double` | µs | 15 ~ 9996427 | 曝光时间。**范围从相机读出，非猜测。** |
| `gain` | `double` | dB | 0 ~ 15 | 增益。**范围从相机读出，非猜测。** |
| `pixel_format` | `string` | — | `BayerRG8`（仅支持此格式） | 像素格式。 |
| `image_topic` | `string` | — | — | 图像话题名。 |
| `camera_serial` | `string` | — | — | 目标相机序列号。 |
| `frame_rate` | `double` | — | — | ⚠️ **见下方说明。** |

**关于 `frame_rate`：**

本相机（MV-CS016-10UC）无 `AcquisitionFrameRate` 节点，**不支持直接设置帧率**。实际帧率由曝光时间决定（帧率 ≈ 1/曝光时间）。设置该参数会返回失败并说明原因。

### 5.1 参数文件

`config/camera.yaml`：

```yaml
/hikrobot_camera:
  ros__parameters:
    use_sim_time: false
    exposure_time: 200000.0
    gain: 0.0
    pixel_format: "BayerRG8"
    image_topic: "image_raw"
    camera_serial: "DB1921834"
```

launch 时加载：

```bash
ros2 launch hikrobot_camera camera.launch.py \
    params_file:=$(ros2 pkg prefix hikrobot_camera)/share/hikrobot_camera/config/camera.yaml
```

### 5.2 运行时改参数

```bash
ros2 param set /hikrobot_camera exposure_time 8000.0
ros2 param set /hikrobot_camera gain 5.0
```

> ⚠️ **运行时参数不持久化**：`ros2 param set` 修改的值仅在当前进程内生效，重启节点后恢复为 `config/camera.yaml` 中的值。

---

## 6. 功能说明

| 作业要求 | 实现 |
|---|---|
| 设备发现与选择 | 枚举设备 + 按序列号选择 |
| 图像发布 | `sensor_msgs/Image`，`bayer_rggb8` 编码 |
| 参数读写 | 声明 + 范围校验 + 运行时动态生效 |
| 断线重连 | 异常回调触发 → 重新枚举 → 按序列号重连 → 恢复配置 |

---

## 7. 已知问题 / 未完成

- **`frame_rate` 参数不支持**（原因见第 5 节）。
- **时间戳使用 ROS 的 `now()`**，不是设备硬件时间戳。
- **`handle_` 在多线程下的访问没有加锁**（重连回调与主线程并发）。

## 8. 参考文献

- MVS SDK 文档路径：`/opt/MVS/doc/`
- 用到的范例：`GrabImage.cpp`、`ReconnectDemo.cpp`
