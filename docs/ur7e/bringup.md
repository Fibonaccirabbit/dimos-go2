# UR7e：Mac 上的 dimOS + Ubuntu ROS 工作站

第一阶段是**真实反馈、Agent 技能和受限关节轨迹**，不是自动抓取 demo。

```text
Mac：dimOS Agent / MCP / UR7eROSAdapter
  → SSH 加密的短请求（不保存密码、不开放控制端口）
Ubuntu：ros_bridge → 官方 ROS 2 UR Driver → scaled trajectory controller
  → UR7e 的 External Control 程序
```

ROS 驱动和桥运行在已有 Ubuntu 工作站；Mac 不需要安装 ROS 或部署整个 dimOS 到工作站。

## 已支持与边界

| 能力 | 第一阶段 |
|---|---|
| 六关节角度、速度、effort 反馈 | 实际 `/joint_states`，按名称重排；effort 单位单独标注，不把电流当力矩 |
| TCP / 六轴力传感器 | 实际 ROS 广播器；过期或缺失返回 `None` |
| Agent / MCP | `ur7e_status`、`ur7e_preview_joint_delta`、`ur7e_move_joint_delta`、`ur7e_stop` |
| 小幅关节测试 | 完整 quintic ROS action；工作站再次检查反馈、状态、幅度、URDF 限位和速度 |
| 停止 | 只取消本桥拥有的未完成轨迹；不是硬件急停，不关电、不换模式 |
| 自动开机、解锁、清保护停机、启动程序 | 不支持 |
| RGB-D 相机 | ROS RGB/原始深度/内参/深度到 RGB 外参 → 独立 SSH 只读桥 → dimOS 图像流和本机预览 |
| RGB ROI 表面定位 | `ur7e_preview_target_roi`，真实深度重投影，输出相机 optical 坐标，非底座运动目标 |
| 笛卡尔悬停路径 | 工作站 cuRobo 离线预览；新相机标定、工具与完整场景尚未验证，禁止执行 |
| 高频 servo、在线碰撞监测、夹爪/抓取 | 尚未接入 |

Adapter 注册名 `ur7e_ros`，满足 `ManipulatorAdapter` 接口，但刻意拒绝
`SERVO_POSITION`、速度和力矩控制。不能把 SSH 上的 ROS action 接到 dimOS
100 Hz 协调器中冒充 servo。此阶段由 UR 控制器执行完整轨迹，Agent 技能等待实际结果。
未来应选择校准后的工作站整轨迹规划，或实现工作站侧本地 servo，再衔接 dimOS 规划器。
已有 cuRobo 离线规划不等于整轨迹已经接入此适配层。

默认只读。单次动作范围、时间、反馈有效期和 watchdog 阈值集中定义于
[policy.py](/dimos/hardware/manipulators/ur7e/policy.py)。当前限幅只是调试边界，
**不代表碰撞检查、安全认证或现场安全保证**，也不应通过连续调用绕过限幅。
当前单次最大幅度为 5°；速度限制仍为 0.01 rad/s，5° 测试采用 20 秒轨迹。

## 准备

工作站需要已有 ROS 2 Humble、官方 `ur_robot_driver` 和已验证的工厂标定配置。
机器人由现场操作员开机、释放制动并运行 External Control；桥不会代做这些操作。
运动前还要求目标 scaled controller 激活，且没有竞争的运动控制器。

Mac 使用已安装 dimOS 的 Python 环境。通过 SSH key 或已认证的 SSH ControlMaster
连接工作站，勿把密码写入代码、命令行或版本库。设置下面三个任务变量：

- `UR7E_SSH_TARGET`：SSH 别名或 `user@host`。
- `UR7E_REMOTE_ROOT`：工作站中新建桥目录的绝对路径。
- `UR7E_CONTROL_PATH`：已认证的本地 SSH socket；使用 SSH key 时可省略对应参数。

部署时，保留现有工作站脚本和 ROS 安装。只把以下两个源文件复制到新目录的
`dimos/hardware/manipulators/ur7e/` 下：`policy.py`、`ros_bridge.py`。
它们不依赖 dimOS 其余 Python 包；无需在工作站安装完整 dimOS。

## 只读验收

```bash
python -m dimos.robot.universal_robots.demo_ur7e \
  --address "$UR7E_SSH_TARGET" \
  --remote-root "$UR7E_REMOTE_ROOT" \
  --control-path "$UR7E_CONTROL_PATH"
```

输出包含真实状态和关节六 `+0.5°` 预览，`executed=false`。
预览会验证关节限位和当前状态，但不代表机器人前方无障碍物。

启动 Agent 工具服务（启动本身不会产生轨迹）：

```bash
python -m dimos.robot.universal_robots.demo_ur7e --mcp \
  --address "$UR7E_SSH_TARGET" \
  --remote-root "$UR7E_REMOTE_ROOT" \
  --control-path "$UR7E_CONTROL_PATH"
```

服务仅监听本机，默认 MCP 地址来自 `GlobalConfig.mcp_port`。
运行内置蓝图时分别使用 `ur7e-ros` 或 `ur7e-ros-agentic`，并通过
`--ur7eskillcontainer.address`、`--ur7eskillcontainer.remote-root` 等模块参数提供配置。
在已有 Go2 服务旁运行时必须隔离 MCP 端口和消息总线，避免 Agent 消息串台。

## 自然语言

在 `--mcp` 命令中使用 `--agent`，按现有模型配置提供 `--model`、
`--model-base-url`、`--model-api-key-env` 和可选的 `--model-use-responses-api`。
模型密钥只从指定环境变量读取，可继续使用本机 Keychain，不存入仓库。
蓝图使用 UR7e 专用 system prompt，不复用 Go2 的导航提示词。

推荐先验证：

> 查看机械臂当前关节和 TCP 状态，再预览第六关节增加 0.5 度，5 秒完成。不要执行运动。

只读会话中即使 Agent 调用运动工具，也会明确得到 `READ_ONLY`，不会发送轨迹。
需要现场监护的运动会话由操作员显式以 `--allow-motion` 启动；不是 Agent 可调用的解锁工具。
执行结果必须同时包含 ROS action 成功和实测终点吻合，不能以“已接受目标”冒充完成。

## 验证记录与下一步

2026-10-02 已通过 Mac → SSH → 官方 ROS 驱动的真机反馈读取，MCP 状态与预览调用，
以及只读运动请求拒绝。此前直接 ROS 真机测试：关节六目标 `+0.5°`，实测 `+0.4995°`，
约 5 秒完成，action 成功；该测试发生在本适配层接入前，不能代替适配层运动验收。
随后已通过本适配层实机验收：MCP → dimOS skill → ROS action
执行关节六 `+5°` / 20 秒一次，实测 `+5.00084594°`，20.053 秒，
action status=4/error=0，10027 个反馈样本，峰值速度 0.009761 rad/s。
其余关节终点变化均小于 0.001°；结束后六关节速度为零，没有自动回转。

自然语言只读实测也通过：DeepSeek 实际调用 `ur7e_status` 与预览工具，读取真机反馈，
生成关节六 `+0.5°` / 5 秒预览，并明确报告未执行、未进行碰撞检查。
只读 `motion_enabled=false` 是本适配会话的权限，并不表示机器人伺服电源关闭。

另已核实工作站 UR driver `2.13.0` 使用 `actual_current` 作为 joint effort，单位为 A，
不是 Nm（[该版本官方实现](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.0/ur_robot_driver/src/hardware_interface.cpp)）。
桥从实际 RTDE recipe 与硬件参数标注单位；`read_joint_efforts()` 不会把电流伪装为力矩。
TCP 和六轴力数据同时保留 ROS `frame_id`，后续标定不可混用 UR `base` 与 ROS `base_link`。

测试包含策略、反馈、adapter 协议、MCP schema、watchdog 和蓝图注册检查。
检查覆盖限幅、速度、只读门控、相机无损深度和本机数据边界；新增源文件 Ruff 与定向 mypy 检查通过。
全仓/机械臂运动集成测试未完成；当前 Mac 的 unrelated whole-body adapter
检查缺少 `can_motor_control`，不要通过修改 UR7e 逻辑掩盖该环境问题。

## 相机接入

已实测 Orbbec Gemini 335L，工作站已有 Orbbec ROS 驱动。启动相机驱动
（不控制机械臂；如果设备被别的程序占用，先确认后再停止原程序）：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=ur7e_camera color_width:=640 color_height:=480 color_fps:=15 \
  depth_width:=640 depth_height:=480 depth_fps:=15 \
  enable_point_cloud:=false enable_colored_point_cloud:=false enumerate_net_device:=false
```

另部署 `camera_bridge.py` 和其纯几何依赖 `target_geometry.py` 至同一桥目录。
相机桥需要已有 `tf2_ros`；不会发布相机到机械臂底座的外参。Mac 启动时加 `--camera`，
可用 `--camera-port <本机空闲端口>` 开启只监听 127.0.0.1 的预览。
预览只有 GET 图像/状态接口，没有运动接口；不修改现有 Go2 调试台。

相机以 15 FPS 采集，SSH 读取和 dimOS 发布限为 5 FPS；RGB JPEG 压缩，
深度 PNG 无损保留 uint16。发布 `color_image`、`depth_image`、
`camera_info`、`depth_camera_info`，保留实际源时间戳、内参和 optical frame。
源驱动的深度缩放转为毫米值（[官方实现](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/src/ob_camera_node.cpp)）；
默认 `depth_unit_m=0.001` 仅适用于这一配置，换驱动必须重新核实。
状态报告 RGB/深度实际时间差；原始图像仍不是按像素对齐的数据，
不可直接用 RGB 像素索引深度。ROI 定位通过驱动实际的
`color_optical ← depth_optical` TF、各自内参和畸变完成三维重投影。
若外参缺失、数据过期、不同步或 ROI 深度混杂，定位工具拒绝输出。

Mac 已独立订阅并收到真实 BGR uint8 和 DEPTH16 uint16 的 dimOS 消息。
`ur7e_camera_status` 只向 Agent 返回健康状态和内参，不返回像素；真实画面没有发送给模型。
相机与运动使用不同桥进程，不把图像压缩放进运动反馈/watchdog 线程。

## 白色盒子悬停：2026-10-02 离线验证

本次用户目标是先到桌面白色盒子上方，**不抓取、不操作夹爪**。
现场确认相机固定但移动过位置。旧的 `E2H_20260802` 外参在当前画面明显错位，
不再作为运动依据；原标定文件保留，未覆写或发布任何新的底座 TF。

已完成：

- 人工选定白色盒子顶面小块 RGB ROI，使用真实深度获得相机 optical 表面点
  `(0.3285, 0.4626, 1.4082) m`。这是 90 个深度样本的表面位置，
  **不是 Agent 自主识别、物体中心或完整物体姿态**。
- 使用工厂标定 URDF、实测六关节位置和单帧分割点云，通过工作站现有
  cuRobo 静态 ICP/SDF 拟合新的相机外参候选。投影的底座、肩部、肘部和末端
  与当前画面相应位置吻合。修剪点云的同帧留出残差约 3.6 mm，
  **不等于独立标定精度**，不能代替标定板/已知参考点验证。
- 候选外参估算白盒表面 `base_link=(-0.7543, -0.2291, 0.0614) m`，
  夹爪 TCP 悬停候选 `(-0.7543, -0.2291, 0.1614) m`，沿底座 +Z 高 10 cm。
- `demo_hover_plan.py` 在工作站仅执行离线 IK/轨迹优化。
  使用完整机械臂及夹爪模型、自碰撞检查、近似桌面/目标碰撞盒和 1 cm 球体膨胀，
  工具参考为 `gripper_tcp`，不是驱动的零偏移 `tool0`。
- 初始求解出现底座关节绕转约 317°；收窄离线求解的关节 winding 窗口后，
  获得 81 个插值点的朝下悬停候选，底座关节最大变化约 42.5°。
  腕部仍需约 174°/165° 变化；**不适用现有 5° 单关节测试门控**。

审计脚本 `demo_render_hover.py` 将夹爪 TCP 路径、目标表面和悬停点投影到
对应采集帧，明确标注 `OFFLINE PREVIEW - NOT EXECUTED`。
本轮新外参拟合与路径预览没有发送机器人运动命令，也没有发送现场像素给模型。
Mac 相机页面和只读 Agent/MCP 服务保持运行，末次状态六关节速度为零。

未完成、也是实际悬停前的必要条件：

1. 用独立标定板或已知三维参考点验证新外参，以及现场夹爪 TCP/安装偏移。
2. 补全当前台面其他物体和人的碰撞场景；现有两个近似碰撞盒不代表完整环境。
3. 独立接入有 watchdog、状态刷新、轨迹校验、低速重定时及现场确认的整轨迹执行。
   不得放宽单关节门控，或通过多次 5° 调用绕过它。

相关定向测试已通过：85 passed、4 个 unrelated whole-body 用例 deselected。
相机与几何核心严格定向 mypy 通过；离线 cuRobo 脚本在工作站真实运行通过，
Mac 对这些脚本的类型检查忽略了未安装的 cuRobo/PyYAML 第三方类型。
这不是全仓测试、独立标定验收或实际悬停成功。
