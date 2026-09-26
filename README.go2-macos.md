# Go2 + macOS + DeepSeek 二开说明

这是 [dimensionalOS/dimos](https://github.com/dimensionalOS/dimos) 的实验性公开 Fork，保留上游 Apache-2.0 许可。适配基线为 `edd7c346b8158a406c915dfe922202dc2d652e21`。验证记录日期：2026-09-26。

本仓库公开源码、启动脚本和测试，不包含 API 密钥、设备 AES 密钥、SSH 凭据、实际相机画面、模型请求日志、Chroma 数据库或本地模型缓存。克隆后需要自行安装依赖、下载上游资源，并重新采集地图/空间记忆。

## 当前完成程度

| 能力 | macOS DimSim 公寓 | Go2 EDU 真机 |
| --- | --- | --- |
| DeepSeek Agent / 中文视觉问答 | 已验证工具调用和实际仿真图像 | 已验证一次真实相机问答；需明确允许上传画面 |
| 点云与建图 | 原生体素地图、代价地图、探索已测试 | ROS 2 只读点云桥到 Mac，原生体素/代价地图已接通 |
| 导航 | 已探索场景的厨房观察点回访成功 | 仅路径预览；真实起点为未知格时正确拒绝 |
| 空间记忆 | CPU CLIP + Chroma 持久化，修复排名/位姿对应 | 未接入仿真记忆，不复用仿真坐标 |
| 真实运动、动态避障、完整 VLN | 未做完整 benchmark，语义匹配仍可能错误 | 未开放、未验收，不保证后续功能自动正常 |

详细边界见 [实验记录](docs/go2-macos/validation.md)。这是开发中的感知/规划验证，不是可直接无人值守使用的产品。

## 安装与资源

以下命令用于新的克隆目录，不会连接机器人。当前已有工作环境通过验证，但完整的全新安装尚未验收。

前置条件：Apple Silicon macOS、Python 3.12、Git LFS、uv、Deno，以及上游安装文档要求的原生依赖（如 libjpeg-turbo、FFmpeg、PortAudio）。如需安装系统依赖，请先查看 [上游安装说明](docs/installation/osx.md)。

```zsh
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Fibonaccirabbit/dimos-go2.git
cd dimos-go2
uv venv --python 3.12
uv pip install --python .venv/bin/python -e '.[unitree,sim]'
uv pip install --python .venv/bin/python 'unitree-webrtc-connect==2.2.0'
source .venv/bin/activate
```

启动脚本默认使用仓库根目录的 `.venv`；已有环境位于其他位置时设置 `DIMOS_VENV_PATH`。`unitree` extra 已包含 base、mapping、Agent、Web 和 perception。测试环境实测使用 WebRTC 包 2.2.0，未改动其安装源码。

继承的 `.lfsconfig` 指向上游公开 LFS 服务；仅引用上游资产，不上传私人数据或镜像全部模型。按上游资源管理流程获取 DimSim 场景、URDF、CLIP 等实际需要的资源。首次运行可能下载资源并构建网页，离线新克隆不能保证开箱运行。参考 [LFS/数据管理](docs/development/large_file_management.md)。

## 模型凭据

启动脚本从 macOS Keychain 名为 `dimos-deepseek` 的通用密码项目读取 API key。请通过“钥匙串访问”保存自己的 key，不要把 key 写入代码、Git 配置、终端命令参数或提交内容。

当前适配使用 `https://api.deepseek.com`、`deepseek-flash` 和 Responses API；模型名和可用接口需要与自己的账号匹配。图像问答会向服务商发送 JPEG 并产生费用，不能把模型的回复当成真实动作成功证明。

## 公寓仿真

```zsh
zsh scripts/go2-macos/run-go2-cockpit.sh
# 浏览器打开 http://127.0.0.1:7780/
```

先观察/探索采集自己的空间记忆，再尝试语义导航；公开仓库不携带原实验的 370 张图像。当前 CLIP 查询优先使用英文描述，例如 `a kitchen with a refrigerator and kitchen cabinets`。导航必须检查 `navigation_status` 的 `goal_reached` 和实际观察，不能把“开始导航”视为到达。

```zsh
zsh scripts/go2-macos/send-go2-command.sh '停止移动，然后观察当前画面，用中文描述。'
zsh scripts/go2-macos/stop-go2-sim.sh
```

`stop-go2-sim.sh` 保留原脚本名，实际执行 `dimos stop`，也可停止本机真机观察服务；它不是物理急停。每次切换仿真/真机必须先停止上一个服务。

旧 Rerun 入口为 `zsh scripts/go2-macos/run-go2-capabilities.sh legacy`；简化红蓝物体实验为 `zsh scripts/go2-macos/run-go2-deepseek-sim.sh`。未证明旧 Rerun 抽搐已根治，默认推荐原生 Cockpit。

仿真证据采集工具仅用于评估，不向 Agent 注入场景真值：

```zsh
python -m dimos.robot.unitree.go2.demo_capture_apartment --output artifacts/apartment-check --seconds 5
# 仅在旧 empty 场景演示中使用；视觉标记不是物理碰撞体：
python -m dimos.robot.unitree.go2.demo_capture_vision --seed left --output artifacts/vision-check
```

## Go2 EDU：只读网线接入

网络示例沿用 EDU 常见地址：Mac 有线网卡 `192.168.123.99/24`，不设置网关/DNS，保留 Wi-Fi 的互联网默认路由；Go2 主控 `192.168.123.161`，Jetson `192.168.123.18`。必须先确认自己的设备地址和网卡；不要机械覆盖已有配置。

SSH 只运行临时 ROS 2 订阅器取点云，建图、代价地图、Agent 和规划仍在 Mac 上。Jetson 要已有 ROS 2 / CycloneDDS、numpy、`/utlidar/cloud_deskewed` 和正确的 `odom` frame。默认 ROS 环境为 `/home/unitree/cyclonedds_ws/install/setup.bash`，网卡为 `eth0`；不同机型/固件应调整模块配置并验证坐标关系。这里 `odom` 到 `world` 仅为同坐标系别名，不是通用标定。

先在终端验证 SSH 主机指纹、建立已知主机信任，再建立连接（密码只在终端输入，不保存到脚本）：

```zsh
ssh -M -S /private/tmp/dimos-go2-edu-readonly.sock \
  -o ControlPersist=600 -o StrictHostKeyChecking=yes -fNT unitree@192.168.123.18
zsh scripts/go2-macos/run-go2-real-observe.sh 192.168.123.161
```

默认只允许本地预览和传感器状态查询，prompt 禁止向模型发送真机画面。**该图像许可限制在 prompt 层，不是传输层隐私护栏**；不应把它当成强制访问控制。若现场人员/运营方已经明确同意画面上传，可使用第二个参数 `--allow-vision`。这个参数不开放运动。

```zsh
# 只读、安全的状态指令；不发送图像：
zsh scripts/go2-macos/send-go2-command.sh '读取电量、点云和导航预览状态，不要移动。'
# 只显示拟议路线，不发送速度；未知或过期地图会拒绝：
zsh scripts/go2-macos/send-go2-command.sh '只预览从实际当前位置向前0.5米的路径，不要移动。'
```

本入口 `read_only=True`：不自动站立、不切换运动模式、不订阅速度命令、关闭时不趴下；连接层拒绝控制请求。规划预览不含 `cmd_vel` / `nav_cmd_vel`，没有 MovementManager 或探索控制。0.5 m footprint 为保守代理，未完成实际尺寸/足式动力学验证。

换电/断网前停止观察服务、关闭 SSH 主连接。重启后必须重新取得新鲜里程计、重建地图，不复用旧 odom 坐标；只读模式不是急停，也不能阻止遥控器/机载程序自行运动。

```zsh
zsh scripts/go2-macos/stop-go2-sim.sh
ssh -S /private/tmp/dimos-go2-edu-readonly.sock -O exit unitree@192.168.123.18
```

## 测试与维护

相关隔离单元测试不需要真实机器人或付费 API，示例：

```zsh
uv pip install --python .venv/bin/python pytest pytest-mock pytest-timeout
CI=1 python -m pytest -o addopts='' --import-mode=importlib \
  dimos/models/vl/test_deepseek.py \
  dimos/agents/skills/test_navigation_status_unit.py \
  dimos/perception/experimental/test_image_embedding_contract.py \
  dimos/perception/experimental/test_spatial_memory_persistence.py \
  dimos/perception/experimental/test_spatial_vector_db.py \
  dimos/robot/unitree/test_connection_readonly.py \
  dimos/robot/unitree/go2/test_connection_readonly.py \
  dimos/robot/unitree/go2/test_ros2_lidar.py \
  dimos/navigation/go2/test_planning_preview.py
```

模块/RPC 测试需要本机共享内存和本地通信权限；受限 sandbox 失败需与逻辑失败区分。未运行全仓测试、完整 mypy 或完整真机运动集成验收。二开测试按仓库 `python-unit-tests` 规范隔离外部机器人和模型服务，不代表实体安全认证。

继续开发使用 `<who>/<type>/<topic>` 分支，`upstream` 指向原项目，`origin` 指向本 Fork。不要直接推送上游 `main`，也不要将私人现场证据加入公开仓库。
