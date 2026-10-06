# dimOS 机器人平台（macOS）

一个入口启动任一已适配机器人；每个机器人都会打开同一形式的 Web Cockpit
<http://127.0.0.1:7780/>：画面 + 地图/状态 + Agent 对话，另有全屏相机和 Stats 页。

**必须用 Chrome / Edge 打开**：Cockpit 走 WebTransport，Safari 不支持，会一直停在
“Waiting for a robot to register...”。每次重启机器人服务后，旧标签页要刷新一次。

```zsh
zsh scripts/platform/dimos-platform.sh go2-sim                       # Go2 公寓仿真：建图、导航、空间记忆、Agent
zsh scripts/platform/dimos-platform.sh go2-real [--allow-vision]     # Go2 EDU 真机（网线），只读传感器
zsh scripts/platform/dimos-platform.sh ur7e [--allow-motion] [--allow-vision]   # UR7e 机械臂（经 Ubuntu 工作站）
zsh scripts/platform/dimos-platform.sh status | stop
```

同一时间只运行一个机器人，切换时脚本会先停止当前实例。dimOS 虚拟环境默认为 `<repo>/.venv`，
可用 `DIMOS_VENV_PATH` 指定。模型统一用 DeepSeek，Key 来自 macOS 钥匙串 `dimos-deepseek`。

| 机器人 | Cockpit 面板 | Agent 能力 | 前置条件 |
|---|---|---|---|
| Go2 仿真 | 相机、3D 体素图、2D 代价地图、键盘遥控、对话 | 观察、语义导航、记忆检索、探索 | 无 |
| Go2 EDU 真机 | 真机相机、ROS 2 点云体素图、代价地图/路径预览、对话 | 观察、电量、路径预览（不运动） | 见 [README.go2-macos.md](../README.go2-macos.md) |
| UR7e | 工作区相机、最近定位目标、对话、Stats | 状态、目标定位、实时规划悬停 / 复位、≤5° 关节测试 | 见下文 |

## 录制

Cockpit 顶栏的 **● record** 用浏览器原生标签页捕获录制当前页面，停止后下载
`dimos-<robot>-<时间>.mp4`。Chrome 录出的是分片 MP4（头部时长为 0，QuickTime 会显示为静止画面），
下载前会在浏览器内重封装为标准 MP4（不重新编码）。录像只保存在本机。

## UR7e

### 准备

- 工作站：ROS 2 Humble + 官方 UR driver，驱动已启动并激活 `scaled_joint_trajectory_controller`；
  示教器运行 External Control 程序（本地模式下无法远程启动）。桥部署见 [ur7e/bringup.md](ur7e/bringup.md)。
- 工作站上有 cuRobo checkout（含 `.venv`），并部署 `demo_hover_plan.py`、`demo_hover_execute.py`。
- Mac：已认证的 SSH ControlMaster（密码只在终端输入）。
- 本机配置写在 `~/.config/dimos/ur7e.env`（不进仓库）：

```sh
UR7E_SSH_TARGET=user@workstation
UR7E_REMOTE_ROOT=/home/user/.local/share/dimos/ur7e-bridge/<version>
UR7E_CONTROL_PATH=/tmp/dimos-ur7e-ssh/control
UR7E_CUROBO_ROOT=/home/user/curobo
UR7E_CAMERA_CALIBRATION=/path/to/static_candidates_refined.json
```

### 零样本流程

`dimos-platform.sh ur7e --allow-motion --allow-vision` 后在对话里说“移动到白色盒子上方”：

1. `ur7e_locate_object(description)`：同一帧 RGB 发给 DeepSeek 视觉，3 路并行、关闭思考、
   要求 0–1000 归一化坐标，按中心一致性取中位框（约 1–2 s）；框内深度重投影后用相机→机械臂外参
   转到基座系，只取最高的一层（物体顶面）。
2. `ur7e_hover_above_point(point)`：工作站按当前实测关节实时 cuRobo 规划到顶面上方（默认 0.1 m，
   夹爪朝下），超出 0.25–0.85 m 水平可达范围直接返回 `OUT_OF_REACH`，不发送轨迹。
3. 执行器只在实测姿态与轨迹起点一致（≤0.5°）且静止时发送，关节速度 ≤0.25 rad/s。
4. `ur7e_return_to_start` 沿上一条轨迹原路返回。

没有预置目标或轨迹；旧的固定轨迹 `ur7e_hover_above_white_box` 仅保留给脚本，对 Agent 隐藏。

### 限制

- 相机→机械臂外参是单姿态粗标定（未用标定板验证），悬停可能偏几厘米；相机移动后必须重新标定。
- 黑色/深色表面深度噪声大，顶面高度可能偏低。
- 规划只含自碰撞、桌面和目标框，不含人员和其他物体。软件停止只取消本连接的轨迹，
  不能代替硬件急停；运动时现场必须有人监护。
- `--allow-vision` 会把工作区画面（可能含人员）发给 DeepSeek；默认不发送。
