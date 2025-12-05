# 🧭 SLAM-TOOLBOX-QT
> **SLAM-TOOLBOX-QT 是一个高性能、模块化、跨平台的 SLAM（Simultaneous Localization and Mapping）系统集成框架，旨在将前沿的 激光雷达-惯性紧耦合算法（如 Fast-LIO2） 与 直观的 Qt 图形用户界面 无缝融合，为机器人开发者、研究人员与工程师提供一个开箱即用的实时建图、定位与可视化平台。**

![2025-11-28_09-11.jpg](docs/2025-11-28_09-11.jpg)

---

## ✅ 特性亮点

- ✅ **模块化传感器架构**：支持任意自定义传感器（IMU / LiDAR / GNSS / …）通过继承 `ImuBase` / `LidarBase`
- ✅ **零拷贝数据流**：基于 **ZeroMQ IPC**（进程间通信）实现 Python ↔ C++ 高吞吐、低延迟数据管道
- ✅ **Fast-LIO2 后端**：紧耦合 LiDAR-IMU 前端 + EKF 后端，支持 `.json` 动态配置
- ✅ **PCL + VTK 可视化**：3D 点云、轨迹、体素栅格实时渲染
- ✅ **vcpkg 全依赖管理**：开箱即用的依赖列表（[vcpkg_dependencies.txt](vcpkg_dependencies.txt)）

---

## 🔧 核心架构设计
本项目采用 分层解耦 + 多线程异步通信 的现代软件架构：

### 业务逻辑层（SLAMBase）
纯 QObject 调度中枢，无任何 GUI 依赖 

统一管理 IMU、LiDAR、Camera 等多源传感器实例 

支持单线程调试模式与多线程高性能模式（各传感器独立线程） 

通过 Qt 信号槽机制实现模块间零拷贝、跨线程安全通信
### 算法接口层（AlgorithmMainBase）
定义标准 SLAM 算法抽象接口

输出三类核心数据流：
> Odometry：高频率位姿估计（6-DoF）
> 
> Path：全局轨迹（用于回环校正后优化）
> 
> PointCloudMsg：稠密/稀疏点云地图

### 可视化交互层（SLAM GUI）
基于 QMainWindow 构建响应式界面

集成 VTK + PCLVisualizer 实现 3D 实时渲染：

动态点云更新

机器人位姿坐标系跟踪

轨迹可视化

网格地面与世界坐标系辅助



## 📦 依赖安装（vcpkg）

本项目**推荐使用 vcpkg + 动态链接 triplet `x64-linux-dynamic`**（因 QtMultimedia/VTK/PCL 可视化依赖插件系统）

### 1️⃣ 一键安装全部依赖

```bash
# 确保 vcpkg 已初始化（若未 bootstrap，会自动完成）
/path/to/vcpkg install @vcpkg_dependencies.txt
```

> 💡 `vcpkg_dependencies.txt` 中已包含全部所需库及特性（Qt6.9.1 + OpenCV4 + VTK9.3 + PCL1.15 + ZeroMQ + nlohmann_json + Eigen3 + FFmpeg + GStreamer + ALSA + PulseAudio 等）

### 2️⃣ 手动安装（推荐校验）

```bash
vcpkg install \
    qtbase[gui,widgets,network,sql,sql-sqlite,sql-psql,dbus,opengl,xcb,xkbcommon-x11,freetype,fontconfig,harfbuzz,openssl,doubleconversion,jpeg,png,zstd]:x64-linux-dynamic \
    qtmultimedia[gstreamer,widgets]:x64-linux-dynamic \
    qttools[linguist]:x64-linux-dynamic \
    opencv4[gtk,jpeg,png,tiff,webp,calib3d,highgui]:x64-linux-dynamic \
    vtk[qt,opengl,proj,cgns,netcdf]:x64-linux-dynamic \
    pcl[qt,visualization]:x64-linux-dynamic \
    zeromq:x64-linux-dynamic \
    nlohmann-json:x64-linux-dynamic \
    eigen3:x64-linux-dynamic \
    ffmpeg[avcodec,avformat,swscale,swresample]:x64-linux-dynamic \
    gstreamer[plugins-base,plugins-bad]:x64-linux-dynamic \
    alsa:x64-linux-dynamic \
    pulseaudio:x64-linux-dynamic
```

---

## 🛠 编译构建（推荐校验）

```bash
mkdir build && cd build
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_TARGET_TRIPLET=x64-linux-dynamic \
  -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

> ⚠️ **务必启用 `vcpkg.cmake` 工具链**，否则 CMake 无法找到 vcpkg 安装的库。

---

## ▶️ 运行示例

### 1️⃣ 启动 Python 数据源（模拟 ROS bag 播放）

```bash
# 播放 bag 并通过 ZeroMQ 发布 IMU + LiDAR
python examples/ros/RosBagReader.py
```

> 📝 `RosBagReader.py` 会创建两个 IPC 套接字：
> - `ipc:///tmp/imu_stream`
> - `ipc:///tmp/lidar_stream`

### 2️⃣ 启动 Qt 主程序

```bash
./fastlio_example
```

程序将：
- 连接 ZeroMQ 流
- 加载 `examples/fastlio.json` 配置
- 启动 Fast-LIO2 算法线程
- 启动 Qt 主循环 + PCL/VTK 渲染窗口

---

## 🧩 二次开发指南

### 📌 1. 添加自定义传感器

```cpp
class CustomGNSSReceiver : public SensorBase {
    Q_OBJECT
public:
    explicit CustomGNSSReceiver(QObject* parent = nullptr) : SensorBase(parent) {
        // 初始化 ZMQ / TCP / Serial 等
    }

public slots:
    void loop() override {
        // 1. 读取原始数据
        // 2. 解析为 std::shared_ptr<GNSS>
        // 3. emit sendGNSSData(gnssPtr);
    }
};
```

> ✅ 所有传感器只需实现 `loop()` 槽函数，框架自动以 ** QTimer 驱动轮询**（默认 1ms 间隔，可配置）

### 📌 2. 修改 Fast-LIO 参数

编辑 [`examples/fastlio.json`](examples/fastlio.json)，支持字段包括：
```json
{
  "lidar_type": 0,
  "N_SCANS": 6,
  "point_filter_num": 3,
  "blind": 4.0,
  "time_unit": 0,
  "SCAN_RATE": 10,

  "NUM_MAX_ITERATIONS": 4,
  "extrinsic_est_en": true,

  "extrinsic_T": [0.04165, 0.02326, -0.0284],
  "extrinsic_R": [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
  ],

  "runtime_pos_log": false
}
```

### 📌 3. 替换/扩展 SLAM 后端

```cpp
auto myCeresLIO = std::make_shared<CeresLioMain>();  // 自定义类
myCeresLIO->initParams(cfg);

slam->addAlgorithmInstance(myCeresLIO);  // 替换 FastLioMain
```

> 框架支持多算法并行（调试比对），只需派生 `AlgorithmMainBase` 并实现 对应传感器callback和loop函数。

---

## 📂 项目结构

```
├── CMakeLists.txt                # 主构建脚本（目标：examples/fastlio_example）
├── docs/                         # 截图/文档
├── examples/
│   ├── fastlio_example.cpp       # 主程序入口（含 main()）
│   ├── fastlio.json              # Fast-LIO 参数配置
│   └── mainwindow.ui             # Qt Designer 界面文件 ✅
│   └── ros/                      # ROSbag 工具链
├── ikd-Tree/                     # Incremental KD-Tree (Fast-LIO 核心)
├── IKFoM_toolkit/                # Error-State EKF on Manifold
├── Msgs/dataTypes.h              # 自定义消息结构（IMU/LidarFrame 等）
├── Sensors/
│   ├── SensorBase.h              # 传感器基类（含虚函数 loop()）
│   └── SensorType.h              # 消息类型定义（IMU / LidarFrame）
├── SLAM/                         # 核心调度层
│   ├── SLAMBase.{h,cpp}          # 传感器/算法注册、信号连接中枢
│   ├── AlgorithmMainBase.h       # SLAM 算法抽象接口
│   └── FastLio/                  # Fast-LIO2 具体实现
└── tools/
    ├── Exp_Math.h                # 数学工具（SO3/SE3 等）
    └── utills.h                  # JSON ↔ 结构体转换、点云处理
```

---

## 📜 License

MIT License — See [LICENSE](LICENSE)

---

## 🙏 致谢

- [Fast-LIO2](https://github.com/hku-mars/FAST_LIO) — HKU-Mars
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) — Incremental KD-Tree
- [Qt](https://www.qt.io/) — GUI & Threading
- [PCL](https://pointclouds.org/) / [VTK](https://vtk.org/) — 3D Visualization
- [ZeroMQ](https://zeromq.org/) — High-performance messaging

---

> 🌟 欢迎 PR / Issue！
