# MediaPipe2Mano Qt

## 目录结构

```text
MediaPipe2Mano_QT/
├─ assets/、models/        运行资源和 MANO 模型
├─ Bin/                    可执行文件及运行时 DLL
├─ configs/                运行配置
├─ mediapipe_bridge/       MediaPipe C ABI 桥接项目
├─ tests/                  独立测试项目源码
├─ tools/                  配置、构建和交付脚本
└─ ThirdParty/             Open3D、Bazel 等第三方开发依赖
```

本地构建包可直接从第5节看起。

## 构建

### 1. 下载项目

```powershell
git clone https://github.com/Ayluuur/Mediepipe2Mano_QT.git
cd .\Mediepipe2Mano_QT
```

后续命令均在 `Mediepipe2Mano_QT` 项目根目录中执行。

### 环境要求

- Windows x64
- Visual Studio 2022：MSVC v143、Windows SDK、CMake
- Qt 6.8.3：`msvc2022_64`
- OpenCV 4.12.0：Windows x64 SDK
- Git Bash：供 Bazel 构建 MediaPipe
- Python：安装 MediaPipe 0.10.9 和 NumPy 1.24.4

### 2. 下载依赖

```powershell
python .\tools\install_dependencies.py
```

### 3. 生成 MANO 模型和测试数据

将 Python 项目克隆到当前项目的同级目录。

```powershell
git clone --branch feature/mano-interaction --single-branch `
  https://github.com/Ayluuur/Mediapipe2Mesh.git `
  ..\Mediapipe2Mesh
```

根据 `..\Mediapipe2Mesh\requirements.txt` 配置 Python 环境，
再将 `$ReferencePython` 设为该环境的 `python.exe`。

以下工具会把 MANO 模型和测试参考数据生成到当前项目中。

```powershell
$ReferencePython = 'path/to/reference/python.exe'

& $ReferencePython .\tools\export_reference.py `
  --source ..\Mediapipe2Mesh `
  --output .

& $ReferencePython .\tools\export_sync_reference.py `
  --source ..\Mediapipe2Mesh `
  --output .

& $ReferencePython .\tools\export_skeleton_reference.py `
  --source ..\Mediapipe2Mesh `
  --output .
```

### 4. 构建 MediaPipe DLL

将参数替换为本机的 Python、OpenCV、Visual C++ 和 Git Bash 路径。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\build_mediapipe.ps1 `
  -Python "path/to/reference/python.exe" `
  -OpenCvRoot "path/to/opencv/build" `
  -VisualCpp "path/to/Microsoft Visual Studio/2022/Community/VC" `
  -GitBash "path/to/Git/bin/bash.exe"
```

### 5. 构建 Qt 程序

生成 `CMakeUserPresets.json`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\configure_visual_studio.ps1 `
  -QtRoot "path/to/Qt/6.8.3/msvc2022_64" `
  -OpenCvRoot "path/to/opencv/build"
```

使用以下命令或者VS的生成解决方案进行构建

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\build.ps1 `
  -QtRoot "path/to/Qt/6.8.3/msvc2022_64" `
  -OpenCvRoot "path/to/opencv/build"
```
脚本会编译程序、部署运行库并执行测试。
脚本构建结果位于 `Bin`；生成解决方案构建结果位于 `out\build\local-vs2022-x64\Release`



## 运行

| 模式 | 启动命令 |
|:---|:---|
| 预览 | `.\run_viewer.cmd` |
| 交互 | `.\run_interaction.cmd` |
| 并行 IK | `.\run_parallel_ik.cmd` |

也可以直接运行程序：

```powershell
.\Bin\MediaPipe2ManoQt.exe 
```

完整说明见[完整项目说明](docs/README_FULL.md)。
