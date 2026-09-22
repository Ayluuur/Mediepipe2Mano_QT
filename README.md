# MediaPipe2Mano Qt

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
$ReferencePython = 'C:\path\to\reference\python.exe'

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

将参数替换为本机的 Python、OpenCV 和 Visual C++ 路径。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\build_mediapipe.ps1 `
  -Python "C:\path\to\reference\python.exe" `
  -OpenCvRoot "C:\path\to\opencv\build" `
  -VisualCpp "C:\Program Files\Microsoft Visual Studio\2022\Community\VC"
```

### 5. 构建 Qt 程序

脚本会编译程序、部署运行库并执行测试。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\build.ps1 `
  -QtRoot "C:\path\to\Qt\6.8.3\msvc2022_64" `
  -OpenCvRoot "C:\path\to\opencv\build"
```

构建结果位于 `bin`。

## 运行

| 模式 | 启动命令 |
|:---|:---|
| 预览 | `.\run_viewer.cmd` |
| 交互 | `.\run_interaction.cmd` |
| 并行 IK | `.\run_parallel_ik.cmd` |

也可以直接运行程序：

```powershell
.\bin\MediaPipe2ManoQt.exe --mode viewer
.\bin\MediaPipe2ManoQt.exe --mode interaction
.\bin\MediaPipe2ManoQt.exe --mode parallel_ik
```

完整说明见[完整项目说明](docs/README_FULL.md)。
