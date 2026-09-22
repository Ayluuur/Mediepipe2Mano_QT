param(
    [string]$Python = 'path/to/python.exe',
    [string]$OpenCvRoot = 'path/to/opencv/build',
    [string]$VisualCpp = 'path/to/Microsoft Visual Studio/2022/Community/VC',
    [string]$GitBash = 'path/to/Git/bin/bash.exe'
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$SourceRoot = Join-Path $ProjectRoot 'ThirdParty\mediapipe-0.10.9'
$Bazel = Join-Path $ProjectRoot 'ThirdParty\bazel.exe'
if (!(Test-Path -LiteralPath $Bazel) -or !(Test-Path -LiteralPath $SourceRoot)) { throw 'Run tools/install_dependencies.py first.' }
foreach ($PathToCheck in @($Python,(Join-Path $OpenCvRoot 'OpenCVConfig.cmake'),$GitBash)) {
    if (!(Test-Path -LiteralPath $PathToCheck)) { throw "Missing dependency: $PathToCheck" }
}
if (!(Test-Path -LiteralPath (Join-Path $VisualCpp 'Auxiliary\Build\vcvarsall.bat'))) { throw "Missing Visual C++ toolchain: $VisualCpp" }
& $Python (Join-Path $PSScriptRoot 'prepare_mediapipe.py') --root $ProjectRoot --opencv $OpenCvRoot
if ($LASTEXITCODE) { throw 'MediaPipe asset preparation failed' }
$env:BAZEL_VC = $VisualCpp
$env:BAZEL_SH = $GitBash
# Forward slashes avoid TensorFlow's Python code generation interpreting \U.
$env:PYTHON_BIN_PATH = $Python.Replace('\','/')
$env:PYTHON_LIB_PATH = ((Split-Path -Parent $Python) + '\Lib\site-packages').Replace('\','/')
$env:VSLANG = '1033'
$CacheRoot = (Join-Path $ProjectRoot 'bazel-cache').Replace('\','/')
Push-Location $SourceRoot
try {
    & $Bazel "--output_user_root=$CacheRoot" --batch build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --jobs=4 "--repo_env=PYTHON_BIN_PATH=$env:PYTHON_BIN_PATH" "--repo_env=PYTHON_LIB_PATH=$env:PYTHON_LIB_PATH" //mediapipe/examples/qt_bridge:mediapipe_hands.dll
    if ($LASTEXITCODE) { throw 'Native MediaPipe build failed; preserve the full build log for diagnosis.' }
    $BinDir = Join-Path $ProjectRoot 'Bin'
    New-Item -ItemType Directory -Path $BinDir -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $SourceRoot 'bazel-bin\mediapipe\examples\qt_bridge\mediapipe_hands.dll') -Destination $BinDir -Force
} finally { Pop-Location }
Write-Host 'Native MediaPipe DLL built. Run tools/build.ps1 to deploy the Qt application.'
