param(
    [string]$QtRoot = 'path\to\Qt\6.8.3\msvc2022_64',
    [string]$OpenCvRoot = 'path\to\OpenCV\opencv\build'
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$SdkRoot = Join-Path $ProjectRoot 'ThirdParty\open3d-devel-windows-amd64-0.19.0'
$VsWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$VsRoot = & $VsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$Cmake = Join-Path $VsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
foreach ($PathToCheck in @($Cmake,(Join-Path $QtRoot 'bin\qmake.exe'),(Join-Path $OpenCvRoot 'OpenCVConfig.cmake'),(Join-Path $SdkRoot 'CMake\Open3DConfig.cmake'))) {
    if (!(Test-Path -LiteralPath $PathToCheck)) { throw "Missing dependency: $PathToCheck. See README.md and tools/install_dependencies.py." }
}
& $Cmake -S $ProjectRoot -B (Join-Path $ProjectRoot 'build') -G 'Visual Studio 17 2022' -A x64 "-DCMAKE_PREFIX_PATH=$QtRoot;$SdkRoot" "-DOpenCV_DIR=$OpenCvRoot"
if ($LASTEXITCODE) { throw 'CMake configure failed' }
& $Cmake --build (Join-Path $ProjectRoot 'build') --config Release --parallel 4
if ($LASTEXITCODE) { throw 'C++ build failed' }
$BinDir = Join-Path $ProjectRoot 'Bin'
New-Item -ItemType Directory -Force -Path $BinDir | Out-Null
$BuildOutput = Join-Path $ProjectRoot 'build\Release'
Copy-Item -LiteralPath (Join-Path $BuildOutput 'MediaPipe2ManoQt.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $BuildOutput 'parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $BuildOutput 'sync_parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $BuildOutput 'skeleton_parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $BuildOutput 'config_profiles_tests.exe') -Destination $BinDir -Force
# CMake's application post-build step produces a self-contained VS output.
# Mirror those runtime DLLs and Qt plugin directories into the delivery bin.
Get-ChildItem -LiteralPath $BuildOutput -File -Filter '*.dll' | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination $BinDir -Force
}
foreach ($PluginDir in @('generic','iconengines','imageformats','networkinformation','platforms','styles','tls')) {
    $SourcePluginDir = Join-Path $BuildOutput $PluginDir
    if (Test-Path -LiteralPath $SourcePluginDir) {
        Copy-Item -LiteralPath $SourcePluginDir -Destination $BinDir -Recurse -Force
    }
}
foreach ($Name in @('Open3D.dll','tbb12.dll')) { Copy-Item -LiteralPath (Join-Path $SdkRoot "bin\$Name") -Destination $BinDir -Force }
Get-ChildItem -LiteralPath (Join-Path $OpenCvRoot 'x64\vc16\bin') -Filter '*.dll' | Where-Object { $_.Name -notmatch 'd\.dll$' } | ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $BinDir -Force }
& (Join-Path $QtRoot 'bin\windeployqt.exe') --release --no-translations (Join-Path $BinDir 'MediaPipe2ManoQt.exe')
if ($LASTEXITCODE) { throw 'Qt deployment failed' }
& (Join-Path $BinDir 'parity_tests.exe') $ProjectRoot
if ($LASTEXITCODE) { throw 'Python/C++ parity checks failed' }
& (Join-Path $BinDir 'sync_parity_tests.exe') $ProjectRoot
if ($LASTEXITCODE) { throw 'Current Python synchronization checks failed' }
& (Join-Path $BinDir 'skeleton_parity_tests.exe') $ProjectRoot
if ($LASTEXITCODE) { throw 'Skeleton and IK synchronization checks failed' }
Write-Host "Built and deployed: $BinDir\MediaPipe2ManoQt.exe"
