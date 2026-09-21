param(
    [string]$QtRoot = 'path\to\Qt\6.8.3\msvc2022_64',
    [string]$OpenCvRoot = 'path\to\OpenCV\opencv\build'
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$SdkRoot = Join-Path $ProjectRoot 'deps\open3d-devel-windows-amd64-0.19.0'
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
$BinDir = Join-Path $ProjectRoot 'bin'
New-Item -ItemType Directory -Force -Path $BinDir | Out-Null
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'build\Release\MediaPipe2ManoQt.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'build\Release\parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'build\Release\sync_parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'build\Release\skeleton_parity_tests.exe') -Destination $BinDir -Force
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'build\Release\config_profiles_tests.exe') -Destination $BinDir -Force
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
& (Join-Path $BinDir 'config_profiles_tests.exe') $ProjectRoot
if ($LASTEXITCODE) { throw 'Parallel configuration profile checks failed' }
Write-Host "Built and deployed: $BinDir\MediaPipe2ManoQt.exe"
