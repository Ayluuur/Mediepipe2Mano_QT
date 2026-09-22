param(
    [Parameter(Mandatory=$true)][string]$Target,
    [ValidateSet('Release','RelWithDebInfo')][string]$Configuration = 'Release',
    [switch]$Rebuild,
    [switch]$Clean
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$UserPreset = Join-Path $ProjectRoot 'CMakeUserPresets.json'
if (!(Test-Path -LiteralPath $UserPreset)) {
    throw 'Missing CMakeUserPresets.json. Run tools/configure_visual_studio.ps1 once with the local Qt and OpenCV paths.'
}
$VsWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$VsRoot = & $VsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$Cmake = Join-Path $VsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
if (!(Test-Path -LiteralPath $Cmake)) { throw "Visual Studio CMake not found: $Cmake" }
$BuildPreset = if ($Configuration -eq 'Release') {'local-release'} else {'local-relwithdebinfo'}
$Mutex = [Threading.Mutex]::new($false,'Local\MediaPipe2ManoQt-CMakeBuild')
try {
    if (!$Mutex.WaitOne([TimeSpan]::FromMinutes(30))) { throw 'Timed out waiting for another CMake build.' }
    Push-Location $ProjectRoot
    try {
        $Cache = Join-Path $ProjectRoot 'out\build\local-vs2022-x64\CMakeCache.txt'
        if (!(Test-Path -LiteralPath $Cache)) {
            & $Cmake --preset local-vs2022-x64
            if ($LASTEXITCODE) { throw 'CMake configure failed' }
        }
        if ($Target -eq 'RUN_TESTS' -and !$Clean) {
            & $Cmake --build --preset $BuildPreset --target parity_tests sync_parity_tests skeleton_parity_tests config_profiles_tests
            if ($LASTEXITCODE) { throw 'Building test targets failed' }
            $Ctest = Join-Path (Split-Path -Parent $Cmake) 'ctest.exe'
            & $Ctest --preset local-tests
        } elseif ($Clean) {
            & $Cmake --build --preset $BuildPreset --target clean
        } else {
            $Arguments = @('--build','--preset',$BuildPreset,'--target',$Target)
            if ($Rebuild) { $Arguments += '--clean-first' }
            & $Cmake @Arguments
        }
        if ($LASTEXITCODE) { throw "CMake target failed: $Target" }
    } finally { Pop-Location }
} finally {
    try { $Mutex.ReleaseMutex() } catch {}
    $Mutex.Dispose()
}
