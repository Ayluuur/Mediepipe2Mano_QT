param(
    [Parameter(Mandatory=$true)][string]$QtRoot,
    [Parameter(Mandatory=$true)][string]$OpenCvRoot
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Open3DRoot = Join-Path $ProjectRoot 'ThirdParty\open3d-devel-windows-amd64-0.19.0'
$VsWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$VsRoot = & $VsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$Cmake = Join-Path $VsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
foreach ($PathToCheck in @($Cmake,(Join-Path $QtRoot 'bin\qmake.exe'),
    (Join-Path $OpenCvRoot 'OpenCVConfig.cmake'),(Join-Path $Open3DRoot 'CMake\Open3DConfig.cmake'))) {
    if (!(Test-Path -LiteralPath $PathToCheck)) { throw "Missing dependency: $PathToCheck" }
}
$ForwardQt = $QtRoot.Replace('\','/')
$ForwardOpenCv = $OpenCvRoot.Replace('\','/')
$ForwardOpen3D = $Open3DRoot.Replace('\','/')
$Preset = [ordered]@{
    version = 5
    cmakeMinimumRequired = [ordered]@{ major=3; minor=24; patch=0 }
    include = @('CMakePresets.json')
    configurePresets = @([ordered]@{
        name='local-vs2022-x64'; displayName='Local Visual Studio 2022 x64'; inherits='vs2022-x64'
        cacheVariables=[ordered]@{
            CMAKE_PREFIX_PATH="$ForwardQt;$ForwardOpen3D"
            OpenCV_DIR=$ForwardOpenCv
        }
    })
    buildPresets = @(
        [ordered]@{name='local-release';configurePreset='local-vs2022-x64';configuration='Release';jobs=4},
        [ordered]@{name='local-relwithdebinfo';configurePreset='local-vs2022-x64';configuration='RelWithDebInfo';jobs=4}
    )
    testPresets = @([ordered]@{
        name='local-tests';configurePreset='local-vs2022-x64';configuration='Release'
        output=[ordered]@{outputOnFailure=$true}
    })
}
$PresetPath = Join-Path $ProjectRoot 'CMakeUserPresets.json'
$Preset | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $PresetPath -Encoding utf8
$PropsPath = Join-Path $ProjectRoot 'vs\Local.Paths.props'
$Props = @"
<?xml version="1.0" encoding="utf-8"?>
<Project ToolsVersion="Current" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <PropertyGroup>
    <QtRoot>$ForwardQt</QtRoot>
    <OpenCvRoot>$ForwardOpenCv</OpenCvRoot>
    <Open3DRoot>$ForwardOpen3D</Open3DRoot>
  </PropertyGroup>
</Project>
"@
$Props | Set-Content -LiteralPath $PropsPath -Encoding utf8
Push-Location $ProjectRoot
try {
    & $Cmake --preset local-vs2022-x64
    if ($LASTEXITCODE) { throw 'Visual Studio CMake configuration failed' }
} finally { Pop-Location }
Write-Host "Visual Studio configuration created: $PresetPath"
Write-Host "Visual Studio IntelliSense paths created: $PropsPath"
Write-Host "Open the delivery solution: $ProjectRoot\MediaPipe2ManoQt.sln"
