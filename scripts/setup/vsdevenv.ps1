<#
.SYNOPSIS
  Imports the Visual Studio x64 developer environment (cl, cmake, ninja, clang-cl, clang-tidy) into the
  current PowerShell session. Visual Studio is located with vswhere, so no path is hard-coded.

.EXAMPLE
  . .\scripts\setup\vsdevenv.ps1
  cmake --preset msvc-release; cmake --build --preset msvc-release
#>
$ErrorActionPreference = 'Stop'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path $vswhere)) { throw 'vswhere.exe not found: install Visual Studio 2022 or the Build Tools.' }
$installation = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'No Visual Studio installation with the C++ x64 toolset was found.' }
$devcmd = Join-Path $installation 'Common7\Tools\VsDevCmd.bat'
$environment = cmd /c "`"$devcmd`" -arch=x64 -host_arch=x64 -no_logo && set"
foreach ($line in $environment) {
  if ($line -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($Matches[1])" -Value $Matches[2] }
}
$llvm = Join-Path $installation 'VC\Tools\Llvm\x64\bin'
if (Test-Path $llvm) { $env:PATH = "$llvm;$env:PATH" }
Write-Host "Visual Studio developer environment loaded from $installation"
