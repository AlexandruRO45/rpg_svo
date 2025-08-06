# Exit on error
$ErrorActionPreference = "Stop"

Write-Output "🚀 Installing dependencies for Windows..."

# vcpkg is pre-installed on GitHub-hosted runners
C:\vcpkg\vcpkg.exe integrate install
C:\vcpkg\vcpkg.exe update
C:\vcpkg\vcpkg.exe install --triplet=x64-windows eigen3 opencv4[core,imgproc,imgcodecs] boost-serialization

Write-Output "✅ Dependencies installed."