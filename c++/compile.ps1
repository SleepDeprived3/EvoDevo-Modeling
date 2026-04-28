# Compilation script for EvoDevo C++ project with MinGW

# Configure these paths for your MinGW installation:
# MinGW is extracted to mingw-20.0\MinGW\bin
$PROJECT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$MINGW_PATH = Join-Path $PROJECT_DIR "mingw-20.0\MinGW\bin"

# Add MinGW to PATH
$env:PATH = "$MINGW_PATH;$env:PATH"

Write-Host "Compiling EvoDevo app with g++..." -ForegroundColor Cyan

# Navigate to project directory
Set-Location $PROJECT_DIR

# Compile
& g++ -O2 -g -Wall -std=gnu++11 `
  -I../bullet-2.82-r2704/Demos/OpenGL/ `
  -I./more_libs/include/ `
  -I../bullet-2.82-r2704/src/ `
  ./main.cpp `
  -L../bullet-build/Demos/OpenGL/ `
  -L../bullet-build/src/BulletDynamics/ `
  -L../bullet-build/src/BulletCollision/ `
  -L../bullet-build/src/LinearMath/ `
  -L./more_libs/lib/ `
  -lOpenGLSupport -lGL -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath `
  -o ./app.exe

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n========== BUILD SUCCESSFUL ==========" -ForegroundColor Green
    Write-Host "Executable created: app.exe" -ForegroundColor Green
    Write-Host "To run: .\app.exe [arguments]" -ForegroundColor Green
} else {
    Write-Host "`n========== BUILD FAILED ==========" -ForegroundColor Red
    Write-Host "Check errors above" -ForegroundColor Red
    Read-Host "Press Enter to continue"
}
