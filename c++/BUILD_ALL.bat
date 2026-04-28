@echo off
REM
REM EvoDevo C++ Application Build Script
REM Windows MinGW x64 with Portable CMake
REM
REM This script compiles the EvoDevo physics simulation engine for Windows.
REM No administrator privileges required - all tools are portable.
REM
REM Usage: BUILD_ALL.bat
REM

echo.
echo ╔════════════════════════════════════════════════════════════════╗
echo ║          EvoDevo C++ Application - Windows Build               ║
echo ║                                                                ║
echo ║  Compiler: MinGW g++ 15.2.0                                   ║
echo ║  Physics:  Bullet Physics 2.82 (recompiled for Windows)       ║
echo ║  Graphics: FreeGLUT (recompiled for Windows)                  ║
echo ║  Build System: Portable CMake 4.3.2                           ║
echo ╚════════════════════════════════════════════════════════════════╝
echo.

REM Run the main build script from build/scripts/
call build\scripts\build_full_freeglut.bat

if %ERRORLEVEL% EQU 0 (
  echo.
  echo ╔════════════════════════════════════════════════════════════════╗
  echo ║                  BUILD COMPLETED SUCCESSFULLY                  ║
  echo ║                                                                ║
  echo ║  Executable: build\app.exe                                    ║
  echo ║                                                                ║
  echo ║  To run the application:                                      ║
  echo ║    Option 1: .\build\app.exe                                  ║
  echo ║    Option 2: python runit.py  (headless mode)                ║
  echo ╚════════════════════════════════════════════════════════════════╝
  echo.
  exit /b 0
) else (
  echo.
  echo ╔════════════════════════════════════════════════════════════════╗
  echo ║                    BUILD FAILED                               ║
  echo ║                                                                ║
  echo ║  Check build output above for errors.                        ║
  echo ║  See BUILD.md for troubleshooting help.                      ║
  echo ╚════════════════════════════════════════════════════════════════╝
  echo.
  exit /b 1
)
