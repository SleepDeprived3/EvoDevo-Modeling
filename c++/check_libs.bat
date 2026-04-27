@echo off
setlocal enabledelayedexpansion

set MINGW_BIN=c:\Users\khatch\Documents\GitHub\EvoDevo-Modeling\c++\mingw-20.0\MinGW\bin\

echo Checking libglut.a symbols...
"%MINGW_BIN%nm.exe" more_libs\lib\libglut.a | findstr "__glutInitWithExit" > nul
if %ERRORLEVEL% EQU 0 (
  echo Found __glutInitWithExit in libglut.a (GOOD)
) else (
  echo NOT FOUND __glutInitWithExit in libglut.a (BAD)
)

echo.
echo Checking libOpenGLSupport.a symbols...
"%MINGW_BIN%nm.exe" ..\bullet-build\Demos\OpenGL\libOpenGLSupport.a 2>&1 | findstr "btCollisionObject" | head -5

echo.
echo Checking libBulletDynamics.a symbols...
"%MINGW_BIN%nm.exe" ..\bullet-build\src\BulletDynamics\libBulletDynamics.a 2>&1 | findstr "btDiscreteDynamicsWorld" | head -5
