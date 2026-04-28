@echo off
REM Compilation script for EvoDevo C++ project with MinGW
REM Multi-stage build: compiles .cpp files to .o, then links

set MINGW_BIN=%~dp0mingw-20.0\MinGW\bin\
set PROJECT_DIR=%~dp0
set CXXFLAGS=-O0 -g -Wall -std=gnu++11
set INCLUDES=-I../bullet-2.82-r2704/Demos/OpenGL/ -I./more_libs/include/ -I../bullet-2.82-r2704/src/
set LIBS=-L../bullet-build/Demos/OpenGL/ -L../bullet-build/src/BulletDynamics/ -L../bullet-build/src/BulletCollision/ -L../bullet-build/src/LinearMath/ -L./more_libs/lib/ -lOpenGLSupport -lopengl32 -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath

echo Compiling EvoDevo app with g++ (multi-stage build)...
cd /d "%PROJECT_DIR%"

REM Clean old object files
del /f main.o DemoApplication.o GlutDemoApplication.o GlutStuff.o 2>nul

REM Compile main.cpp
echo Compiling main.cpp...
"%MINGW_BIN%g++.exe" -c %CXXFLAGS% %INCLUDES% ./main.cpp -o main.o
if %ERRORLEVEL% NEQ 0 goto error

REM Compile DemoApplication.cpp
echo Compiling DemoApplication.cpp...
"%MINGW_BIN%g++.exe" -c %CXXFLAGS% %INCLUDES% ../bullet-2.82-r2704/Demos/OpenGL/DemoApplication.cpp -o DemoApplication.o
if %ERRORLEVEL% NEQ 0 goto error

REM Compile GlutDemoApplication.cpp
echo Compiling GlutDemoApplication.cpp...
"%MINGW_BIN%g++.exe" -c %CXXFLAGS% %INCLUDES% ../bullet-2.82-r2704/Demos/OpenGL/GlutDemoApplication.cpp -o GlutDemoApplication.o
if %ERRORLEVEL% NEQ 0 goto error

REM Compile GlutStuff.cpp
echo Compiling GlutStuff.cpp...
"%MINGW_BIN%g++.exe" -c %CXXFLAGS% %INCLUDES% ../bullet-2.82-r2704/Demos/OpenGL/GlutStuff.cpp -o GlutStuff.o
if %ERRORLEVEL% NEQ 0 goto error

REM Link all object files
echo Linking app.exe...
"%MINGW_BIN%g++.exe" %CXXFLAGS% main.o DemoApplication.o GlutDemoApplication.o GlutStuff.o %LIBS% -o ./app.exe
if %ERRORLEVEL% NEQ 0 goto error

echo.
echo ========== BUILD SUCCESSFUL ==========
echo Executable created: app.exe
echo To run: .\app.exe [arguments]
exit /b 0

:error
echo.
echo ========== BUILD FAILED ==========
echo Check errors above
exit /b 1
