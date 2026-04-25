#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(pwd)"

# ── Step 1: Clean & Configure ──────────────────────────────────────────────
# We remove the arm64 flags and let CMake detect your Intel CPU.
rm -rf bullet-build
mkdir bullet-build
cd bullet-build

# Note: We use "MinGW Makefiles" to ensure it works with g++ on Windows
cmake ../bullet-2.82-r2704 -G "MinGW Makefiles" \
  -DBUILD_DEMOS=OFF \
  -DBUILD_EXTRAS=OFF \
  -DBUILD_MULTITHREADING=OFF \
  -DUSE_MSVC_SSE=ON \
  -DCMAKE_BUILD_TYPE=Release

# ── Step 2: Build libraries ─────────────────────────────────────────────────
echo "--- Building Bullet Libraries ---"
mingw32-make -j8

cd "$REPO_ROOT"

# ── Step 3: Compile OpenGL support sources ──────────────────────────────────
# Windows uses -lopengl32 and -lfreeglut instead of Frameworks
echo "--- Compiling OpenGL Support ---"
g++ -std=c++11 -O2 \
  -I ./bullet-2.82-r2704/src/ \
  -I ./bullet-2.82-r2704/Demos/OpenGL/ \
  -c ./bullet-2.82-r2704/Demos/OpenGL/GlutDemoApplication.cpp -o GlutDemoApplication.o

g++ -std=c++11 -O2 \
  -I ./bullet-2.82-r2704/src/ \
  -I ./bullet-2.82-r2704/Demos/OpenGL/ \
  -c ./bullet-2.82-r2704/Demos/OpenGL/GLDebugDrawer.cpp -o GLDebugDrawer.o

g++ -std=c++11 -O2 \
  -I ./bullet-2.82-r2704/src/ \
  -I ./bullet-2.82-r2704/Demos/OpenGL/ \
  -c ./bullet-2.82-r2704/Demos/OpenGL/DemoApplication.cpp -o DemoApplication.o

# ── Step 4: Link the simulation binary ─────────────────────────────────────
echo "--- Linking Final Binary ---"
g++ -std=c++11 -O2 \
  c++/main.cpp GlutDemoApplication.o GLDebugDrawer.o DemoApplication.o \
  -I ./bullet-2.82-r2704/src/ \
  -I ./bullet-2.82-r2704/Demos/OpenGL/ \
  -L ./bullet-build/src/BulletDynamics/ \
  -L ./bullet-build/src/BulletCollision/ \
  -L ./bullet-build/src/LinearMath/ \
  -lBulletDynamics -lBulletCollision -lLinearMath \
  -lfreeglut -lopengl32 -lglu32 -lgdi32 -lwinmm \
  -o c++/app.exe

echo "--- Build Complete: Run c++/app.exe ---"