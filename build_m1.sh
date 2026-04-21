#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO_ROOT"

OGL_SRC="$REPO_ROOT/bullet-2.82-r2704/Demos/OpenGL"
BT_SRC="$REPO_ROOT/bullet-2.82-r2704/src"
BUILD="$REPO_ROOT/bullet-build"
TMP="$BUILD/_ogl_objs"

# --- Step 1: Clean & Configure ---
echo "--- [1/5] Configuring Bullet 2.82 for arm64 ---"
rm -rf "$BUILD"
mkdir -p "$BUILD"
cd "$BUILD"

cmake "$REPO_ROOT/bullet-2.82-r2704" \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DBUILD_DEMOS=OFF \
  -DBUILD_EXTRAS=OFF \
  -DBUILD_MULTITHREADING=OFF \
  -DBUILD_MINICL_SUPPORT=OFF \
  -DUSE_GRAPHICAL_BENCHMARK=OFF \
  -DCMAKE_CXX_FLAGS="-DBT_USE_NEON=0 -DBT_USE_SSE=0 -DBT_NO_SIMD_OPERATOR_OVERLOADS"

# --- Step 2: Build Bullet libraries ---
# Build only the three required libs - BulletSoftBody is not needed and has extra SSE issues
echo "--- [2/5] Building Bullet libraries (targeted, no SoftBody) ---"
make -j8 BulletDynamics BulletCollision LinearMath

for lib in \
  "$BUILD/src/LinearMath/libLinearMath.a" \
  "$BUILD/src/BulletCollision/libBulletCollision.a" \
  "$BUILD/src/BulletDynamics/libBulletDynamics.a"; do
  if [ ! -f "$lib" ]; then
    echo "ERROR: Expected library not found: $lib"
    exit 1
  fi
done
echo "  OK: All three Bullet .a libraries confirmed"

# --- Step 3: Compile OpenGL support sources ---
echo "--- [3/5] Compiling OpenGL support objects ---"
cd "$REPO_ROOT"
mkdir -p "$TMP"

COMMON_FLAGS="-std=c++11 -DGL_SILENCE_DEPRECATION -O2 -arch arm64 -I$BT_SRC -I$OGL_SRC"

for src in GlutDemoApplication.cpp DemoApplication.cpp GLDebugDrawer.cpp GL_ShapeDrawer.cpp; do
  echo "  compiling $src ..."
  g++ $COMMON_FLAGS -c "$OGL_SRC/$src" -o "$TMP/${src%.cpp}.o"
done
echo "  OK: OpenGL objects compiled"

# --- Step 4: Link the simulation binary ---
echo "--- [4/5] Linking c++/app ---"

g++ $COMMON_FLAGS \
  "$REPO_ROOT/c++/main.cpp" \
  "$TMP/GlutDemoApplication.o" \
  "$TMP/DemoApplication.o" \
  "$TMP/GLDebugDrawer.o" \
  "$TMP/GL_ShapeDrawer.o" \
  -L"$BUILD/src/BulletDynamics/" \
  -L"$BUILD/src/BulletCollision/" \
  -L"$BUILD/src/LinearMath/" \
  -lBulletDynamics -lBulletCollision -lLinearMath \
  -framework OpenGL -framework GLUT \
  -o "$REPO_ROOT/c++/app"

# --- Step 5: Verify ---
echo "--- [5/5] Verifying binary ---"
chmod +x "$REPO_ROOT/c++/app"
file "$REPO_ROOT/c++/app"
echo "--- Build complete ---"