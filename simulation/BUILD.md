# EvoDevo C++ Application Build Guide

## Overview

The EvoDevo C++ application is a physics simulation engine built with Bullet Physics, Boost, and OpenGL/FreeGLUT for graphical visualization. This document provides instructions for building the application on Windows.

## System Requirements

- **OS**: Windows (tested on Windows 10/11)
- **Compiler**: MinGW g++ (included in `mingw-20.0/` directory)
- **Build Tools**: Portable CMake 4.3.2 (included in `Documents/CMake/`)
- **No Admin Rights Required**: All tools are portable and run from the project directory

## Project Structure

```
c++/
├── *.cpp, *.h          # Main application source code
├── more_libs/          # Local OpenGL and other libraries
├── build/              # Build outputs (generated)
│   ├── app.exe         # Compiled executable
│   ├── scripts/        # Build helper scripts
│   ├── bullet-build-windows/    # Windows-compiled Bullet Physics
│   └── freeglut-local/          # Windows-compiled FreeGLUT
├── BUILD.md            # This file
├── .gitignore          # Git ignore rules
└── runit.py            # Python script to run the simulation
```

## Key Files

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point for the application |
| `NoiseWorld.cpp/.h` | Core physics simulation engine |
| `build/scripts/build_full_freeglut.bat` | Main build script (Windows) |
| `build/app.exe` | Compiled executable |

## Building from Source

### Method 1: Run Main Build Script (Recommended)

```bash
cd c++
.\build\scripts\build_full_freeglut.bat
```

This script will:
1. Compile Bullet Physics source files from `../bullet-2.82-r2704/src/`
2. Compile OpenGL/Bullet demo application files
3. Compile FreeGLUT headers with static library linking
4. Link all object files with Windows-compatible Bullet and FreeGLUT libraries
5. Generate `build/app.exe`

### Method 2: Step-by-Step Compilation

For debugging or custom builds:

```batch
# Compile Bullet libraries (done once)
cd build\scripts
cmake_build.bat

# Compile FreeGLUT (done once)
build_freeglut.bat

# Compile and link application
build_full_freeglut.bat
```

## Compiled Libraries

The build process generates Windows-compatible libraries:

### Bullet Physics (Windows MinGW)
- **Location**: `build/bullet-build-windows/lib/`
- **Libraries**:
  - `libBulletDynamics.a` (1.1 MB)
  - `libBulletCollision.a` (2.2 MB)
  - `libLinearMath.a` (154 KB)

### FreeGLUT (Windows MinGW)
- **Location**: `build/freeglut-local/freeglut-build/lib/`
- **Library**:
  - `libglut.a` (657 KB)

## Compiler Flags

The build uses the following key compiler flags:

```
-O0 -g -Wall -std=gnu++11 -DFREEGLUT_STATIC
```

- `-O0`: No optimization (debug build)
- `-g`: Debug symbols
- `-Wall`: All warnings
- `-std=gnu++11`: C++11 standard
- `-DFREEGLUT_STATIC`: Link FreeGLUT as static library

## Linker Libraries

The final executable links against:
- `lBulletDynamics` (Bullet physics dynamics)
- `-lBulletCollision` (Bullet collision detection)
- `-lLinearMath` (Bullet linear algebra)
- `-lglut` (FreeGLUT windowing/input)
- `-lopengl32` (Windows OpenGL)
- `-lglu32` (OpenGL utilities)
- `-lgdi32` (Windows graphics device interface)
- `-lwinmm` (Windows multimedia)
- `-lstdc++` (Standard C++ library)

## Running the Application

### Graphical Mode (default)

```bash
.\build\app.exe
```

The application will launch a window with the physics simulation visualization.

### Headless Mode (via Python)

```bash
python runit.py
```

This runs the simulation headlessly using the Python script.

## Troubleshooting

### Build Fails: "undefined reference to `...`"

**Cause**: Missing compiled library or object file  
**Solution**: Ensure all steps in the build script completed successfully. Check for errors in the build output.

### app.exe Won't Start

**Cause**: OpenGL/graphics driver issue  
**Solution**: Ensure your graphics drivers support OpenGL 2.1+

### Cannot Find MinGW

**Cause**: MinGW compiler not in expected location  
**Solution**: Verify `mingw-20.0/` directory exists in this folder. If not, reinstall from source.

## Development Notes

### Header Locations

- **Bullet Physics**: `../bullet-2.82-r2704/src/`
- **OpenGL/GLUT**: `../bullet-2.82-r2704/Demos/OpenGL/`
- **FreeGLUT**: `build/freeglut-local/freeglut-master/include/GL/`

### Compilation Order

The build script compiles in this order to resolve dependencies:
1. Bullet source files (LinearMath, BulletCollision, BulletDynamics)
2. Bullet demo application files (DemoApplication, GlutDemoApplication, GlutStuff, GL_ShapeDrawer, GLDebugFont)
3. EvoDevo application files (main, NoiseWorld)
4. Linking all .o files with precompiled libraries

### Code Changes

If you modify any `.cpp` files, you must rebuild:

```batch
cd build\scripts
.\build_full_freeglut.bat
```

The script automatically recompiles all modified source files and regenerates `build/app.exe`.

## Windows-Specific Considerations

### No Admin Rights Required

All build components are portable:
- MinGW (included in project)
- CMake (portable version in Documents/CMake/)
- FreeGLUT (compiled from source)
- Bullet Physics (compiled from source)

### Library Format

- **Input**: All source files are cross-platform C/C++
- **Output**: Windows PE (Portable Executable) format
- **Linker**: MinGW ld (GNU linker for Windows)

### Environment

The build scripts automatically set the PATH and compiler locations. No system-wide configuration needed.

## Related Documentation

- [Bullet Physics Documentation](https://github.com/bulletphysics/bullet3)
- [FreeGLUT Documentation](http://freeglut.sourceforge.net/)
- [MinGW-w64 Documentation](https://www.mingw-w64.org/)

## License

See LICENSE files in respective directories:
- Bullet Physics: `bullet-2.82-r2704/BulletLicense.txt`
- FreeGLUT: `build/freeglut-local/freeglut-master/COPYING`
- EvoDevo: Check project root LICENSE

---

**Last Updated**: April 26, 2026  
**Build Status**: ✓ Successful (7.19 MB executable)
