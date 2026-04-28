#!/usr/bin/env python3
"""
runit.py --- C++ Physics Simulation Engine Runner

This script provides a Python interface to run the compiled C++ physics simulation engine.
It supports both graphics (interactive) and headless (data collection) modes.

Usage:
    python runit.py                    # Run with graphics (default)
    python runit.py --headless         # Run in headless mode
    python runit.py --help             # Show help

Author: EvoDevo Development Team
"""

import subprocess
import sys
import os
from pathlib import Path


def get_executable_path():
    """Get the full path to the compiled executable."""
    script_dir = Path(__file__).parent
    exe_path = script_dir / "build" / "app.exe"
    return exe_path


def build_if_needed():
    """Check if app.exe exists and build if needed."""
    exe_path = get_executable_path()
    
    if not exe_path.exists():
        print("=" * 70)
        print("WARNING: app.exe not found. Building C++ application...")
        print("=" * 70)
        
        build_script = exe_path.parent.parent / "BUILD_ALL.bat"
        if not build_script.exists():
            print("ERROR: BUILD_ALL.bat not found!")
            return False
        
        try:
            result = subprocess.run([str(build_script)], shell=True, cwd=exe_path.parent.parent)
            if result.returncode != 0:
                print("ERROR: Build failed!")
                return False
        except Exception as e:
            print(f"ERROR: Build execution failed: {e}")
            return False
    
    return exe_path.exists()


def run_simulation(headless=False):
    """
    Run the physics simulation engine.
    
    Args:
        headless (bool): If True, run in headless mode (no graphics)
                        If False, run with graphics window
    
    Returns:
        int: Exit code from the executable
    """
    exe_path = get_executable_path()
    
    # Build if needed
    if not build_if_needed():
        print("ERROR: Could not locate or build app.exe")
        return 1
    
    print("=" * 70)
    print("EvoDevo Physics Simulation Engine")
    print("=" * 70)
    print(f"Executable: {exe_path}")
    print(f"Mode: {'Headless' if headless else 'Graphics'}")
    print("=" * 70)
    print()
    
    # Build command
    cmd = [str(exe_path)]
    if headless:
        cmd.append("--headless")
    
    # Run the executable
    try:
        result = subprocess.run(cmd, cwd=str(exe_path.parent))
        return result.returncode
    except FileNotFoundError:
        print(f"ERROR: Could not find executable at {exe_path}")
        return 1
    except Exception as e:
        print(f"ERROR: Failed to run simulation: {e}")
        return 1


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Run the EvoDevo C++ Physics Simulation Engine"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no graphics window)"
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Only build the application, don't run it"
    )
    
    args = parser.parse_args()
    
    if args.build_only:
        print("Building C++ application...")
        if build_if_needed():
            print("✓ Build successful!")
            return 0
        else:
            print("✗ Build failed!")
            return 1
    
    return run_simulation(headless=args.headless)


if __name__ == "__main__":
    sys.exit(main())
