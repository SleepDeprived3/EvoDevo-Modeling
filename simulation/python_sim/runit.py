"""runit.py
Calls the physics engine world every time we want to run a simulation.

Author: James Hatch
"""

import subprocess
import sys
import os
from pathlib import Path


def run_simulation(io_file, sim_number, gravity, headless=True, steps=500):
    """
    Run the PyBullet physics simulation.
    
    Args:
        io_file (str): Base configuration file path
        sim_number (int): Simulation number
        gravity (float): Gravity value
        headless (bool): Run in headless mode (yes or no)
        steps (int): Number of simulation steps
    
    Returns:
        int: Exit code from the simulation
    """
    script_dir = Path(__file__).parent
    main_script = script_dir / "main_pybullet.py"
    
    if not main_script.exists():
        print(f"ERROR: {main_script} not found!")
        return 1
    
    print("=" * 70)
    print("EvoDevo PyBullet Physics Simulation")
    print("=" * 70)
    print(f"Script: {main_script}")
    print("=" * 70)
    print()
    
    # Build command
    cmd = [sys.executable, str(main_script)]
    cmd.extend(["-f", str(io_file)])
    cmd.extend(["-n", str(sim_number)])
    cmd.extend(["-g", str(gravity)])
    cmd.extend(["--steps", str(steps)])
    
    if headless:
        cmd.append("--headless")
    
    # Run the simulation
    try:
        result = subprocess.run(cmd, cwd=str(script_dir))
        return result.returncode
    except FileNotFoundError:
        print(f"ERROR: Could not find Python executable")
        return 1
    except Exception as e:
        print(f"ERROR: Failed to run simulation: {e}")
        return 1




def main():
    """
    Main entry point for the simulation
    Runs the simulation with specified parameters
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Run the EvoDevo PyBullet Physics Simulation"
    )
    parser.add_argument(
        "-f", "--file",
        default="blueprint.dat",
        help="Base configuration file path"
    )
    parser.add_argument(
        "-n", "--number",
        type=int,
        default=0,
        help="Simulation number"
    )
    parser.add_argument(
        "-g", "--gravity",
        type=float,
        default=-9.81,
        help="Gravity value"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=500,
        help="Number of simulation steps"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no graphics)"
    )
    
    args = parser.parse_args()
    
    return run_simulation(
        io_file=args.file,
        sim_number=args.number,
        gravity=args.gravity,
        headless=args.headless,
        steps=args.steps
    )


if __name__ == "__main__":
    sys.exit(main())
