"""main_pybullet.py
PyBullet physics simulation runner (replaces C++ Main.cpp and NoiseWorld.cpp)

Author: James Hatch
"""

import sys
import argparse
from pathlib import Path
import numpy as np
import os
import time
import pybullet as p

from physics_engine_MULTI import PyBulletWorld, BodyPart, JointPart, SensorPart



def load_blueprints_from_file(filename, file_type): ###
    """
    Load blueprint data for each part (senors, joints, bodies) from the data files
    and returns them as a list of the appropriate part objects.
    """
    results = []
    
    if not Path(filename).exists():
        print(f"Warning: Blueprint file {filename} not found")
        return results
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            if file_type == "body":
                parts = line.split(',')
                body = BodyPart(
                    id = int(parts[0]),
                    x = float(parts[1]),
                    y = float(parts[2]),
                    z = float(parts[3]),
                    size = float(parts[4])
                )
                results.append(body)
            
            elif file_type == "joint":
                parts = line.split(',')
                joint = JointPart(
                    id = int(parts[0]),
                    base_body = int(parts[1]),
                    other_body = int(parts[2]),
                    px = float(parts[3]),
                    py = float(parts[4]),
                    pz = float(parts[5]),
                    ax = float(parts[6]),
                    ay = float(parts[7]),
                    az = float(parts[8]),
                    lower_limit = float(parts[9]),
                    upper_limit = float(parts[10]),
                    motor = (parts[11].strip() == 'T')
                )
                results.append(joint)
            
            elif file_type == "sensor":
                parts = line.split(',')
                sensor = SensorPart(
                    id = int(parts[0]),
                    body_id = int(parts[1]),
                    x = float(parts[2]),
                    y = float(parts[3]),
                    z = float(parts[4])
                )
                results.append(sensor)
    
    return results



def load_matrix_from_file(filename):
    """Load ANN weight matrix from CSV file"""
    matrix = []
    
    if not Path(filename).exists():
        print(f"Warning: Matrix file {filename} not found")
        return matrix
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            row = [float(x) for x in line.split(',')]
            matrix.append(row)
    
    return matrix



def build_blueprint_filenames(io_file, sim_number):
    """Build file paths matching C++ naming convention"""
    def insert_sim_number(template, sim_num):
        parts = template.split('_bf_')
        return f"{parts[0]}_bf_{sim_num}{parts[1]}"
    
    io_dir = Path(io_file)
    
    return {
        'body': str(io_dir / insert_sim_number(f"b_bf_.dat", sim_number)),
        'joint': str(io_dir / insert_sim_number(f"j_bf_.dat", sim_number)),
        'sensor': str(io_dir / insert_sim_number(f"s_bf_.dat", sim_number)),
        's2n': str(io_dir / insert_sim_number(f"s2n_bf_.dat", sim_number)),
        'n2n': str(io_dir / insert_sim_number(f"n2n_bf_.dat", sim_number)),
        's2j': str(io_dir / insert_sim_number(f"s2j_bf_.dat", sim_number)),
        'n2j': str(io_dir / insert_sim_number(f"n2j_bf_.dat", sim_number)),
    }



def run_simulation(io_file, sim_number, gravity, headless=False, max_steps=1000):
    """Run the PyBullet physics simulation"""
    
    '''
    print("=" * 70)
    print("EvoDevo PyBullet Physics Simulation")
    print("=" * 70)
    print(f"Simulation #: {sim_number}")
    print(f"Config file: {io_file}")
    print(f"Gravity: {gravity}")
    print(f"Headless: {headless}")
    print("=" * 70)
    print()
    '''
    
    # Load blueprints
    files = build_blueprint_filenames(io_file, sim_number)
    
    #print("Loading blueprints...")
    bodies = load_blueprints_from_file(files['body'], 'body')
    joints = load_blueprints_from_file(files['joint'], 'joint')
    sensors = load_blueprints_from_file(files['sensor'], 'sensor')
    
    #print(f"  Bodies: {len(bodies)}")
    #print(f"  Joints: {len(joints)}")
    #print(f"  Sensors: {len(sensors)}")
    
    weights_s2n = load_matrix_from_file(files['s2n'])
    weights_n2n = load_matrix_from_file(files['n2n'])
    weights_s2j = load_matrix_from_file(files['s2j'])
    weights_n2j = load_matrix_from_file(files['n2j'])
    
    #print(f"  ANN weights loaded")
    #print()
    
    # Initialize world (either in graphics or headless mode)
    world = PyBulletWorld(gravity=gravity, headless=headless)

    world.weights_s2n = weights_s2n
    world.weights_n2n = weights_n2n
    world.weights_s2j = weights_s2j
    world.weights_n2j = weights_n2j
    
    # Create bodies and joints
    #print("Creating physics objects...")
    for body in bodies:
        world.create_body(body)
    
    for joint in joints:
        world.create_joint(joint)
    
    for sensor in sensors:
        world.add_sensor(sensor)
    
    #print(f"  {len(bodies)} bodies created")
    #print(f"  {len(joints)} joints created")
    #print(f"  {len(sensors)} sensors added")
    #print()

    if not headless:        
        # Disable GUI panel clutter (optional, makes it look cleaner)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # Safely enable single-step rendering now that everything is loaded
        # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
    
    # Run simulation
    # Run simulation (0 to max_steps inclusive = max_steps + 1 total steps)
    for step in range(max_steps + 1):

        if not headless:
            world.step(headless=False)
            time.sleep(world.dt)
        
        else:
            world.step()
                
    
    #print("Simulation complete!")
    #print()
    
    # Save final position
    output_file = os.path.join(os.path.abspath(io_file), f'sim_{sim_number}.dat')
    distance = world.save_position(output_file, completed=True)
    #print(f"Final distance: {distance:.6f}")
    #print(f"Saved to: {output_file}")

    
    # Cleanup
    world.disconnect()

    print(max_steps)
    return 0



def main():
    parser = argparse.ArgumentParser(
        description="EvoDevo PyBullet Physics Simulation"
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
        "--headless",
        action="store_true",
        help="Run in headless mode (no graphics)"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1000,
        help="Number of simulation steps"
    )
    
    args = parser.parse_args()

    return run_simulation(
        io_file=args.file,
        sim_number=args.number,
        gravity=args.gravity,
        headless=args.headless,
        max_steps=args.steps
    )



if __name__ == "__main__":
    sys.exit(main())