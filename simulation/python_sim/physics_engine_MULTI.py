"""physics_engine_(MULTI)
Used to run a pybullet implementation of the physics engine (instead of Bullet sim, 
which was previously used in previous iterations).

This file is currently a work in progress, and is not fully implemented yet,
but the hope is that this engine will allow for a stronger and more easily adoptable/adjustable
implementation of the physics simulation used in the previous ECE mode.

Author: James Hatch
"""

from pathlib import Path

import pybullet as p
import pybullet_data
import numpy as np
import math

# ---- All identified part constants ----
UNITS_TO_RADS = math.pi
SENSOR_RADIUS = 0.1
DENSITY = 1.9098593171   
FRICTION = 0.8
ROLLING_FRICTION = 0.5
MOTOR_MAX_IMPULSE = 0.4
DT = 1.0 / 60.0


# Still trying to figure out
# 1. what about matrix blueprints? (see UseMatrixBlueprints() in NoiseWorld.h)
# 2. why choose 1.9098593171 for density? (see NoiseWorld.cpp line 28)
# 3. should the ground plane be at y=-2 like it was in the C++ simulation? I have set it to 0 for now... is this problematic?
# 4. do i need to add in delete functions if turning off the simulation effectively ends/deletes everything
# 5. is add_sensor() necessary
# 6. Is the hinge used in the C++ code a fixed joint with constraints, but there is no good pybullet equivalent? While I can replicate a similar joint without angle limits, angle limits seem hard to incorporate without writing a custom constraint, which is a bit beyond my current pybullet knowledge 

# 6. CURRENT ISSUE: blueprints are not being read. I think /io and /data need to exist within the 
# project directory, and the blueprint files need to be in there with the correct naming convention (e.g. b_bf_0.dat for body blueprints for sim 0). I have added some print statements to main.py to check if the blueprints are being read correctly, and they are not showing up. I will need to investigate this further and make sure the file paths are correct and the files are being read properly.


# ---- Imports for all part types (all found in legacy code) ----
# Current implementations include BodyPart, JointPart, and SensorPart
# originally from - NoiseWorld.h
# Note: C++ version had methods for each part called readBlueprint() and printSelf()
#       these have been replaced with the new load_blueprints_from_file() method in main.py.
class BodyPart: ###
    """
    Represents a rigid body (sphere) in the simulation
    Properties include the sphere's ID, position (x,y,z), and size (radius)
    """
    def __init__(self, id, x, y, z, size):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.size = size

class JointPart: ###
    """
    Represents a hinge constraint between two bodies
    Properties include the joint's ID, connecting bodies, 
    and constraint parameters (location, flexion limits, associated motor)
    """
    def __init__(self, id, base_body, other_body, px, py, pz, 
                 ax, ay, az, lower_limit, upper_limit, motor):
        self.id = id
        self.base_body = base_body
        self.other_body = other_body
        self.px = px
        self.py = py
        self.pz = pz
        self.ax = ax
        self.ay = ay
        self.az = az
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.motor = motor

class SensorPart: ###
    """
    Represents a touch sensor attached to a body
    Properties include the sensor's ID, associated body, and relative position (x,y,z)
    """
    def __init__(self, id, body_id, x, y, z):
        self.id = id
        self.body_id = body_id
        self.x = x
        self.y = y
        self.z = z



# ---- A class that defines part contact ----
# originally from - NoiseWorld.cpp
class ContactCallback:
    """Manages collision detection and touch tracking"""
    def __init__(self):
        self.body_touches = {}
        self.touches_point = {}

    def check_collisions(self, physics_client):
        """Check all collisions and update body_touches"""
        num_contacts = len(p.getContactPoints(physicsClientId = physics_client))
        
        # Reset all touches
        for body_id in self.body_touches:
            self.body_touches[body_id] = 0
            self.touches_point[body_id] = np.array([0.0, 0.0, 0.0])
        
        # Process all contacts
        for i in range(num_contacts):
            contact = p.getContactPoints(physicsClientId=physics_client)[i]
            body_id_a = contact[1]
            body_id_b = contact[2]
            contact_point_a = np.array(contact[5])  # positionOnA
            contact_point_b = np.array(contact[6])  # positionOnB
            
            if body_id_a in self.body_touches:
                self.body_touches[body_id_a] = 1
                self.touches_point[body_id_a] = contact_point_a
            if body_id_b in self.body_touches:
                self.body_touches[body_id_b] = 1
                self.touches_point[body_id_b] = contact_point_b




class PyBulletWorld:
    """
    A class that defines the world using the previously defined bodies and collision properties
    """
    
    def __init__(self, gravity=-9.81, dt=DT, headless=True):
        """
        Initialize PyBullet world using gravity, timestep rate, and mode (GUI vs. headless)
        """
        # Connect to PyBullet (GUI or DIRECT mode... we need headless for most cases)
        # Most information taken from: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0
        if headless:
            self.client = p.connect(p.DIRECT)
        else:
            self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # world parameters (gravity, timesteps, etc.)
        p.setGravity(0, gravity, 0, physicsClientId=self.client)
        self.dt = dt
        self.gravity = gravity
        self.time_step = 0
        
        # setting the bounds of the 3D world
        self.world_aabb_min = [-10000, -10000, -10000]
        self.world_aabb_max = [10000, 10000, 10000]
        
        # setting up dictionaries to tie part ids with their Pybullet ids
        self.bodies = {} 
        self.joints = {}  
        # setting up the lists of all physical robot structures
        self.body_parts = []  
        self.joint_parts = [] 
        self.sensors = []  
        
        # setting upcollision detection using the previously defined ContactCallback class
        self.contact_callback = ContactCallback()
        
        # setting up ANN matrices
        self.weights_s2n = []  # sensor to neuron
        self.weights_n2n = []  # neuron to neuron
        self.weights_s2j = []  # sensor to joint
        self.weights_n2j = []  # neuron to joint
        
        # setting up ANN outputs
        self.sensor_touches = []
        self.output_s2n = []
        self.output_n2n = []
        self.output_s2j = []
        self.output_n2j = []
        
        # creating a ground plane (defined in the following function)
        self.create_ground()
    


    # ---- Creating world objects ----
    def create_ground(self): ###
        """
        Create static ground plane at y = 0
        """
        # sets up the plane shape
        ground_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client)
        
        # creates the ground body (static and massless)
        ground_body = p.createMultiBody(
            baseMass = 0,
            baseCollisionShapeIndex = ground_shape,
            basePosition = [0, 0, 0], 
            physicsClientId = self.client
        )

        # sets the ground id and contact tracking for the ground body
        self.ground_id = ground_body
        self.contact_callback.body_touches[ground_body] = 0
        self.contact_callback.touches_point[ground_body] = np.array([0.0, 0.0, 0.0])
    

    def create_body(self, body_part): ###
        """
        Create a rigid body sphere in the simulation
        """
        # finding the size from the body part's variable
        size = body_part.size
        
        # calculate volume from:  (4/3) * size^3 * pi
        volume = (4.0 / 3.0) * size * size * size * math.pi
        mass = volume * DENSITY
        
        # create sphere collision shape
        shape = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=size,
            physicsClientId=self.client
        )
        
        # create rigid body
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=shape,
            basePosition=[body_part.x, body_part.y, body_part.z],
            physicsClientId=self.client
        )
        
        # set physics properties (fricion, damping)
        p.changeDynamics(
            body_id,
            -1,  # referring to the original body itself (not a link)
            lateralFriction=FRICTION,
            rollingFriction=ROLLING_FRICTION,
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client
        )
        
        # keeping the component active and trackable for collisions
        p.setCollisionFilterGroupMask(body_id, -1, 1, 1, physicsClientId=self.client)
        p.changeDynamics(body_id, -1, 
                        physicsClientId=self.client)
        
        # Store mapping
        self.bodies[body_part.id] = body_id
        self.body_parts.append(body_part)
        
        # Initialize touch tracking
        self.contact_callback.body_touches[body_id] = 0
        self.contact_callback.touches_point[body_id] = np.array([0.0, 0.0, 0.0])
        #-------

        return body_id
    

    def PointWorldToLocal(self, bodyIndex, point): ###
        """
        Convert a point from world coordinates to local coordinates for a given body
        """
        # get the body's position and orientation
        pos, orientation = p.getBasePositionAndOrientation(
            self.bodies[bodyIndex],
            physicsClientId=self.client
        )
        
        # invert the transform (gets inverse position and orientation)
        inv_pos, inv_orn = p.invertTransform(pos, orientation)
        
        # apply inverse transform to the point (includes both rotation and translation)
        point_local = np.array(p.multiplyTransforms(inv_pos, inv_orn, point, [0, 0, 0, 1])[0])
        
        return point_local
    

    def AxisWorldToLocal(self, bodyIndex, axis): ###
        """
        Convert an axis from world coordinates to local coordinates for a given body
        """
        # get the body's position and orientation
        _, orientation = p.getBasePositionAndOrientation(
            self.bodies[bodyIndex],
            physicsClientId=self.client
        )
        
        # invert the vector to get the inverse rotation
        inv_orientation = p.invertTransform([0, 0, 0], orientation)[1]
        
        # apply inverse rotation to convert from world to local coordinates
        axis_local = p.rotateVector(inv_orientation, axis)
        
        return axis_local


    def _debug_constraint(self, body_a_id, body_b_id):
        num_bodies = p.getNumBodies(physicsClientId=self.client)
        valid_ids = [p.getBodyUniqueId(i, physicsClientId=self.client) 
                    for i in range(num_bodies)]
        
        print(f"Valid body IDs in world: {valid_ids}")
        print(f"body_a_id={body_a_id} valid={body_a_id in valid_ids}")
        print(f"body_b_id={body_b_id} valid={body_b_id in valid_ids}")
        print(f"Same body? {body_a_id == body_b_id}")
        
        for bid in [body_a_id, body_b_id]:
            num_links = p.getNumJoints(bid, physicsClientId=self.client)
            print(f"  Body {bid} has {num_links} joints/links")
    
    def create_joint(self, joint_part):
        """
        Create a hinge constraint between two bodies
        Converts world coordinates to local coordinates to match C++ behavior
        """
        body_a_id = self.bodies[joint_part.base_body]
        body_b_id = self.bodies[joint_part.other_body]
        
        # World coordinates from joint blueprint
        position = np.array([joint_part.px, joint_part.py, joint_part.pz])
        
        # Convert to local coordinates for each body (matching C++ CreateHinge)
        loc_point_1 = self.PointWorldToLocal(joint_part.base_body, position)
        loc_point_2 = self.PointWorldToLocal(joint_part.other_body, position)
        #loc_axis_2 = self.AxisWorldToLocal(joint_part.other_body, axis)

        # Create hinge constraint with local coordinates
        constraint_id = p.createConstraint(
            body_a_id,
            -1,  # base link
            body_b_id,
            -1,  # base link
            p.JOINT_POINT2POINT, # <------------------------------------------------------
            jointAxis = [0, 0, 0],
            parentFramePosition = loc_point_1.tolist(),
            childFramePosition = loc_point_2.tolist(),
            physicsClientId = self.client
        )

        # Set joint limits (lower and upper)
        p.changeConstraint(
            constraint_id,
            maxForce = MOTOR_MAX_IMPULSE if joint_part.motor else 0,
            physicsClientId = self.client
        )
        
        # Store joint
        self.joints[joint_part.id] = {
            'constraint_id': constraint_id,
            'body_a': body_a_id,
            'body_b': body_b_id,
            'motor_enabled': joint_part.motor,
            'lower': joint_part.lower_limit,
            'upper': joint_part.upper_limit,
            'axis': np.array([joint_part.ax, joint_part.ay, joint_part.az])
        }
        self.joint_parts.append(joint_part)


    
    def add_sensor(self, sensor_part): #<----------------------------------------------------------DO I NEED
        """
        Add a touch sensor to a body
        """
        self.sensors.append(sensor_part)


    
    def actuate_joint(self, joint_id, desired_angle, dt): ###
        """
        Apply motor command to joint
        """
        if joint_id in self.joints and self.joints[joint_id]['motor_enabled']:
            constraint = self.joints[joint_id]['constraint_id']
            # Set target angle for motor control with timestep
            p.changeConstraint(
                constraint,
                targetPosition = desired_angle,
                maxForce = MOTOR_MAX_IMPULSE,
                physicsClientId = self.client
            )
    


    def calculate_layer(self, weights, data_in): ###
        """
        Neural network layer calculation 
        output[j] = sum_i(data_in[i] * weights[i][j])
        """
        if not weights or not data_in:
            return []
        
        output = []
        num_outputs = len(weights[0])
        num_inputs = len(data_in)
        
        for j in range(num_outputs):
            d_hold = 0.0
            for i in range(num_inputs):
                if i < len(weights) and j < len(weights[i]):
                    d_hold += float(data_in[i]) * float(weights[i][j])
            output.append(d_hold)
        
        return output
    


    def tanh(self, x):
        """
        Used in steps to adjust the neural network
        Tanh activation: 2/(1+exp(-x)) - 1
        """
        try:
            return 2.0 / (1.0 + math.exp(-x)) - 1.0
        except OverflowError:
            return 1.0 if x > 0 else -1.0
    


    def detect_sensor_touches(self):
        """
        Detect which sensors are in contact (matching C++ clientMoveAndDisplay logic)
        Uses axis-aligned box check matching C++ implementation
        """
        self.sensor_touches = [0] * len(self.sensors)
        
        for sensor_idx, sensor in enumerate(self.sensors):
            sensor_body_id = self.bodies[sensor.body_id]
            
            # Check if body is touching
            if sensor_body_id in self.contact_callback.body_touches and \
               self.contact_callback.body_touches[sensor_body_id] == 1:
                
                # Find the body part
                body_part = None
                for bp in self.body_parts:
                    if self.bodies[bp.id] == sensor_body_id:
                        body_part = bp
                        break
                
                if body_part:
                    # Get contact point (world coordinates)
                    contact_point = self.contact_callback.touches_point[sensor_body_id]
                    
                    # C++ logic: check if contact point is within axis-aligned box
                    # sensor position = (sensor.x * body_size, sensor.y * body_size, sensor.z * body_size)
                    # check if contact is within [sensor_pos +/- sensor_radius] for each axis
                    body_size = body_part.size
                    sensor_x = sensor.x * body_size
                    sensor_y = sensor.y * body_size
                    sensor_z = sensor.z * body_size
                    
                    # Axis-aligned box check (matching C++)
                    if (((contact_point[0] <= (sensor_x + SENSOR_RADIUS)) and
                         (contact_point[1] <= (sensor_y + SENSOR_RADIUS)) and
                         (contact_point[2] <= (sensor_z + SENSOR_RADIUS))) and
                        ((contact_point[0] >= (sensor_x - SENSOR_RADIUS)) and
                         (contact_point[1] >= (sensor_y - SENSOR_RADIUS)) and
                         (contact_point[2] >= (sensor_z - SENSOR_RADIUS)))):
                        self.sensor_touches[sensor_idx] = 1
    



    def step(self):
        """Advance simulation by one timestep and process control"""
        # Check collisions
        self.contact_callback.check_collisions(self.client)
        
        # Step physics
        p.stepSimulation(physicsClientId=self.client)
        
        # Detect sensor touches
        self.detect_sensor_touches()
        
        # Calculate ANN layers
        self.output_s2n = self.calculate_layer(self.weights_s2n, self.sensor_touches)
        self.output_n2n = self.calculate_layer(self.weights_n2n, self.output_s2n)
        self.output_s2j = self.calculate_layer(self.weights_s2j, self.sensor_touches)
        self.output_n2j = self.calculate_layer(self.weights_n2j, self.output_n2n)
        
        # Actuate joints
        for joint_idx, joint in enumerate(self.joint_parts):
            if joint.motor and joint_idx < len(self.output_s2j) and \
               joint_idx < len(self.output_n2j):
                # Combine outputs
                motor_command = self.output_s2j[joint_idx] + self.output_n2j[joint_idx]
                
                # Apply tanh activation
                motor_command = self.tanh(motor_command)
                
                # Convert to radians
                motor_command = motor_command * UNITS_TO_RADS
                
                # Actuate joint with timestep (matching C++ ActuateJoint(index, angle, dt))
                self.actuate_joint(joint.id, motor_command, self.dt)
        
        self.time_step += 1
    


    def get_body_position(self, body_id):
        """Get position of a body"""
        if body_id in self.bodies:
            pos, _ = p.getBasePositionAndOrientation(
                self.bodies[body_id],
                physicsClientId=self.client
            )
            return pos
        return None
    


    def save_position(self, output_file, completed): ###
        """
        Save final position of first body 
        Only calculates distance if completed flag is True
        """
        distance = 0.0
        if len(self.bodies) > 0 and completed:
            # Get first body position
            first_body_id = self.bodies[0]
            pos, _ = p.getBasePositionAndOrientation(
                first_body_id,
                physicsClientId=self.client
            )
            
            # Calculate distance from origin (A^2 + B^2 = C^2)
            distance = math.sqrt(pos[0]**2 + pos[2]**2)
        
        output_path = Path(output_file)

        # create the file if it doesn't exist, otherwise overwrite
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to file
        with open(output_path, 'w') as f:
            f.write(f"{distance}\n")
        
        return distance
    



    def disconnect(self):
        """
        Clean up PyBullet connection
        """
        p.disconnect(physicsClientId=self.client)