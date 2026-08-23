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
import time
from collections import deque


# ---- All identified part constants ----
UNITS_TO_RADS = math.pi
SENSOR_RADIUS = 0.1
DENSITY = 1.9098593171   
FRICTION = 0.8
ROLLING_FRICTION = 0.5
MOTOR_MAX_IMPULSE = 0.4
DT = 1.0 / 60.0
MOTOR_MAX_FORCE = 500.0 #50.0  


# Still trying to figure out
# 1. what about matrix blueprints? (see UseMatrixBlueprints() in NoiseWorld.h)
# 3. should the ground plane be at y=-2 like it was in the C++ simulation? I have set it to 0 for now... is this problematic?
# 4. do i need to add in delete functions if turning off the simulation effectively ends/deletes everything
# 5. is add_sensor() necessary
# 6. Is the hinge used in the C++ code a fixed joint with constraints, but there is no good pybullet equivalent? While I can replicate a similar joint without angle limits, angle limits seem hard to incorporate without writing a custom constraint, which is a bit beyond my current pybullet knowledge 
# 7. Is the Random(42) in run_generations() in runit.py consistent across each generation? Is that bad?

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
        contacts = (p.getContactPoints(physicsClientId = physics_client))
        
        # reset all contacts
        for body_id in self.body_touches:
            self.body_touches[body_id] = 0
            self.touches_point[body_id] = np.array([0.0, 0.0, 0.0])
        
        # process current contacts
        for contact in contacts:

            # finding the bodies and links that are contacting + the position on each body that is beng contacted
            body_id_a = contact[1]
            body_id_b = contact[2]
            link_index_a = contact[3]
            link_index_b = contact[4]
            contact_point_a = np.array(contact[5]) 
            contact_point_b = np.array(contact[6])
            
            # map tracking directly to unique (body, link) configurations <--------------------- TODO:
            self.body_touches[(body_id_a, link_index_a)] = 1
            self.touches_point[(body_id_a, link_index_a)] = contact_point_a
            
            self.body_touches[(body_id_b, link_index_b)] = 1
            self.touches_point[(body_id_b, link_index_b)] = contact_point_b




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
        p.setGravity(0, 0, gravity, physicsClientId=self.client)
        self.gravity = gravity

        p.setTimeStep(dt, physicsClientId=self.client)
        self.dt = dt
        self.time_step = 0
        
        # setting the bounds of the 3D world
        self.world_aabb_min = [-10000, -10000, -10000]
        self.world_aabb_max = [10000, 10000, 10000]
        
        # setting up dictionaries to tie part ids with their Pybullet ids # TODO: Has now been fricken made superfluous
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

        self.robot_id = None
        
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
            basePosition = [0, 0, 0],  # trying this for now since the bodies keep spawning below it
            physicsClientId = self.client
        )

        # setting up the collision group
        p.setCollisionFilterGroupMask(ground_body, -1, collisionFilterGroup=1, collisionFilterMask=3, physicsClientId=self.client)

        # sets the ground id and contact tracking for the ground body
        self.ground_id = ground_body
        self.contact_callback.body_touches[ground_body] = 0
        self.contact_callback.touches_point[ground_body] = np.array([0.0, 0.0, 0.0])
    

    # ---------- SPAWNING ALL THE PARTS ---------
    def create_robot(self, body_blueprints, joint_blueprints, sensor_blueprints=[]):
        """
        Assembles a single robot structure out of individual 
        body and joint blueprints using reduced-coordinate multibodies.
        """

        # ----------- The base body ------------

        # end if there is no base
        if not body_blueprints:
            return None
        
        # trying something new here... graph tree
        # making a dict of body id's which have values of each body + a similar dict which uses empty lists as the value
        body_map = {b.id: b for b in body_blueprints}
        adjacency_list = {b.id: [] for b in body_blueprints}

        # if a base body is in the adjacency list, add the attached joint to the dict value list
        for joint in joint_blueprints:
            if joint.base_body in adjacency_list:
                adjacency_list[joint.base_body].append(joint)


        # map out all children to find the root
        child_ids = {joint.other_body for joint in joint_blueprints}
        # find at least one body that is never a child
        base_blueprint = None
        for b in body_blueprints:
            if b.id not in child_ids:
                base_blueprint = b
                break    
        # fallback to the first body if no child is found
        if base_blueprint is None:
            base_blueprint = body_blueprints[0]

        
        # calculate base body mass/volume (sphere = 4/3 * size^3 * pi)
        base_size = base_blueprint.size
        base_volume = (4.0 / 3.0) * base_size * base_size * base_size * math.pi
        base_mass = base_volume * DENSITY
        
        print(f"DEBUG: Selected Base ID: {base_blueprint.id} (Type: {type(base_blueprint.id)})")
        print(f"DEBUG: Adjacency list keys: {list(adjacency_list.keys())}")
        print(f"DEBUG: Children of Base: {adjacency_list[base_blueprint.id]}")
        
        # create a collision shape and visual shape for the base body
        base_col_id = p.createCollisionShape(
            p.GEOM_SPHERE, 
            radius = base_size,
            physicsClientId = self.client
        )
        base_vis_id = p.createVisualShape(
            p.GEOM_SPHERE, 
            radius = base_size,
            physicsClientId = self.client
        )
        
        # set the base body to position 0,0,0
        base_pos = [base_blueprint.x, base_blueprint.y, base_blueprint.z]
        base_orn = [0, 0, 0, 1]

        # shift the entire robot up so the lowest sphere clears the ground (z=0)
        # (only adjusting the base since all other positions are relative to the base)
        min_z = min(b.z - b.size for b in body_blueprints)
        if min_z < 0.01:
            base_pos[2] += -min_z + 0.01


        # ----------- The non-bass bodies ------------

        # preparing body lists
        link_masses = []
        link_collision_shapes = []
        link_visual_shapes = []
        link_positions = []        # (local offset relative to parent frame)
        link_orientations = []     # (local orientation relative to parent frame)
        link_parent_indices = [] 
        link_joint_types = []    # all are just p.JOINT_REVOLUTE, but we need a fricken list man
        link_joint_axes = []     # Hinge axis vector
        link_inertial_positions = []      # inertial tracking and orientations
        link_inertial_orientations = []

        # defining the self variables
        self.bodies = {base_blueprint.id: -1} 
        self.joint_indices_map = {}
        self.joint_parts = []
        self.body_parts = body_blueprints
        self.sensors = sensor_blueprints 

        # using a queue to iterate through the link tree
        queue = deque([base_blueprint.id])

        while queue:
            # removes the first item from the queue (the parent)
            parent_id = queue.popleft()
            
            # find the adjancy list of the parent to find the nearby bodies
            for joint in adjacency_list[parent_id]:
                child_id = joint.other_body
                if child_id in self.bodies:
                    continue 
                
                # find the blueprint of the unencountered child body
                child_blueprint = body_map[child_id]
                parent_blueprint = body_map[parent_id]
                
                # Set the body id to the last value in the self.bodies / self.joints lists
                self.bodies[child_id] = len(link_masses)
                self.joint_indices_map[joint.id] = len(link_masses)
                joint.blueprint_idx = joint_blueprints.index(joint) 
                self.joint_parts.append(joint)
                


                # add physics values
                child_volume = (4.0 / 3.0) * (child_blueprint.size ** 3) * math.pi
                link_masses.append(child_volume * DENSITY)
                # adding collision shapes and graphics shapes
                col_id = p.createCollisionShape(
                    p.GEOM_SPHERE, 
                    radius=child_blueprint.size, 
                    physicsClientId=self.client
                )
                vis_id = p.createVisualShape(
                    p.GEOM_SPHERE, 
                    radius=child_blueprint.size, 
                    physicsClientId=self.client
                )
                # adding the physics values to the body lists
                link_collision_shapes.append(col_id)
                link_visual_shapes.append(vis_id)


                # finding the pivot locations
                parent_com = np.array([parent_blueprint.x, parent_blueprint.y, parent_blueprint.z])
                child_com = np.array([child_blueprint.x, child_blueprint.y, child_blueprint.z])

                # Use blueprint joint pivot (world coords) and convert to local offsets
                pivot_world = np.array([joint.px, joint.py, joint.pz])
                parent_local_pivot = pivot_world - parent_com
                child_local_pivot = pivot_world - child_com

                # defining the link orientation and center of mass
                # Use the parent-local pivot as the link position so the joint aligns at the blueprint pivot
                link_positions.append(parent_local_pivot.tolist())
                link_orientations.append([0, 0, 0, 1])
                link_inertial_positions.append([0.0, 0.0, 0.0])        
                link_inertial_orientations.append([0.0, 0.0, 0.0, 1.0])
                
                # shifting body indexes since we start at index -1 for the basee
                parent_link_idx = self.bodies[parent_id]
                if parent_link_idx == -1:
                    link_parent_indices.append(0)  
                else:
                    link_parent_indices.append(parent_link_idx + 1)

                # setting up everything as a Revolute joint
                link_joint_types.append(p.JOINT_REVOLUTE)
                # Axis specified in blueprint is in world coords; since we initialize with identity orientations, use it directly
                link_joint_axes.append([joint.ax, joint.ay, joint.az])

                # store the child-local pivot for potential constraint creation or diagnostics
                if not hasattr(self, 'joint_local_frames'):
                    self.joint_local_frames = {}
                self.joint_local_frames[joint.id] = {
                    'parent_pivot': parent_local_pivot.tolist(),
                    'child_pivot': child_local_pivot.tolist(),
                    'axis': [joint.ax, joint.ay, joint.az]
                }

                queue.append(child_id)


        # create the entire robot multibody
        self.robot_id = p.createMultiBody(
            # base properties
            baseMass = base_mass,
            baseCollisionShapeIndex = base_col_id,
            baseVisualShapeIndex = base_vis_id,
            basePosition = base_pos,
            baseOrientation = base_orn,

            # link properties (with child objects attached)
            linkMasses = link_masses,
            linkCollisionShapeIndices = link_collision_shapes,
            linkVisualShapeIndices = link_visual_shapes,
            linkPositions = link_positions,
            linkOrientations = link_orientations,
            linkInertialFramePositions = link_inertial_positions, 
            linkInertialFrameOrientations = link_inertial_orientations,
            linkParentIndices = link_parent_indices,
            linkJointTypes = link_joint_types,
            linkJointAxis = link_joint_axes,

            # client again
            physicsClientId = self.client
        )

        # setting up joint limits on the created body (iterating over robot's list of joints)
        for joint in self.joint_parts:
            pybullet_idx = self.joint_indices_map[joint.id]
            p.changeDynamics(
                self.robot_id, 
                pybullet_idx,
                jointLowerLimit = float(joint.lower_limit),
                jointUpperLimit = float(joint.upper_limit),
                physicsClientId = self.client
            )

        # disabling default motors to allow full motor control YIPEE (this was to resolve an error where joints 
        # could not be actuated and would just swing freely)
        for i in range(p.getNumJoints(self.robot_id, physicsClientId=self.client)):
            p.setJointMotorControl2(
                self.robot_id, 
                i,
                controlMode = p.VELOCITY_CONTROL,
                targetVelocity = 0, # initially stationary
                force = 0,  # non-free-swinging
                physicsClientId = self.client
            )

        
        # surface properties for every sub-link
        for link_idx in range(-1, len(link_masses)):
            p.changeDynamics(
                self.robot_id, link_idx,
                lateralFriction = FRICTION,
                rollingFriction = ROLLING_FRICTION,
                physicsClientId = self.client
            )
        # for debugging (print mapping from blueprint joint id to pybullet joint index)
        try:
            for joint in self.joint_parts:
                py_idx = self.joint_indices_map.get(joint.id, None)
                print(f"DEBUG: joint blueprint id={joint.id} -> pybullet_index={py_idx}, blueprint_idx={getattr(joint, 'blueprint_idx', None)}, motor={getattr(joint, 'motor', False)}")
        except Exception as e:
            print("DEBUG: failed to print joint mappings:", e)
            
        return self.robot_id
    
    
    def add_sensor(self, sensor_part): #<----------------------------------------------------------DO I NEED
        """
        Add a touch sensor to a body
        """
        self.sensors.append(sensor_part)
    

    def PointWorldToLocal(self, bodyIndex, point): ###
        """
        Convert a point from world coordinates to local coordinates for a given body
        """
        if bodyIndex in self.bodies:
            # find the body index
            link_idx = self.bodies[bodyIndex]

            # if the body is the base body
            if link_idx == -1:
                position, orientation = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId=self.client
                )
            # if the body is a child body
            else:
                state = p.getLinkState(
                    self.robot_id, 
                    link_idx, 
                    physicsClientId=self.client
                )
                position, orientation = state[4], state[5] # worldLinkFramePosition, worldLinkFrameOrientation
            
            # invert the transform (gets inverse position and orientation)
            inv_pos, inv_orn = p.invertTransform(position, orientation)

            # apply inverse transform to the point (includes both rotation and translation)
            point_local = np.array(p.multiplyTransforms(inv_pos, inv_orn, point, [0, 0, 0, 1])[0])
            return point_local
        
        return np.array([0.0, 0.0, 0.0]) # surely returning zeros like this in null cases will have no unintended consequences


    

    def AxisWorldToLocal(self, bodyIndex, axis): ###
        """
        Convert an axis from world coordinates to local coordinates for a given body
        """
        if bodyIndex in self.bodies:
            # find the link index
            link_idx = self.bodies[bodyIndex]

            # for the base body
            if link_idx == -1:
                _, orientation = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId = self.client
                )
            
            # for all child bodies
            else:
                state = p.getLinkState(
                    self.robot_id, 
                    link_idx, 
                    physicsClientId = self.client
                )
                orientation = state[5]

            # invert the vector to get the inverse rotation
            inv_orientation = p.invertTransform([0, 0, 0], orientation)[1]

            # apply inverse rotation to convert from world to local coordinates
            axis_local = p.rotateVector(inv_orientation, axis)
            return axis_local
        return axis


    
    def actuate_joint(self, joint_id, desired_angle, dt): ###
        """
        Apply motor command to joint
        """
        # get the current joint angle
        joint_state = p.getJointState(self.robot_id, joint_id, physicsClientId=self.client)
        current_angle = joint_state[0]
        
        # compute the target velocity (same as Bullet's setMotorTarget internals)
        angle_error = desired_angle - current_angle
        target_velocity = angle_error / dt
        
        # apply velocity control with max impulse converted to force
        # (force = impulse / dt)
        max_force = MOTOR_MAX_FORCE # MOTOR_MAX_IMPULSE / dt 

        # moving the correct joint on the robot to the desired angle
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=max_force,
            physicsClientId=self.client
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
        
        # for all sensors...
        for sensor_idx, sensor in enumerate(self.sensors):
            sensor_body_id = self.bodies[sensor.body_id]

            # body/sensor key
            tracking_key = (self.robot_id, sensor_body_id)

            # check if body is touching
            if (tracking_key in self.contact_callback.body_touches) \
            and (self.contact_callback.body_touches[tracking_key] == 1):
                
                # if it is and the part is a sensor, use it for the following functions
                body_part = next((part for part in self.body_parts if part.id == sensor.body_id), None)
                
                if body_part:                    
                    # global contact points
                    contact_point_world = self.contact_callback.touches_point[tracking_key]
    
                    # Get contact point (local coordinates)
                    contact_point = self.PointWorldToLocal(body_part.id, contact_point_world)
                    
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
    



    def step(self, headless=True):
        """Advance simulation by one timestep and process control"""
        # Check collisions
        self.contact_callback.check_collisions(self.client)

        # move camera if in GUI mode
        if (not headless) and (len(self.bodies) > 0):
            
            # Get first body position (same as what I did for save position)
            pos, _ = p.getBasePositionAndOrientation(
                self.robot_id,
                physicsClientId=self.client
            )
            
            p.resetDebugVisualizerCamera(
                cameraDistance = 30,      
                cameraYaw = 50,            
                cameraPitch = -35,         
                cameraTargetPosition = pos 
            )
        
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
            if (joint.id in self.joint_indices_map) and getattr(joint, 'motor', False):
                # 1. Fetch the correct PyBullet link/joint index
                pybullet_joint_idx = self.joint_indices_map[joint.id]

                # 2. Find neural net index from blueprint ordering
                nn_index = getattr(joint, 'blueprint_idx', None)
                motor_command = 0.0
                if nn_index is not None and nn_index < len(self.output_s2j) and nn_index < len(self.output_n2j):
                    motor_command = self.output_s2j[nn_index] + self.output_n2j[nn_index]
                    motor_command = self.tanh(motor_command) * UNITS_TO_RADS
                else:
                    # fallback small oscillation if ANN gives no command
                    motor_command = math.sin(self.time_step * 0.05) * (45 * (math.pi / 180))

                # 3. Clamp to joint blueprint limits if present
                try:
                    low = float(joint.lower_limit)
                    high = float(joint.upper_limit)
                    motor_command = max(min(motor_command, high), low)
                except Exception:
                    pass

                # Debug log
                print(f"ACTUATE: joint.id={joint.id}, py_idx={pybullet_joint_idx}, nn_index={nn_index}, target={motor_command:.3f}")

                # 4. Apply to pybullet joint
                self.actuate_joint(pybullet_joint_idx, motor_command, self.dt)
                '''
                # find the neural net index
                nn_index = joint.blueprint_idx

                if nn_index < len(self.output_s2j) and nn_index < len(self.output_n2j):
                    # Combine outputs
                    motor_command = self.output_s2j[nn_index] + self.output_n2j[nn_index]
                    # Apply tanh activation
                    motor_command = self.tanh(motor_command)
                    # Convert to radians
                    motor_command = motor_command * UNITS_TO_RADS
                else:
                    motor_command = 0.0
                
                # map joint actuation to the link index
                pybullet_joint_index = self.joint_indices_map[joint.id]
                self.actuate_joint(self.robot_id, pybullet_joint_index, motor_command, self.dt)
                '''
        
        self.time_step += 1
    


    def get_body_position(self, body_id):
        """Get position of a body inside the robot multibody"""
        if body_id in self.bodies:
            # find the body index
            index = self.bodies[body_id]

            # if the body is the base body
            if index == -1:
                position, _ = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId=self.client
                )

            # if the body is a child body
            else:
                link_state = p.getLinkState(
                    self.robot_id,
                    index,
                    physicsClientId=self.client
                )
                position = link_state[0]

            # return the position
            return position
        return None
    


    def save_position(self, output_file, completed): ###
        """
        Save final position of first body 
        Only calculates distance if completed flag is True
        """
        distance = 0.0
        if self.robot_id is not None and completed:
            # Get first body position
            pos, _ = p.getBasePositionAndOrientation(
                self.robot_id,
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