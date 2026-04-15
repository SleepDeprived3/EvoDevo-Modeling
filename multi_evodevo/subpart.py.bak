"""subpart --- a module that defines the specfic sub-characteristics of the 
different parts of a robot.

Author: Joshua Hawthorne-Madell
"""
import numpy as np
import math
import itertools

from part import Part
from collections import namedtuple

Body_Characteristics = namedtuple('Body_Characteristics',
                                  ['size', 'joint_mount_num',
                                   'joint_mount_loc',
                                   'neuron_mount_num',
                                   'sensor_mount_num',
                                   'sensor_mount_loc'])
Joint_Characteristics = namedtuple('Joint_Characteristics',
                                   ['motor', 'free',
                                    'upper_limit', 'lower_limit',
                                    'input_num'])
Neuron_Characteristics = namedtuple('Neuron_Characteristics',
                                    ['input_num', 'output_num'])
Sensor_Characteristics = namedtuple('Sensor_Characteristics',
                                    ['output_num'])
Wire_Characteristics = namedtuple('Wire_Characteristics',
                                  ['weight', 'to_joint'])


class BodyPart(Part):
    def __init__(self, gene_sequence):
        Part.__init__(self, gene_sequence)
        self.j_mount_loc = []
        self.j_mount_num = 0
        self.j_loc_holder = [0., 0., 0.]
        self.s_mount_loc = []
        self.s_mount_num = 0
        self.s_loc_holder = [0., 0., 0.]
        self.characteristics = Body_Characteristics(0, 0, 0, 0, 0, 0)

    def calculate_mount_info(self):
        # Store last runs sensor and joint mount numbers
        old_s_num, old_j_num = self.s_mount_num, self.j_mount_num
        # calculate current sensor and joint locs
        self.j_loc_holder = [self.reg_j_loc[0] - self.reg_j_loc[3],
                             self.reg_j_loc[1] - self.reg_j_loc[4],
                             self.reg_j_loc[2] - self.reg_j_loc[5]]
        self.s_loc_holder = [self.reg_s_loc[0] - self.reg_s_loc[3],
                             self.reg_s_loc[1] - self.reg_s_loc[4],
                             self.reg_s_loc[2] - self.reg_s_loc[5]]
        # Update current sensor and joint mount numbers
        # Sensor
        try:
            self.s_mount_num = round(self.reg_s_num[0]/self.reg_s_num[1])
        except ZeroDivisionError:
            self.s_mount_num = round(self.reg_s_num[0]/1)
        if self.s_mount_num < 1:
            self.s_mount_num = 0
        # Joint
        try:
            self.j_mount_num = round(self.reg_j_num[0]/self.reg_j_num[1])
        except ZeroDivisionError:
            self.j_mount_num = round(self.reg_j_num[0]/1)
        if self.j_mount_num < 1:
            self.j_mount_num = 0
        # If there's a change in amount of sensor mounts,
        # append a normalized location vector
        if ((self.s_mount_num - old_s_num) >= 1):
            location = np.array([self.s_loc_holder[0],
                                 self.s_loc_holder[1],
                                 self.s_loc_holder[2]], dtype='f')
            # if any(location):
            #     location = location / np.linalg.norm(location)
            #     self.s_mount_loc.append(location)
            if any(location):
                location = location / np.linalg.norm(location)
                if any((location == i).all() or (location == j).all()
                       for i, j in itertools.izip(self.s_mount_loc,
                                                  self.j_mount_loc)):
                    pass
                else:
                    self.s_mount_loc.append(location)
        # Same for joints
        if ((self.j_mount_num - old_j_num) >= 1):
            location = np.array([self.j_loc_holder[0],
                                 self.j_loc_holder[1],
                                 self.j_loc_holder[2]], dtype='f')
            # if any(location):
            #     location = location / np.linalg.norm(location)
            #     self.j_mount_loc.append(location)
            if any(location):
                location = location / np.linalg.norm(location)
                if any((location == i).all() or (location == j).all()
                       for i, j in itertools.izip(self.s_mount_loc,
                                                  self.j_mount_loc)):
                    pass
                else:
                    self.j_mount_loc.append(location)

    def update(self):
        self._update()
        self.calculate_mount_info()

    def rotate_body(self, orientation):
        """Uniformly rotates all jount and sensor mount locations.

        The rotation will be by the difference in angle between the
        additive inverse of the orientation vector (which is the
        active jount mount vector of the base part) and the direction
        of the first joint mount of this body."""
        inv_orientation = -1 * orientation
        jm_direction = self.j_mount_loc[0]
        # spherical coordinates are defined by three numbers:
        # r --- radius, will always be one in this code
        # theta --- azimuth angle [0-2pi]  =  atan(y/x)
        # phi --- zenith angle [0-pi]      =  acos(-z/r)
        io_theta = math.atan2(inv_orientation[1], inv_orientation[0])
        try:
            io_phi = math.acos(inv_orientation[2])
        except ValueError:
            if inv_orientation[2] > 1:
                io_phi = 0
            else:
                io_phi = math.pi
        jm_theta = math.atan2(jm_direction[1], jm_direction[0])
        jm_phi = math.acos(jm_direction[2])
        diff_theta = (io_theta - jm_theta) % (2 * math.pi)
        diff_phi = (io_phi - jm_phi) % math.pi
        # Calculate rotation matrix
        cos_t, sin_t = math.cos(diff_theta), math.sin(diff_theta)
        cos_p, sin_p = math.cos(diff_phi), math.sin(diff_phi)
        z_matrix = np.array([[cos_t, -1 * sin_t, 0], [sin_t, cos_t, 0],
                             [0, 0, 1]], dtype='f')
        y_matrix = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-1 * sin_p, 0,
                                                            cos_p]], dtype='f')
        rotation_matrix = np.around(z_matrix.dot(y_matrix), 10)
        # Apply rotation matrix
        for i in range(len(self.s_mount_loc)):
            self.s_mount_loc[i] = rotation_matrix.dot(self.s_mount_loc[i])
        for i in range(len(self.j_mount_loc)):
            self.j_mount_loc[i] = rotation_matrix.dot(self.j_mount_loc[i])

    def set_characteristics(self):
        """Returns values for BodyPart characteristics.

        This includes: size measure--(radius); joint mount number;
        joint mount locations; neuron mount number; sensor mount number;
        and sensor mount locations."""
        size = round((1 + .2 * (self.reg_size[0] - self.reg_size[1])), 3)
        if size < .5:
            size = .5
        if size > 10:
            size = 10
        # Resize joint mounts based on number of locations registered
        joint_mounts = len(self.j_mount_loc)
        # Resize joint mounts based on size
        if joint_mounts > 1:
            if size <= .5:
                joint_mounts = 1
                self.j_mount_loc = self.j_mount_loc[0:1]
            elif size <= 1:
                joint_mounts = min(5, joint_mounts)
                self.j_mount_loc = self.j_mount_loc[0:joint_mounts]
            elif size <= 2:
                joint_mounts = min(9, joint_mounts)
                self.j_mount_loc = self.j_mount_loc[0:joint_mounts]
        # calculate neuron mounts
        try:
            neuron_mounts = round(self.reg_n_num[0]/self.reg_n_num[1])
        except ZeroDivisionError:
            neuron_mounts = round(self.reg_n_num[0]/1)
        # Resize sensor mounts based on number of locations registered
        sensor_mounts = len(self.s_mount_loc)
        # Create characteristics
        self.characteristics = Body_Characteristics(size,
                                                    joint_mounts,
                                                    self.j_mount_loc,
                                                    neuron_mounts,
                                                    sensor_mounts,
                                                    self.s_mount_loc)

    def get_characteristics(self):
        if any(self.characteristics):
            return self.characteristics
        else:
            raise ValueError


class JointPart(Part):
    def __init__(self, gene_sequence):
        Part.__init__(self, gene_sequence)
        self.characteristics = Joint_Characteristics(0, 0, 0, 0, 0)

    def update(self):
        self._update()

    def set_characteristics(self):
        """Returns final values for JointPart characteristics.

        Includes whether joint gets a motor; whether it is free or rigid;
        and one value for each limit in radians (upper/lower)"""
        motor = (self.reg_active_passive[0] - self.reg_active_passive[1]) > 0
        free = (self.reg_free_rigid[0] - self.reg_free_rigid[1]) > 0
        try:
            upper_ratio = ((float(self.reg_upper_lower[0]) -
                           self.reg_upper_lower[1]) /
                           (self.reg_upper_lower[0] +
                            self.reg_upper_lower[1]))
            upper_limit = upper_ratio * math.pi
        except ZeroDivisionError:
            upper_limit = math.pi
        try:
            lower_ratio = -((float(self.reg_upper_lower[2]) -
                             self.reg_upper_lower[3]) /
                            (self.reg_upper_lower[2] +
                             self.reg_upper_lower[3]))
            lower_limit = lower_ratio * math.pi
        except ZeroDivisionError:
            lower_limit = -1 * math.pi
        if free:
            pass
        else:
            if (sum(self.reg_upper_lower[:2]) > sum(self.reg_upper_lower[2:])):
                lower_limit = upper_limit
            else:
                upper_limit = lower_limit
        try:
            inputs = int(round(self.reg_j_inputs[0] / self.reg_j_inputs[1]))
        except ZeroDivisionError:
            inputs = int(round(self.reg_j_inputs[0]))
        if inputs < 0:
            inputs = 0
        self.characteristics = Joint_Characteristics(motor, free,
                                                     round(upper_limit, 5),
                                                     round(lower_limit, 5),
                                                     inputs)

        def get_characteristics(self):
            if any(self.characteristics):
                return self.characteristics
            else:
                raise ValueError


class NeuronPart(Part):
    def __init__(self, gene_sequence):
        Part.__init__(self, gene_sequence)
        self.characteristics = Neuron_Characteristics(0, 0)

    def update(self):
        self._update()

    def set_characteristics(self):
        """Returns final values for NeuronPart characteristics.

        Includes number of inputs and number of outputs"""
        try:
            input_slots = int(round(self.reg_n_inputs[0] /
                                    self.reg_n_inputs[1]))
        except ZeroDivisionError:
            input_slots = int(round(self.reg_n_inputs[0]))
        try:
            output_slots = int(round(self.reg_n_outputs[0] /
                                     self.reg_n_outputs[1]))
        except ZeroDivisionError:
            output_slots = int(round(self.reg_n_outputs[0]))

        self.characteristics = Neuron_Characteristics(input_slots,
                                                      output_slots)

    def get_characteristics(self):
        if any(self.characteristics):
            return self.characteristics
        else:
            raise ValueError


class SensorPart(Part):
    def __init__(self, gene_sequence):
        Part.__init__(self, gene_sequence)
        self.characteristics = Sensor_Characteristics(0)

    def update(self):
        self._update()

    def set_characteristics(self):
        """Returns final values for SensorPart characteristics.

        This includes: Number of output slots"""
        try:
            output_slots = int(round(self.reg_s_outputs[0] /
                                     self.reg_s_outputs[1]))
        except ZeroDivisionError:
            output_slots = int(round(self.reg_s_outputs[0]/1))
        self.characteristics = Sensor_Characteristics(output_slots)

    def get_characteristics(self):
        if any(self.characteristics):
            return self.characteristics
        else:
            raise ValueError


class WirePart(Part):
    def __init__(self, gene_sequence):
        Part.__init__(self, gene_sequence)
        self.characteristics = Wire_Characteristics(0, 0)

    def update(self):
        self._update()

    def set_characteristics(self):
        """Returns final values for WirePart characteristics.

        Includes weight (0,1); and connection preference."""
        connection_weight = math.atan(((self.reg_weight[0] -
                                        self.reg_weight[1]) /
                                       2.) / (math.pi / 2))
        direct = ((self.reg_direct[0] - self.reg_direct[1]) >= 0)
        self.characteristics = Wire_Characteristics(connection_weight,
                                                    direct)

    def get_characteristics(self):
        if any(self.characteristics):
            return self.characteristics
        else:
            raise ValueError
