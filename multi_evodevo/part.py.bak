"""part --- a module that defines the base characteristics of the parts 
of a robot.

Author: Joshua Hawthorne-Madell
"""

import itertools
import math
import numpy as np

from my_table import table


class RegulatorPool(object):
    def __init__(self, pull_rate, push_rate):
        self.pool = [0.] * 40
        self._pull_rate = pull_rate
        self._push_rate = push_rate

    def get_pull_rate(self):
        return self._pull_rate

    def get_push_rate(self):
        return self._push_rate


class Part(object):
    """Part class that defines general attributes and functionality.

    This class has the basic information for all parts, such as the number and
    kinds of codons and regulatory elements  (REs) the part contains. This
    superclass should never be used directly, only via one of the five
    subclasses: BodyPart, JointPart, NeuronPart, SensorPart, or WirePart.

    Attributes:
        gene_sequence:  The gene-sequence that defines the rest of the part's
                        attributes.
        capacity:  The number of REs a part can hold. This attribute is used to
                   determine the when development ends, along with
                   regulatory_elements and regulators_per_update.  This is
                   determined by the number and magnitude of regulatory element
                   capacity (REC) codons (see rc*).
        regulatory_elements:  The number of regulatory_elements the part
                              currently contains.
        regulators_per_update:  The number of regulatory_elements the part
                                produces every update cycle of development.
                                This should be equal to the sum of all
                                values for the codon_* attributes.
        codon_*:  The count of how many of codon * this part's gene_sequence
                  has, where * is a regulatory element codon. This number is
                  set during the initialization phase, and is used during the
                  development stage.
        rc*:  The count of how many codon * this part's gene_sequence has,
              where * is the magntitude of a regulatory element capacity codon.
              These are used to calculate the capacity attribute of the part.
        other_codons: Any codon that isn't a RE or a REC get's tallied here.
                      These codons have no developmental implications, but may
                      have influenced the type of part the was intiated.
                      (I.e. syntax codons and part-type codons.)
        reg_*:  The count of how many REs of type * the part currently has.
                These counts are modified during the developmental period, and
                their relative magnitudes end up determining the
                characteristics of the part.
        num_updates:  Counts how many update cycles a part has gone through.
                      Used for analysis purposes.
    """
    def __init__(self, gene_sequence):
        """
        Initializes a part with attributes, codon number, and capacity.
        """
        # self.is_developing = True
        self.gene_sequence = gene_sequence
        self.capacity = 0
        self.regulatory_elements = 0
        self.regulators_per_update = 0
        # FOR NEXT RELEASE
        # Turn numbered list of codons into a mapped list of codons
        # codon_list_nums = []
        # for i in range(0, len(gene_sequence), 3):
        #     codon_list_nums.append(gene_sequence[i:i+3])
        # self.codon_list = [table[i] for i in codon_list_nums].sort()
        # BodyPart Codons
        self.codon_size = [0, 0]   # +/- codons
        self.codon_s_num = [0, 0]
        self.codon_j_num = [0, 0]
        self.codon_n_num = [0, 0]
        self.codon_s_loc = [0, 0, 0, 0, 0, 0]   # pos x,y,z then neg x,y,z
        self.codon_j_loc = [0, 0, 0, 0, 0, 0]
        # JointPart Codons
        self.codon_active_passive = [0, 0]
        self.codon_free_rigid = [0, 0]
        self.codon_upper_lower = [0, 0, 0, 0]   # upper+/-, lower +/-
        self.codon_j_inputs = [0, 0]
        # NeuronPart Codons
        self.codon_n_inputs = [0, 0]
        self.codon_n_outputs = [0, 0]
        # self.codon_hidden = [0, 0]
        # SensorPart Codon
        self.codon_s_outputs = [0, 0]
        # WirePart Codons
        self.codon_weight = [0, 0]
        self.codon_direct = [0, 0]
        # Capacity codons
        self.rc30 = 0
        self.rc40 = 0
        self.rc50 = 0
        self.rc60 = 0
        self.rc70 = 0
        self.rc80 = 0
        self.rc90 = 0
        self.rc100 = 0
        self.rc110 = 0
        # Other codons
        self.other_codons = 0
        # BodyPart REs
        self.reg_size = [0., 0.]  # +/- values
        self.reg_s_num = [0., 0.]
        self.reg_j_num = [0., 0.]
        self.reg_n_num = [0, 0]
        self.reg_s_loc = [0., 0., 0., 0., 0., 0.]  # pos x,y,z then neg
        self.reg_j_loc = [0., 0., 0., 0., 0., 0.]
        # JointPart REs
        self.reg_active_passive = [0., 0.]
        self.reg_free_rigid = [0., 0.]
        self.reg_upper_lower = [0., 0., 0., 0.]   # upper+/-, then lower...
        self.reg_j_inputs = [0., 0.]
        # NeuronPart REs
        self.reg_n_inputs = [0., 0.]
        self.reg_n_outputs = [0., 0.]
        # SensorPart REs
        self.reg_s_outputs = [0., 0.]
        # WirePart REs
        self.reg_weight = [0., 0.]
        self.reg_direct = [0., 0.]
        # Data vars
        self.num_updates = 0
        # Initialization methods
        self._init_re_codons()
        self._calculate_capacity()
        self._count_regulators()

    def __eq__(self, other):
        """Standard equality definition"""
        if type(self) is type(other):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __ne__(self, other):
        """Standard non-equality definition"""
        return not self.__eq__(other)

    def _calculate_capacity(self):
        """Sets the capacity attribute based on rc* numbers"""
        for i in range(0, len(self.gene_sequence), 3):
            cur_codon = table[self.gene_sequence[i:i+3]]
            if cur_codon == 'RC+30':
                self.rc30 += 1
                self.capacity += 30
            elif cur_codon == 'RC+40':
                self.rc40 += 1
                self.capacity += 40
            elif cur_codon == 'RC+50':
                self.rc50 += 1
                self.capacity += 50
            elif cur_codon == 'RC+60':
                self.rc60 += 1
                self.capacity += 60
            elif cur_codon == 'RC+70':
                self.rc70 += 1
                self.capacity += 70
            elif cur_codon == 'RC+80':
                self.rc80 += 1
                self.capacity += 80
            elif cur_codon == 'RC+90':
                self.rc90 += 1
                self.capacity += 90
            elif cur_codon == 'RC+100':
                self.rc100 += 1
                self.capacity += 100
            elif cur_codon == 'RC+110':
                self.rc110 += 1
                self.capacity += 110

    def _count_regulators(self):
        """Sets the regulators_per_update attribute."""
        for i in range(0, len(self.gene_sequence), 3):
            cur_codon = table[self.gene_sequence[i:i+3]]
            if cur_codon[1] == 'R':
                self.regulators_per_update += 1

    def _init_re_codons(self):
        """Sets the codon_* attributes."""
        for i in range(0, len(self.gene_sequence), 3):
            cur_codon = table[self.gene_sequence[i:i+3]]
            # BodyPart Codons
            if cur_codon == 'BR_SIZE+':
                self.codon_size[0] += 1
            elif cur_codon == 'BR_SIZE-':
                self.codon_size[1] += 1
            elif cur_codon == 'BR_S_M+':
                self.codon_s_num[0] += 1
            elif cur_codon == 'BR_S_M-':
                self.codon_s_num[1] += 1
            elif cur_codon == 'BR_J_M+':
                self.codon_j_num[0] += 1
            elif cur_codon == 'BR_J_M-':
                self.codon_j_num[1] += 1
            elif cur_codon == 'BR_N_M+':
                self.codon_n_num[0] += 1
            elif cur_codon == 'BR_N_M-':
                self.codon_n_num[1] += 1
            elif cur_codon == 'BR_S_X+':
                self.codon_s_loc[0] += 1
            elif cur_codon == 'BR_S_X-':
                self.codon_s_loc[3] += 1
            elif cur_codon == 'BR_S_Y+':
                self.codon_s_loc[1] += 1
            elif cur_codon == 'BR_S_Y-':
                self.codon_s_loc[4] += 1
            elif cur_codon == 'BR_S_Z+':
                self.codon_s_loc[2] += 1
            elif cur_codon == 'BR_S_Z-':
                self.codon_s_loc[5] += 1
            elif cur_codon == 'BR_J_X+':
                self.codon_j_loc[0] += 1
            elif cur_codon == 'BR_J_X-':
                self.codon_j_loc[3] += 1
            elif cur_codon == 'BR_J_Y+':
                self.codon_j_loc[1] += 1
            elif cur_codon == 'BR_J_Y-':
                self.codon_j_loc[4] += 1
            elif cur_codon == 'BR_J_Z+':
                self.codon_j_loc[2] += 1
            elif cur_codon == 'BR_J_Z-':
                self.codon_j_loc[5] += 1
            # JointPart Codons
            elif cur_codon == 'JR_AP+':
                self.codon_active_passive[0] += 1
            elif cur_codon == 'JR_AP-':
                self.codon_active_passive[1] += 1
            elif cur_codon == 'JR_FR+':
                self.codon_free_rigid[0] += 1
            elif cur_codon == 'JR_FR-':
                self.codon_free_rigid[1] += 1
            elif cur_codon == 'JR_U+':
                self.codon_upper_lower[0] += 1
            elif cur_codon == 'JR_U-':
                self.codon_upper_lower[1] += 1
            elif cur_codon == 'JR_L+':
                self.codon_upper_lower[2] += 1
            elif cur_codon == 'JR_L-':
                self.codon_upper_lower[3] += 1
            elif cur_codon == 'JR_I+':
                self.codon_j_inputs[0] += 1
            elif cur_codon == 'JR_I-':
                self.codon_j_inputs[1] += 1
                # NeuronPart Codons
            elif cur_codon == 'NR_I+':
                self.codon_n_inputs[0] += 1
            elif cur_codon == 'NR_I-':
                self.codon_n_inputs[1] += 1
            elif cur_codon == 'NR_O+':
                self.codon_n_outputs[0] += 1
            elif cur_codon == 'NR_O-':
                self.codon_n_outputs[1] += 1
                # SensorPart Codons
            elif cur_codon == 'SR_O+':
                self.codon_s_outputs[0] += 1
            elif cur_codon == 'SR_O-':
                self.codon_s_outputs[1] += 1
                # WirePart Codons
            elif cur_codon == 'WR_W+':
                self.codon_weight[0] += 1
            elif cur_codon == 'WR_W-':
                self.codon_weight[1] += 1
            elif cur_codon == 'WR_D+':
                self.codon_direct[0] += 1
            elif cur_codon == 'WR_D-':
                self.codon_direct[1] += 1
            # Everything else
            elif (cur_codon[1] == 'P' or cur_codon[0] == 'R' or
                  cur_codon[1] == 'T'):
                self.other_codons += 1
            else:
                raise KeyError('Take a look at your Table!')

    def get_push_list(self, reg_pool):
        """Returns list of REs to push to REGULATOR_POOL and modifies RP."""
        push_list = list()
        # BodyPart REs
        push_list += self.reg_size
        push_list += self.reg_s_num
        push_list += self.reg_j_num
        push_list += self.reg_n_num
        push_list += self.reg_s_loc
        push_list += self.reg_j_loc
        # JointPart REs
        push_list += self.reg_active_passive
        push_list += self.reg_free_rigid
        push_list += self.reg_upper_lower
        push_list += self.reg_j_inputs
        # NeuronPart REs
        push_list += self.reg_n_inputs
        push_list += self.reg_n_outputs
        # SensorPart REs
        push_list += self.reg_s_outputs
        # WirePart REs
        push_list += self.reg_weight
        push_list += self.reg_direct
        # Make push list, modify REGULATOR_POOL, and output list
        o_push_list = [int(math.floor(reg_pool.get_push_rate() * i))
                       for i in push_list]
        reg_pool.pool = [i + j for i, j in
                         itertools.izip(reg_pool.pool, o_push_list)]
        return o_push_list

    def get_pull_list(self, reg_pool):
        # Make pull list, modify REGULATOR_POOL, make sure that
        # REGULATOR_POOL doesn't drop below 0, and output list
        pull_list = [i for i in reg_pool.pool]
        o_pull_list = [int(math.floor(reg_pool.get_pull_rate() * i))
                       for i in pull_list]
        reg_pool.pool = [i - j for i, j in
                         itertools.izip(reg_pool.pool, o_pull_list)]
        for c, e in enumerate(reg_pool.pool):
            if e < 0:
                reg_pool.pool[c] = 0
                o_pull_list[c] += e
        return o_pull_list

    def use_phpl_list(self, phpllst):
        """Updates the parts REs based on diffusion rate."""
        # BodyPart REs
        self.reg_size[0] = max(0, self.reg_size[0] + phpllst[0])
        self.reg_size[1] = max(0, self.reg_size[1] + phpllst[1])
        self.reg_s_num[0] = max(0, self.reg_s_num[0] + phpllst[2])
        self.reg_s_num[1] = max(0, self.reg_s_num[1] + phpllst[3])
        self.reg_j_num[0] = max(0, self.reg_j_num[0] + phpllst[4])
        self.reg_j_num[1] = max(0, self.reg_j_num[1] + phpllst[5])
        self.reg_n_num[0] = max(0, self.reg_n_num[0] + phpllst[6])
        self.reg_n_num[1] = max(0, self.reg_n_num[1] + phpllst[7])
        self.reg_s_loc[0] = max(0, self.reg_s_loc[0] + phpllst[8])
        self.reg_s_loc[1] = max(0, self.reg_s_loc[1] + phpllst[9])
        self.reg_s_loc[2] = max(0, self.reg_s_loc[2] + phpllst[10])
        self.reg_s_loc[3] = max(0, self.reg_s_loc[3] + phpllst[11])
        self.reg_s_loc[4] = max(0, self.reg_s_loc[4] + phpllst[12])
        self.reg_s_loc[5] = max(0, self.reg_s_loc[5] + phpllst[13])
        self.reg_j_loc[0] = max(0, self.reg_j_loc[0] + phpllst[14])
        self.reg_j_loc[1] = max(0, self.reg_j_loc[1] + phpllst[15])
        self.reg_j_loc[2] = max(0, self.reg_j_loc[2] + phpllst[16])
        self.reg_j_loc[3] = max(0, self.reg_j_loc[3] + phpllst[17])
        self.reg_j_loc[4] = max(0, self.reg_j_loc[4] + phpllst[18])
        self.reg_j_loc[5] = max(0, self.reg_j_loc[5] + phpllst[19])
        # JointPart REs
        self.reg_active_passive[0] = max(0, self.reg_active_passive[0] +
                                         phpllst[20])
        self.reg_active_passive[1] = max(0, self.reg_active_passive[1] +
                                         phpllst[21])
        self.reg_free_rigid[0] = max(0, self.reg_free_rigid[0] + phpllst[22])
        self.reg_free_rigid[1] = max(0, self.reg_free_rigid[1] + phpllst[23])
        self.reg_upper_lower[0] = max(0, self.reg_upper_lower[0] + phpllst[24])
        self.reg_upper_lower[1] = max(0, self.reg_upper_lower[1] + phpllst[25])
        self.reg_upper_lower[2] = max(0, self.reg_upper_lower[2] + phpllst[26])
        self.reg_upper_lower[3] = max(0, self.reg_upper_lower[3] + phpllst[27])
        self.reg_j_inputs[0] = max(0, self.reg_j_inputs[0] + phpllst[28])
        self.reg_j_inputs[1] = max(0, self.reg_j_inputs[1] + phpllst[29])
        # NeuronPart REs
        self.reg_n_inputs[0] = max(0, self.reg_n_inputs[0] + phpllst[30])
        self.reg_n_inputs[1] = max(0, self.reg_n_inputs[1] + phpllst[31])
        self.reg_n_outputs[0] = max(0, self.reg_n_outputs[0] + phpllst[32])
        self.reg_n_outputs[1] = max(0, self.reg_n_outputs[1] + phpllst[33])
        # SensorPart REs
        self.reg_s_outputs[0] = max(0, self.reg_s_outputs[0] + phpllst[34])
        self.reg_s_outputs[1] = max(0, self.reg_s_outputs[1] + phpllst[35])
        # WirePart REs
        self.reg_weight[0] = max(0, self.reg_weight[0] + phpllst[36])
        self.reg_weight[1] = max(0, self.reg_weight[1] + phpllst[37])
        self.reg_direct[0] = max(0, self.reg_direct[0] + phpllst[38])
        self.reg_direct[1] = max(0, self.reg_direct[1] + phpllst[39])

    def _diffusion(self, reg_pool):
        """
        Wrapper for diffusion sub_methods.

        Uses get_pull_list(), get_push_list(), and use_phpl_list(),
        with added check that class-instance regulatory_elements
        doesn't drop below 0.
        """
        pllst = self.get_pull_list(reg_pool)
        phlst = self.get_push_list(reg_pool)
        # Negative values means more pushed than pulled
        phpllst = [i - j for i, j in itertools.izip(pllst, phlst)]
        self.use_phpl_list(phpllst)
        self.regulatory_elements = max(0, self.regulatory_elements +
                                       sum(phpllst))

    def _update(self):
        """Adds one RE for each codon of each type the part has

        Also increases num_updates class-instance."""
        self.num_updates += 1
        self.regulatory_elements += self.regulators_per_update
        # BodyPart REs
        self.reg_size[0] += self.codon_size[0]
        self.reg_size[1] += self.codon_size[1]
        self.reg_s_num[0] += self.codon_s_num[0]
        self.reg_s_num[1] += self.codon_s_num[1]
        self.reg_j_num[0] += self.codon_j_num[0]
        self.reg_j_num[1] += self.codon_j_num[1]
        self.reg_n_num[0] += self.codon_n_num[0]
        self.reg_n_num[1] += self.codon_n_num[1]
        self.reg_s_loc[0] += self.codon_s_loc[0]
        self.reg_s_loc[1] += self.codon_s_loc[1]
        self.reg_s_loc[2] += self.codon_s_loc[2]
        self.reg_s_loc[3] += self.codon_s_loc[3]
        self.reg_s_loc[4] += self.codon_s_loc[4]
        self.reg_s_loc[5] += self.codon_s_loc[5]
        self.reg_j_loc[0] += self.codon_j_loc[0]
        self.reg_j_loc[1] += self.codon_j_loc[1]
        self.reg_j_loc[2] += self.codon_j_loc[2]
        self.reg_j_loc[3] += self.codon_j_loc[3]
        self.reg_j_loc[4] += self.codon_j_loc[4]
        self.reg_j_loc[5] += self.codon_j_loc[5]
        # JointPart REs
        self.reg_active_passive[0] += self.codon_active_passive[0]
        self.reg_active_passive[1] += self.codon_active_passive[1]
        self.reg_free_rigid[0] += self.codon_free_rigid[0]
        self.reg_free_rigid[1] += self.codon_free_rigid[1]
        self.reg_upper_lower[0] += self.codon_upper_lower[0]
        self.reg_upper_lower[1] += self.codon_upper_lower[1]
        self.reg_upper_lower[2] += self.codon_upper_lower[2]
        self.reg_upper_lower[3] += self.codon_upper_lower[3]
        self.reg_j_inputs[0] += self.codon_j_inputs[0]
        self.reg_j_inputs[1] += self.codon_j_inputs[1]
        # NeuronPart REs
        self.reg_n_inputs[0] += self.codon_n_inputs[0]
        self.reg_n_inputs[1] += self.codon_n_inputs[1]
        self.reg_n_outputs[0] += self.codon_n_outputs[0]
        self.reg_n_outputs[1] += self.codon_n_outputs[1]
        # SensorPart REs
        self.reg_s_outputs[0] += self.codon_s_outputs[0]
        self.reg_s_outputs[1] += self.codon_s_outputs[1]
        # WirePart REs
        self.reg_weight[0] += self.codon_weight[0]
        self.reg_weight[1] += self.codon_weight[1]
        self.reg_direct[0] += self.codon_direct[0]
        self.reg_direct[1] += self.codon_direct[1]
