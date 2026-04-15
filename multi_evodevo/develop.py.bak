"""develop --- a module that runs the proto-parts through development.

Author: Joshua Hawthorne-Madell
"""
import itertools
from more_itertools import peekable
from collections import namedtuple

from part import (Part, RegulatorPool)
from subpart import (BodyPart, JointPart, NeuronPart, SensorPart, WirePart)


Body = namedtuple('Body', ['body_index', 'mounts_used', 'mounts_total'])
Joint = namedtuple('Joint', ['joint', 'base_body', 'base_mount',
                             'other_body'])
Sensor = namedtuple('Sensor', ['sensor', 'body_index', 'mount_used'])
Neuron = namedtuple('Neuron', ['neuron', 'body_index', 'mount_used'])
Wire = namedtuple('Wire', ['wire', 'from_type', 'from_index',
                           'to_type', 'to_index'])
Mount = namedtuple('Mount', ['part_index', 'mounts_used', 'mounts_total'])


def update_cycles(parts_developing):
    """
    Returns a list of fully developed parts when given a list of proto-parts.

    Updates all proto-parts, in series (step-by-step). When proto-part is fully
    developed, this function removes it from the cycle.  Some proto-parts may
    not produce any regulatory elements; when they are all that is left then
    discard them and end the function.

    Args:
        parts_developing: A list of undeveloped subpart objects, normally taken
                          from the output of initiate.setup_agent(genome).
                          Here, undeveloped means that the sum of regulatory-
                          elements of a subpart is 0.

    Returns:
        developed_parts: A list of subpart objects that are fully developed.
                         Here, fully developed means that the sum of
                         regulatory-elements of a part is one update cycle away
                         from the part's capacity.
    """
    diffusion_pull = .1
    diffusion_push = .05
    reg_pool = RegulatorPool(diffusion_pull, diffusion_push)
    developed_parts = list()
    while(parts_developing):
        count = 0
        for i in parts_developing:
            # Check for non-producing proto-parts, and if they are all
            # that is left, end development.
            if i.regulators_per_update == 0:
                count += 1
                i._diffusion(reg_pool)
                if count == len(parts_developing):
                    parts_developing.remove(i)
                    count -= 1
            # Check if proto-part is done with development, and take out
            # of the update_cycles if it is.
            elif (i.capacity <= (i.regulatory_elements +
                                 i.regulators_per_update)):
                i.set_characteristics()
                developed_parts.append(i)
                parts_developing.remove(i)
            # Otherwise, keep updating
            else:
                i.update()
                i._diffusion(reg_pool)
    return developed_parts


def iterate_mounts_used(ntuple):
    """Returns given namedtuple with its mounts_used value increased by 1.

    """
    return ntuple._replace(mounts_used=ntuple.mounts_used+1)


def select_frame_parts(developed_parts):
    """Returns the bodyparts and the jointparts that will be used for the frame

    The returned information also indicates which bodyparts are
    attached to each-other, using which jointparts.
    """
    bodypart_list = [i for i in developed_parts
                     if (i.__class__ == BodyPart and
                         i.characteristics.joint_mount_num > 0)]
    unused_bodys = [c for c in enumerate(bodypart_list)]
    used_bodys = list()
    joint_list = [i for i in developed_parts
                  if i.__class__ == JointPart]
    unused_joints = [c for c in enumerate(joint_list)]
    used_joints = list()

    # Get first base_part
    if unused_bodys:
        base_part = Body(body_index=unused_bodys[0][0],
                         mounts_used=0,
                         mounts_total=unused_bodys[0][1].characteristics.joint_mount_num)
        used_bodys.append(base_part)
        unused_bodys.pop(0)
        base_pointer = used_bodys[-1].body_index
    else:
        return [used_bodys, used_joints]

    while True:
        # Get next mounted part
        if unused_bodys:
            mounted_part = Body(body_index=unused_bodys[0][0],
                                mounts_used=0,
                                mounts_total=unused_bodys[0][1].characteristics.joint_mount_num)
            used_bodys.append(mounted_part)
            unused_bodys.pop(0)
            mounted_pointer = used_bodys[-1].body_index
        else:
            return [used_bodys, used_joints]
        # Get next joint part
        if unused_joints:
            bi = [i for i in xrange(len(used_bodys))
                  if used_bodys[i].body_index == base_pointer][0]
            joint_part = Joint(joint=unused_joints[0][0],
                               base_body=base_pointer,
                               base_mount=used_bodys[bi].mounts_used,
                               other_body=mounted_pointer)
            used_joints.append(joint_part)
            unused_joints.pop(0)
            used_bodys[bi] = iterate_mounts_used(used_bodys[bi])
            used_bodys[-1] = iterate_mounts_used(used_bodys[-1])
            base_pointer = mounted_pointer
        else:
            return [used_bodys[:-1], used_joints]
        # Get next base part
        if used_bodys[-1].mounts_used < used_bodys[-1].mounts_total:
            pass
        else:
            for ind, part in enumerate(used_bodys):
                if part.mounts_used < part.mounts_total:
                    base_pointer = used_bodys[ind].body_index
                    break
            else:
                return [used_bodys, used_joints]


def mounts_left(mount_list):
    """Returns true if there are unused mounts left in the given list
    
    """
    return any([part.mounts_used < part.mounts_total
                for part in mount_list])


def select_neuron_parts(developed_parts, frame_parts):
    """Returns the neurons to be used in the robots ANN.

       Even neurons with no input or output slots get put in
       the pool of neuron parts to be used.
    """
    neuron_list = [i for i in developed_parts
                   if i.__class__ == NeuronPart]
    unused_neurons = range(len(neuron_list))
    used_neurons = list()
    body_list = [i for i in developed_parts
                 if (i.__class__ == BodyPart and
                     i.characteristics.joint_mount_num > 0)]
    neuron_mounts = [Mount(part_index=part.body_index,
                           mounts_used=0,
                           mounts_total=body_list[part.body_index].characteristics.neuron_mount_num)
                     for part in frame_parts[0]]

    for i in itertools.cycle(range(len(neuron_mounts))):
        if unused_neurons and mounts_left(neuron_mounts):
            if mounts_left([neuron_mounts[i]]):
                neuron_part = Neuron(neuron=unused_neurons[0],
                                     body_index=neuron_mounts[i].part_index,
                                     mount_used=neuron_mounts[i].mounts_used)
                used_neurons.append(neuron_part)
                unused_neurons.pop(0)
                neuron_mounts[i] = iterate_mounts_used(neuron_mounts[i])
        else:
            return used_neurons


def select_sensor_parts(developed_parts, frame_parts):
    """Returns the sensors to be used in the robots ANN.

    Even sensors with no output slots get put in the
    pool of sensor parts to be used
    """
    sensor_list = [i for i in developed_parts
                   if i.__class__ == SensorPart]
    unused_sensors = range(len(sensor_list))
    used_sensors = list()
    body_list = [i for i in developed_parts
                 if (i.__class__ == BodyPart and
                     i.characteristics.joint_mount_num > 0)]
    sensor_mounts = [Mount(part_index=part.body_index,
                           mounts_used=0,
                           mounts_total=body_list[part.body_index].characteristics.sensor_mount_num)
                     for part in frame_parts[0]]

    for i in itertools.cycle(range(len(sensor_mounts))):
        if unused_sensors and mounts_left(sensor_mounts):
            if mounts_left([sensor_mounts[i]]):
                sensor_part = Sensor(sensor=unused_sensors[0],
                                     body_index=sensor_mounts[i].part_index,
                                     mount_used=sensor_mounts[i].mounts_used)
                used_sensors.append(sensor_part)
                unused_sensors.pop(0)
                sensor_mounts[i] = iterate_mounts_used(sensor_mounts[i])
        else:
            return used_sensors


def make_wire_part(part, case, generators):
    """Returns a Wire namedtuple; a wire part to be used.

    Helper function for select_wire_parts().
    """
    if case == 'S->J':
        return Wire(wire=part[0],
                    from_type=SensorPart,
                    from_index=generators[3].next(),
                    to_type=JointPart,
                    to_index=generators[0].next())
    elif case == 'S->N':
        return Wire(wire=part[0],
                    from_type=SensorPart,
                    from_index=generators[3].next(),
                    to_type=NeuronPart,
                    to_index=generators[1].next())
    elif case == 'N->J':
        return Wire(wire=part[0],
                    from_type=NeuronPart,
                    from_index=generators[2].next(),
                    to_type=JointPart,
                    to_index=generators[0].next())
    elif case == 'N->N':
        return Wire(wire=part[0],
                    from_type=NeuronPart,
                    from_index=generators[2].next(),
                    to_type=NeuronPart,
                    to_index=generators[1].next())
    else:
        raise ValueError


def jump_generator(generator, mount_list):
    """Skips generator ahead to an index where there are mounts
    available, and returns that index.

    Helper function for select_wire_parts().
    """
    while (mount_list[generator.peek()].mounts_used >=
           mount_list[generator.peek()].mounts_total):
        generator.next()
    return generator.peek()


def select_wire_parts(developed_parts, frame_parts,
                      neuron_parts, sensor_parts):
    """Returns the set of wires to be used in bulding the robot."""
    wire_list = [i for i in developed_parts
                 if i.__class__ == WirePart]
    unused_wires = [c for c in enumerate(wire_list)]
    used_wires = list()
    joint_list = [i for i in developed_parts
                  if i.__class__ == JointPart]
    neuron_list = [i for i in developed_parts
                   if i.__class__ == NeuronPart]
    sensor_list = [i for i in developed_parts
                   if i.__class__ == SensorPart]

    # Create mount information holders
    joint_ins = [Mount(part_index=part.joint,
                       mounts_used=0,
                       mounts_total=joint_list[part.joint].characteristics.input_num)
                 for part in frame_parts[1]]
    try:
        neuron_ins = [Mount(part_index=part.neuron,
                            mounts_used=0,
                            mounts_total=neuron_list[part.neuron].characteristics.input_num)
                      for part in neuron_parts]
    except TypeError:
        return used_wires
    neuron_outs = [Mount(part_index=part.neuron,
                         mounts_used=0,
                         mounts_total=neuron_list[part.neuron].characteristics.output_num)
                   for part in neuron_parts]
    try:
        sensor_outs = [Mount(part_index=part.sensor,
                             mounts_used=0,
                             mounts_total=sensor_list[part.sensor].characteristics.output_num)
                       for part in sensor_parts]
    except TypeError:
        return used_wires

    # Create generators to help iterate through the mount information holders
    joint_in_generator = peekable(itertools.cycle(range(len(joint_ins))))
    neuron_in_generator = peekable(itertools.cycle(range(len(neuron_ins))))
    neuron_out_generator = peekable(itertools.cycle(range(len(neuron_outs))))
    sensor_out_generator = peekable(itertools.cycle(range(len(sensor_outs))))
    generator_list = [joint_in_generator, neuron_in_generator,
                      neuron_out_generator, sensor_out_generator]
    from_sensor = True

    # Create wires
    for part in unused_wires:
        if ((mounts_left(joint_ins) or mounts_left(neuron_ins)) and
           (mounts_left(neuron_outs) or mounts_left(sensor_outs))):
            if part[1].characteristics.to_joint:
                if from_sensor:
                    if mounts_left(sensor_outs):
                        ind = jump_generator(sensor_out_generator, sensor_outs)
                        sensor_outs[ind] = iterate_mounts_used(sensor_outs[ind])
                        if mounts_left(joint_ins):
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'S->J', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'S->N', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                    else:
                        ind = jump_generator(neuron_out_generator, neuron_outs)
                        neuron_outs[ind] = iterate_mounts_used(neuron_outs[ind])
                        if mounts_left(joint_ins):
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'N->J', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'N->N', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                else:
                    if mounts_left(neuron_outs):
                        ind = jump_generator(neuron_out_generator, neuron_outs)
                        neuron_outs[ind] = iterate_mounts_used(neuron_outs[ind])
                        if mounts_left(joint_ins):
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'N->J', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'N->N', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                    else:
                        ind = jump_generator(sensor_out_generator, sensor_outs)
                        sensor_outs[ind] = iterate_mounts_used(sensor_outs[ind])
                        if mounts_left(joint_ins):
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'S->J', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'S->N', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
            else:
                if from_sensor:
                    if mounts_left(sensor_outs):
                        ind = jump_generator(sensor_out_generator, sensor_outs)
                        sensor_outs[ind] = iterate_mounts_used(sensor_outs[ind])
                        if mounts_left(neuron_ins):
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'S->N', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'S->J', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                    else:
                        ind = jump_generator(neuron_out_generator, neuron_outs)
                        neuron_outs[ind] = iterate_mounts_used(neuron_outs[ind])
                        if mounts_left(neuron_ins):
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'N->N', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'N->J', generator_list)
                            from_sensor = False
                            used_wires.append(wire_part)
                else:
                    if mounts_left(neuron_outs):
                        ind = jump_generator(neuron_out_generator, neuron_outs)
                        neuron_outs[ind] = iterate_mounts_used(neuron_outs[ind])
                        if mounts_left(neuron_ins):
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'N->N', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'N->J', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                    else:
                        ind = jump_generator(sensor_out_generator, sensor_outs)
                        sensor_outs[ind] = iterate_mounts_used(sensor_outs[ind])
                        if mounts_left(neuron_ins):
                            ind = jump_generator(neuron_in_generator, neuron_ins)
                            neuron_ins[ind] = iterate_mounts_used(neuron_ins[ind])
                            wire_part = make_wire_part(part, 'S->N', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
                        else:
                            ind = jump_generator(joint_in_generator, joint_ins)
                            joint_ins[ind] = iterate_mounts_used(joint_ins[ind])
                            wire_part = make_wire_part(part, 'S->J', generator_list)
                            from_sensor = True
                            used_wires.append(wire_part)
        else:
            return used_wires
    return used_wires


def select_ann_parts(developed_parts, frame_parts):
    """Returns a list containing the lists of neurons, sensors, and wires to
    be used in the robots ANN."""
    neuron_parts = select_neuron_parts(developed_parts, frame_parts)
    sensor_parts = select_sensor_parts(developed_parts, frame_parts)
    wire_parts = select_wire_parts(developed_parts, frame_parts,
                                neuron_parts, sensor_parts)
    return [neuron_parts, sensor_parts, wire_parts]
