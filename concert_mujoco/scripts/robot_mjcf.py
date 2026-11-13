#!/usr/bin/env python3
"""MJCF actuators, joint classes and initial pose of a modular robot, merged by xbot2_mujoco.

Usage: robot_mjcf.py URDF SRDF

Every actuated URDF joint gets an actuator named after it (the xbot2 bridge requires this):
- wheels (modular names them J_wheel_*), velocity-controlled as in the gz HAL: intvelocity,
  class concert_wheel;
- joints whose next actuated joints include a wheel (steering): position, class concert_steering;
- the others (arms): position, class concert_arm.
The initial pose is the SRDF's home group_state.
"""
import sys
import xml.etree.ElementTree as ET

ACTUATED = ('revolute', 'continuous', 'prismatic')


def is_wheel(joint):
    return joint.get('name').startswith('J_wheel')


def robot_mjcf(urdf, srdf):
    joints = urdf.findall('joint')
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint.find('parent').get('link'), []).append(joint)

    def next_actuated(link):
        """Actuated joints below link, not looking past the first actuated joint of each branch."""
        found = []
        for joint in by_parent.get(link, []):
            if joint.get('type') in ACTUATED:
                found.append(joint)
            else:
                found += next_actuated(joint.find('child').get('link'))
        return found

    mujoco = ET.Element('mujoco')
    actuator = ET.SubElement(mujoco, 'actuator')
    for joint in (j for j in joints if j.get('type') in ACTUATED):
        name = joint.get('name')
        if is_wheel(joint):
            tag, cls = 'intvelocity', 'concert_wheel'
        elif any(is_wheel(j) for j in next_actuated(joint.find('child').get('link'))):
            tag, cls = 'position', 'concert_steering'
        else:
            tag, cls = 'position', 'concert_arm'
        ET.SubElement(actuator, tag, {'class': cls, 'name': name, 'joint': name})
        setattribute = ET.SubElement(mujoco, 'setattribute', element='joint', name=name)
        ET.SubElement(setattribute, 'attribute', name='class', value=cls)

    q_init = ET.SubElement(mujoco, 'q_init')
    for state in srdf.findall('group_state'):
        if state.get('name') == 'home':
            for joint in state.findall('joint'):
                ET.SubElement(q_init, 'joint', name=joint.get('name'), value=joint.get('value'))

    ET.indent(mujoco)
    return ET.tostring(mujoco, encoding='unicode')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    print(robot_mjcf(ET.parse(sys.argv[1]).getroot(), ET.parse(sys.argv[2]).getroot()))
