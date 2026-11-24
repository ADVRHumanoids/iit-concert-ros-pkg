from modular.URDF_writer import *


# create UrdfWriter object and joint map to store homing values
urdf_writer = UrdfWriter(speedup=True)
homing_joint_map = {}

# urdf_writer.add_table()
# urdf_writer.add_socket(0.150, 0.225, 0.0, 0.0)
urdf_writer.add_module('socket.yaml')

# J1
data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml')
homing_joint_map[str(data['name'])] = 0.0

# J2
data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml')
homing_joint_map[str(data['name'])] = 0.5

# J3
data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml')
homing_joint_map[str(data['name'])] = 0.0

# J4
data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml')
homing_joint_map[data['name']] = 1.0

# J5
data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml')
homing_joint_map[str(data['name'])] = 0.0

# J6
data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml')
homing_joint_map[str(data['name'])] = 1.5


# gripper
# data = urdf_writer.add_module('module_gripper.yaml')
urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)


urdf_writer.write_file_to_stdout(homing_joint_map)