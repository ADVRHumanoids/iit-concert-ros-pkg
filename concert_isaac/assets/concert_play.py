"""Configuration for the Concert mobile robot (IIT).

The following configurations are available:

* :obj:`CONCERT_CFG_PLAY`: Concert with steering, wheels, and arm chain E (for xbot2 control)
* :obj:`CONCERT_BASE_ONLY_CFG_PLAY`: Concert with only steering and wheels (no arm)
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

import os

##
# USD path resolution
# The deployment USD is committed with this package under assets/usd.
##
_ASSETS_DIR = os.path.abspath(os.path.dirname(__file__))
_CONCERT_USD = os.path.join(
    _ASSETS_DIR, "usd", "concert_complete", "concert_complete.usd"
)

if not os.path.exists(_CONCERT_USD):
    raise FileNotFoundError(
        f"Concert deployment USD not found at '{_CONCERT_USD}'. "
        "The canonical asset set lives under concert_isaac/assets/usd."
    )


##
# Concert Joint Structure (from URDF):
#
# Steering:  J1_A, J1_B, J1_C, J1_D        (revolute, effort=127 Nm, vel=8.1 rad/s)
# Wheels:    J_wheel_A, J_wheel_B, ...      (revolute, effort=24 Nm,  vel=9.5 rad/s)
# Arm:       J1_E, J2_E (large motors)      (revolute, effort=460 Nm, vel=2.14 rad/s)
#            J4_E       (large motor)        (revolute, effort=460 Nm, vel=2.14 rad/s)
#            J3_E, J5_E, J6_E (medium)      (revolute, effort=314 Nm, vel=2.85 rad/s)
#
# Wheel radius: 0.16m
#
# NOTE: regex "J1_[A-D]" matches steering but NOT J1_E (arm base)
##


# --- Full Concert: steering + wheels + arm ---
CONCERT_CFG_PLAY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_CONCERT_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # TODO: verify spawn height — this is an estimate based on wheel radius (0.16m)
        # and kinematic chain offsets. Load in Isaac Sim GUI and check if robot
        # sits correctly on the ground plane. Adjust as needed.
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            # Steering joints — all at 0 (straight ahead)
            "J1_A": 0.0,
            "J1_B": 0.0,
            "J1_C": 0.0,
            "J1_D": 0.0,
            # Wheel joints — all at 0
            "J_wheel_A": 0.0,
            "J_wheel_B": 0.0,
            "J_wheel_C": 0.0,
            "J_wheel_D": 0.0,
            # Arm chain E — validated deployment home position
            "J1_E": 0.0,
            "J2_E": -0.5,
            "J3_E": 0.0,
            "J4_E": 0.5,
            "J5_E": 0.0,
            "J6_E": -0.5,
        }
    ),
    actuators={
        # Steering motors (J1_A, J1_B, J1_C, J1_D)
        # From URDF: effort=127 Nm, velocity=8.1 rad/s
        # From ModularBot_impd4.yaml: stiffness=500, damping=20
        "motor_steering": DCMotorCfg(
            joint_names_expr=["J1_[A-D]"],
            saturation_effort=127,
            effort_limit=127,
            velocity_limit=8.1,
            stiffness=500,
            damping=20,
            armature=0.382,
            friction=3.2,
            dynamic_friction=3.2,
            viscous_friction=6.75,
        ),
        # Wheel motors (J_wheel_A, J_wheel_B, J_wheel_C, J_wheel_D)
        # From URDF: effort=24 Nm, velocity=9.5 rad/s
        # Pure velocity control: stiffness=0 (same pattern as Kyon wheels)
        "motor_wheel": DCMotorCfg(
            joint_names_expr=["J_wheel_.*"],
            saturation_effort=24,
            effort_limit=24,
            velocity_limit=9.5,
            stiffness=0,
            damping=10,
            armature=0.078,
            friction=1.0,
            dynamic_friction=1.0,
            viscous_friction=0.7,
        ),
        # Arm large motors (J1_E, J2_E, J4_E)
        # From URDF: effort=460 Nm, velocity=2.14 rad/s
        "motor_arm_large": DCMotorCfg(
            joint_names_expr=["J[124]_E"],
            saturation_effort=460,
            effort_limit=460,
            velocity_limit=2.14,
            stiffness=500,
            damping=20,
            armature=0.472,
            friction=2.75,
            dynamic_friction=2.75,
            viscous_friction=5.1,
        ),
        # Arm medium motors (J3_E, J5_E, J6_E)
        # From URDF: effort=314 Nm, velocity=2.85 rad/s
        "motor_arm_medium": DCMotorCfg(
            joint_names_expr=["J[356]_E"],
            saturation_effort=314,
            effort_limit=314,
            velocity_limit=2.85,
            stiffness=500,
            damping=20,
            armature=0.382,
            friction=3.2,
            dynamic_friction=3.2,
            viscous_friction=6.75,
        ),
    },
)


# --- Concert base only: steering + wheels, no arm ---
CONCERT_BASE_ONLY_CFG_PLAY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_CONCERT_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            "J1_A": 0.0,
            "J1_B": 0.0,
            "J1_C": 0.0,
            "J1_D": 0.0,
            "J_wheel_A": 0.0,
            "J_wheel_B": 0.0,
            "J_wheel_C": 0.0,
            "J_wheel_D": 0.0,
        }
    ),
    actuators={
        "motor_steering": DCMotorCfg(
            joint_names_expr=["J1_[A-D]"],
            saturation_effort=127,
            effort_limit=127,
            velocity_limit=8.1,
            stiffness=500,
            damping=20,
            armature=0.382,
            friction=3.2,
            dynamic_friction=3.2,
            viscous_friction=6.75,
        ),
        "motor_wheel": DCMotorCfg(
            joint_names_expr=["J_wheel_.*"],
            saturation_effort=24,
            effort_limit=24,
            velocity_limit=9.5,
            stiffness=0,
            damping=10,
            armature=0.078,
            friction=1.0,
            dynamic_friction=1.0,
            viscous_friction=0.7,
        ),
    },
)
