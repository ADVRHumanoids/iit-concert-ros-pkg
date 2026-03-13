"""Isaac Lab simulation server for Concert robot with xbot2 socket communication.

Runs the Concert robot in Isaac Lab and exposes a UNIX DGRAM socket server
for bidirectional communication with xbot2 (via zmq_hal plugin).

Protocol:
  - Discovery: xbot2 sends {type: discovery} -> server replies with {joint_names, imu_sensors}
  - State:     server broadcasts {type: state, time, q, dq, tau, k, d, qref, vref, tauref, imu}
  - Control:   xbot2 sends {type: control, q, dq, tau} -> server applies targets

Usage (inside isaac-sim container):
    isaaclab -p /workspace/concert_isaac/run_sim.py --enable_cameras
    isaaclab -p /workspace/concert_isaac/run_sim.py --enable_cameras --real-time
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Concert robot simulation server for xbot2.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import time

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.assets.articulation import Articulation
from isaaclab.sensors.imu import Imu
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

import os
import socket
import yaml

##
# Import Concert config
# Option 1: if concert_isaac is installed as package (pip install -e .)
# Option 2: add to sys.path for direct import
##
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from assets.concert_play import CONCERT_CFG_PLAY  # noqa: E402


@configclass
class ConcertSceneCfg(InteractiveSceneCfg):
    """Configuration for the Concert simulation scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Concert robot
    robot: ArticulationCfg = CONCERT_CFG_PLAY.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop with xbot2 socket communication."""

    # Extract scene entities
    robot: Articulation = scene["robot"]
    imu_sensors: dict[str, Imu] = {}
    for sn, s in scene.sensors.items():
        print(f"Sensor name: {sn}, type: {type(s)}")
        if isinstance(s, Imu):
            print(f"IMU sensor found: {sn}")
            imu_sensors[sn] = s

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    time_sim = 0
    # real-time factor tracking
    rtf_last_print = time.time()
    rtf_sim_elapsed = 0.0
    rtf_steps = 0

    # Socket for communication with xbot2 (via zmq_hal)
    server_socket_path = "/tmp/.xbot2_isaac/xbot2_isaac_server.sock"
    os.makedirs(os.path.dirname(server_socket_path), exist_ok=True)
    if os.path.exists(server_socket_path):
        os.unlink(server_socket_path)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(server_socket_path)
    sock.setblocking(False)
    os.chmod(server_socket_path, 0o777)

    print(f"[Concert] Server socket created at {server_socket_path}")
    print(f"[Concert] Joint names: {robot.joint_names}")
    print(f"[Concert] Num joints: {robot.num_joints}")

    client_sockets = set()

    # Set initial joint positions
    qinit = robot.data.default_joint_pos.clone()
    robot.write_joint_position_to_sim(qinit)
    robot.set_joint_position_target(qinit)
    print(f'[Concert] Initial joint positions: {qinit.cpu().numpy().flatten().tolist()}')

    while simulation_app.is_running():

        tic = time.time()

        # Broadcast robot state to all connected xbot2 clients
        state_msg = {'type': 'state'}
        state_msg['time'] = time_sim
        state_msg['q'] = robot.data.joint_pos.cpu().numpy().flatten().tolist()
        state_msg['dq'] = robot.data.joint_vel.cpu().numpy().flatten().tolist()
        state_msg['tau'] = robot.data.applied_torque.cpu().numpy().flatten().tolist()
        state_msg['k'] = robot.data.joint_stiffness.cpu().numpy().flatten().tolist()
        state_msg['d'] = robot.data.joint_damping.cpu().numpy().flatten().tolist()
        state_msg['qref'] = robot.data.joint_pos_target.cpu().numpy().flatten().tolist()
        state_msg['vref'] = robot.data.joint_vel_target.cpu().numpy().flatten().tolist()
        state_msg['tauref'] = robot.data.joint_effort_target.cpu().numpy().flatten().tolist()

        state_msg['imu'] = dict()
        for imu_name, imu_sensor in imu_sensors.items():
            imu_data = imu_sensor.data
            state_msg['imu'][imu_name] = {
                'quat_w': imu_data.quat_w.cpu().numpy().flatten().tolist(),
                'lin_acc_b': imu_data.lin_acc_b.cpu().numpy().flatten().tolist(),
                'ang_vel_b': imu_data.ang_vel_b.cpu().numpy().flatten().tolist(),
            }

        state_msg = yaml.dump(state_msg)
        sockets_to_remove = []
        for cli_addr in client_sockets:
            try:
                sock.sendto(state_msg.encode(), cli_addr)
            except ConnectionRefusedError:
                print(f"[Concert] Client at {cli_addr} disconnected.")
                sockets_to_remove.append(cli_addr)
            except Exception as e:
                print(f"[Concert] Error sending state to {cli_addr}: {e}")

        for s in sockets_to_remove:
            client_sockets.remove(s)
        sockets_to_remove.clear()

        # Handle client connections
        try:
            data, cli_addr = sock.recvfrom(4096)

            # Consume buffer (keep only latest message)
            while True:
                try:
                    data, cli_addr = sock.recvfrom(4096)
                except BlockingIOError:
                    break

            data = data.decode('utf-8')
            data = yaml.safe_load(data)
            data_type = data['type']

            if data_type == 'discovery':
                response = {'type': 'discovery'}
                response['joint_names'] = robot.joint_names
                response['imu_sensors'] = list(imu_sensors.keys())
                try:
                    sock.sendto(yaml.dump(response).encode('utf-8'), cli_addr)
                except Exception as e:
                    print(f"[Concert] Error sending discovery response to {cli_addr}: {e}")
                client_sockets.add(cli_addr)
                print(f"[Concert] Client at {cli_addr} connected. Sent {len(robot.joint_names)} joints.")

            elif data_type == 'control':
                joint_pos_def = torch.tensor(data['q'], device=robot.device).unsqueeze(0)
                robot.set_joint_position_target(joint_pos_def)
                joint_vel_def = torch.tensor(data['dq'], device=robot.device).unsqueeze(0)
                robot.set_joint_velocity_target(joint_vel_def)
                joint_effort_def = torch.tensor(data['tau'], device=robot.device).unsqueeze(0)
                robot.set_joint_effort_target(joint_effort_def)

            else:
                print(f"[Concert] Unknown data type received: {data_type}")

        except BlockingIOError:
            pass

        # Simulation step
        scene.write_data_to_sim()
        sim.step()
        time_sim += sim_dt
        rtf_sim_elapsed += sim_dt
        rtf_steps += 1
        count += 1
        scene.update(sim_dt)

        # Print real-time factor every ~1s
        now = time.time()
        real_elapsed = now - rtf_last_print
        if real_elapsed >= 1.0:
            real_time_factor = rtf_sim_elapsed / real_elapsed if real_elapsed > 0 else float('inf')
            print(
                f"[Concert] RTF: {real_time_factor:.3f}x "
                f"(sim: {rtf_sim_elapsed:.3f}s, real: {real_elapsed:.3f}s, steps: {rtf_steps})"
            )
            rtf_last_print = now
            rtf_sim_elapsed = 0.0
            rtf_steps = 0

        # Time delay for real-time evaluation
        toc = time.time()
        sleep_time = sim_dt - (toc - tic)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_cfg = ConcertSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    print("[Concert] Setup complete. Waiting for xbot2 connection...")

    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
