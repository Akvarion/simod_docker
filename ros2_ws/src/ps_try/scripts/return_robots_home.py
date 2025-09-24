#!/usr/bin/env python3
"""
Teleport two robots to the centre of the world, facing each other.

Usage (from a sourced ROS2 workspace with Gazebo running):

python3 return_robots_home.py [--left_model LEFT] [--right_model RIGHT] [--y_offset 0.5]

Defaults try common model names ('left_summit','left_robot') and ('right_summit','right_robot').
The script calls /gazebo/set_model_state and will try alternative model names if the first attempt fails.
"""

import math
import argparse
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState, GetModelList, GetEntityState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelState, EntityState
import json
from geometry_msgs.msg import Pose


def yaw_to_quaternion(yaw: float):
    """Return quaternion dict (x,y,z,w) for a yaw angle (radians)."""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class ReturnRobotsHome(Node):
    def __init__(self, args):
        super().__init__('return_robots_home')
        self.declare_parameter('left_model', args.left_model)
        self.declare_parameter('right_model', args.right_model)

        self.left_candidates = args.left_candidates or []
        self.right_candidates = args.right_candidates or []

        # default target layout: centred on x=0, separated on y
        self.x = args.x
        self.y_offset = args.y_offset

        # orientations: face each other along y axis
        self.left_yaw = args.left_yaw
        self.right_yaw = args.right_yaw

        # create clients for different possible Gazebo APIs
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.set_entity_state_client = self.create_client(SetEntityState, '/state/set_entity_state')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.get_entity_state_client = self.create_client(GetEntityState, '/state/get_entity_state')

        # wait a short time for any of the clients to become available; don't require a specific one
        start = time.time()
        timeout = 5.0
        while time.time() - start < timeout:
            if (self.set_model_state_client.service_is_ready() or
                    self.set_entity_state_client.service_is_ready() or
                    self.get_model_list_client.service_is_ready()):
                break
            time.sleep(0.1)

        if not (self.set_model_state_client.service_is_ready() or
                self.set_entity_state_client.service_is_ready() or
                self.get_model_list_client.service_is_ready()):
            self.get_logger().warn('No Gazebo services appear ready (/gazebo/set_model_state or /state/set_entity_state or /get_model_list)')

    def teleport_model(self, model_name: str, x: float, y: float, yaw: float, reference_frame='world') -> bool:
        # prefer SetModelState if available
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # try to preserve current z if available to avoid collisions
        try:
            cur = self.get_entity_state(model_name)
            if cur is not None and hasattr(cur, 'pose') and cur.pose is not None:
                cz = getattr(cur.pose.position, 'z', None)
                if cz is not None and abs(cz) > 1e-6:
                    pose.position.z = float(cz)
                    self.get_logger().info(f'Using current z={pose.position.z:.3f} for model {model_name}')
        except Exception:
            pass

        if self.set_model_state_client.service_is_ready():
            req = SetModelState.Request()
            state = ModelState()
            state.model_name = model_name
            state.pose = pose
            state.reference_frame = reference_frame
            req.model_state = state
            future = self.set_model_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                try:
                    res = future.result()
                    if hasattr(res, 'success') and res.success:
                        self.get_logger().info(f"Teleported '{model_name}' via SetModelState to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                        return True
                    else:
                        msg = getattr(res, 'status_message', '<no message>')
                        self.get_logger().warn(f"SetModelState failed for '{model_name}': {msg}")
                except Exception as e:
                    self.get_logger().error(f'Exception waiting SetModelState result for {model_name}: {e}')

        # fallback to state/set_entity_state if available; include a small retry
        if self.set_entity_state_client.service_is_ready():
            req2 = SetEntityState.Request()
            ent = EntityState()
            ent.name = model_name
            ent.pose = pose
            ent.reference_frame = reference_frame
            req2.state = ent
            for attempt in range(2):
                future2 = self.set_entity_state_client.call_async(req2)
                rclpy.spin_until_future_complete(self, future2, timeout_sec=2.0)
                if future2.done():
                    try:
                        res2 = future2.result()
                        if hasattr(res2, 'success') and res2.success:
                            self.get_logger().info(f"Teleported '{model_name}' via SetEntityState to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                            return True
                        else:
                            self.get_logger().warn(f'SetEntityState returned success=False for {model_name} (attempt {attempt+1})')
                    except Exception as e:
                        self.get_logger().error(f'Exception waiting SetEntityState result for {model_name}: {e}')
                time.sleep(0.1)

        self.get_logger().error(f'No suitable service available to teleport {model_name}')
        return False

    def get_model_list(self):
        """Call /get_model_list to retrieve models present in the world."""
        if not self.get_model_list_client.service_is_ready():
            # try a short wait
            if not self.get_model_list_client.wait_for_service(timeout_sec=1.0):
                return []
        req = GetModelList.Request()
        future = self.get_model_list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done():
            try:
                res = future.result()
                names = []
                if hasattr(res, 'model_names'):
                    names = list(res.model_names)
                return names
            except Exception:
                return []
        return []

    def get_entity_state(self, model_name: str):
        """Call /state/get_entity_state to retrieve the current EntityState for a model."""
        if not self.get_entity_state_client.service_is_ready():
            if not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
                return None
        req = GetEntityState.Request()
        req.name = model_name
        future = self.get_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done():
            try:
                res = future.result()
                # res may contain 'state' or similar
                if hasattr(res, 'state'):
                    return res.state
                return None
            except Exception:
                return None
        return None

    def save_snapshot(self, filename: str, model_names=None):
        """Save poses of model_names (or all models) to a JSON file."""
        if model_names is None:
            model_names = self.get_model_list()
        data = {}
        for m in model_names:
            st = self.get_entity_state(m)
            if st is None:
                self.get_logger().warn(f'Could not get state for {m}')
                continue
            p = st.pose
            data[m] = {
                'position': [p.position.x, p.position.y, p.position.z],
                'orientation': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            }
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Snapshot saved to {filename} ({len(data)} models)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to write snapshot {filename}: {e}')
            return False

    def restore_snapshot(self, filename: str, only_models=None):
        """Restore poses from a JSON snapshot file. If only_models provided, restore only those names."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read snapshot {filename}: {e}')
            return False

        names = list(data.keys())
        if only_models:
            names = [n for n in names if n in only_models]

        ok = True
        for n in names:
            rec = data[n]
            pos = rec.get('position', [0, 0, 0])
            ori = rec.get('orientation', [0, 0, 0, 1])
            # build pose
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(ori[0])
            pose.orientation.y = float(ori[1])
            pose.orientation.z = float(ori[2])
            pose.orientation.w = float(ori[3])

            # try to teleport using available services
            if self.set_model_state_client.service_is_ready():
                req = SetModelState.Request()
                state = ModelState()
                state.model_name = n
                state.pose = pose
                state.reference_frame = 'world'
                req.model_state = state
                future = self.set_model_state_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if not (future.done() and getattr(future.result(), 'success', False)):
                    self.get_logger().warn(f'SetModelState failed for {n}, trying SetEntityState')
                    # fallback below
            if self.set_entity_state_client.service_is_ready():
                req2 = SetEntityState.Request()
                ent = EntityState()
                ent.name = n
                ent.pose = pose
                ent.reference_frame = 'world'
                req2.state = ent
                future2 = self.set_entity_state_client.call_async(req2)
                rclpy.spin_until_future_complete(self, future2, timeout_sec=1.0)
                # accept success if call did not error
        return ok

    def run(self, left_models, right_models):
        # Try candidate names for left, right
        left_done = False
        right_done = False

        left_y = self.y_offset
        right_y = -self.y_offset

        for lm in left_models:
            if self.teleport_model(lm, self.x, left_y, self.left_yaw):
                left_done = True
                break
            time.sleep(0.1)

        for rm in right_models:
            if self.teleport_model(rm, self.x, right_y, self.right_yaw):
                right_done = True
                break
            time.sleep(0.1)

        if not left_done:
            self.get_logger().error(f'Failed to teleport any left model from candidates: {left_models}')
        if not right_done:
            self.get_logger().error(f'Failed to teleport any right model from candidates: {right_models}')

        return left_done and right_done


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--action', choices=['teleport', 'save', 'restore'], default='teleport', help='Action to perform')
    parser.add_argument('--snapshot', default='sim_snapshot.json', help='Snapshot filename for save/restore')
    parser.add_argument('--left_model', default='left_summit', help='Primary left robot model name')
    parser.add_argument('--right_model', default='right_summit', help='Primary right robot model name')
    parser.add_argument('--left_candidates', nargs='*', default=['left_summit', 'left_robot', 'ur_left', 'ur_left_robot'], help='Alternate left model names to try')
    parser.add_argument('--right_candidates', nargs='*', default=['right_summit', 'right_robot', 'ur_right', 'ur_right_robot'], help='Alternate right model names to try')
    parser.add_argument('--x', type=float, default=0.0, help='X coordinate for both robots')
    parser.add_argument('--y_offset', type=float, default=0.6, help='Absolute Y offset from centre (left positive, right negative)')
    parser.add_argument('--left_yaw', type=float, default=-math.pi/2.0, help='Yaw angle for left robot (radians)')
    parser.add_argument('--right_yaw', type=float, default=math.pi/2.0, help='Yaw angle for right robot (radians)')
    args = parser.parse_args()

    # Build candidate preference lists (primary first)
    left_models = [args.left_model] + [c for c in args.left_candidates if c != args.left_model]
    right_models = [args.right_model] + [c for c in args.right_candidates if c != args.right_model]

    rclpy.init()
    node = None
    try:
        node = ReturnRobotsHome(args)
        if args.action == 'save':
            node.save_snapshot(args.snapshot)
        elif args.action == 'restore':
            # restore only left/right models if present in snapshot
            node.restore_snapshot(args.snapshot, only_models=[args.left_model, args.right_model])
        else:
            ok = node.run(left_models, right_models)
            if ok:
                node.get_logger().info('Both robots teleported to home positions')
            else:
                node.get_logger().warn('One or both robots failed to teleport; check model names and Gazebo service')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
