#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import argparse, time
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import (
    SetModelState,
    GetEntityState,
    SetEntityState,
)
from gazebo_msgs.msg import ModelState, EntityState
import tf2_ros
from rclpy.duration import Duration
from std_srvs.srv import Empty

# Servizi del link-attacher (aggiorna i tipi se il tuo plugin usa nomi diversi)
from linkattacher_msgs.srv import AttachLink, DetachLink


# ========= CONFIG GENERALI =========
# Durate (s)
APPROACH_TIME = 20.0
DESCEND_AND_PICK_TIME = 12.0
TRANSPORT_TIME = 20.0
RELEASE_TIME = 10.0

# Velocità base in approach
APPROACH_LEFT_BASE_XY_VEL  = (0.002, 0.24)
APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.24)

# Velocità bracci (placeholder: usa le tue preferite)
LEFT_ARM_VEL   =  [0.07, -0.01, 0.01, -0.01, -0.01, -0.01]
RIGHT_ARM_VEL  = [-0.09, -0.01, 0.01, -0.01,  0.01,  0.021]

# Pose/velocità per calata e presa
LEFT_ARM_PICK   =  [-0.001, -0.08, 0.08, 0.0, -0.12, -0.05]
RIGHT_ARM_PICK  =  [ 0.001, -0.12, 0.08, 0.0,  0.12,  0.05]

# Velocità base nel trasporto
LEFT_TRANSPORT_VEL_XY  = (-0.002, -0.24)
RIGHT_TRANSPORT_VEL_XY = ( 0.02 , -0.24)


class PickAndPlaceDemo(Node):
    def __init__(self, args):
        super().__init__('pick_and_place_demo')

        # Oggetto da manipolare
        self.object_name = args.object             # es. 'pacco_clone_1'
        self.object_link = args.object_link        # es. 'pacco_clone_1::link_1'

        # Robot/EE per ATTACH (sinistro e destro)
        self.attach_robot_model_left  = args.attach_robot_model_left    # 'left_robot'
        self.attach_robot_link_left   = args.attach_robot_link_left     # 'ur_left_wrist_3_link'
        self.attach_robot_model_right = args.attach_robot_model_right   # 'right_robot'
        self.attach_robot_link_right  = args.attach_robot_link_right    # 'ur_right_wrist_3_link'

        # Topic movimento
        self.left_base_topic  = args.left_base_topic
        self.right_base_topic = args.right_base_topic
        self.left_arm_topic   = args.left_arm_topic
        self.right_arm_topic  = args.right_arm_topic

        # Publisher
        self.left_base_pub  = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)
        self.left_arm_pub   = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_arm_pub  = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)

        # TF (per eventuale debug/overlay in RViz)
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo services (per eventuali snap/set pose)
        self.cli_set_model_state  = self.create_client(SetModelState,   '/gazebo/set_model_state')
        self.cli_get_entity_state = self.create_client(GetEntityState,  '/state/get_entity_state')
        self.cli_set_entity_state = self.create_client(SetEntityState,  '/state/set_entity_state')

        # Link Attacher services
        self.attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_cli = self.create_client(DetachLink, '/DETACHLINK')

        # Pause/Unpause fisica
        self.pause_cli   = self.create_client(Empty, '/pause_physics')
        self.unpause_cli = self.create_client(Empty, '/unpause_physics')

        # Attendi servizi essenziali con timeout morbido
        deadline = time.time() + 10.0
        while time.time() < deadline:
            ok_model = (self.cli_set_model_state.service_is_ready() or
                        self.cli_set_entity_state.service_is_ready())
            ok_attach = self.attach_cli.service_is_ready() and self.detach_cli.service_is_ready()
            ok_pause = self.pause_cli.service_is_ready() and self.unpause_cli.service_is_ready()
            if ok_model and ok_attach and ok_pause:
                break
            try:
                if not self.cli_set_model_state.service_is_ready():
                    self.cli_set_model_state.wait_for_service(timeout_sec=0.3)
                if not self.cli_set_entity_state.service_is_ready():
                    self.cli_set_entity_state.wait_for_service(timeout_sec=0.3)
                if not self.attach_cli.service_is_ready():
                    self.attach_cli.wait_for_service(timeout_sec=0.3)
                if not self.detach_cli.service_is_ready():
                    self.detach_cli.wait_for_service(timeout_sec=0.3)
                if not self.pause_cli.service_is_ready():
                    self.pause_cli.wait_for_service(timeout_sec=0.3)
                if not self.unpause_cli.service_is_ready():
                    self.unpause_cli.wait_for_service(timeout_sec=0.3)
            except Exception:
                pass

        # State machine
        self.state = 'approach'
        self.state_start = time.time()
        self.durations = {
            'approach': APPROACH_TIME,
            'descend_and_pick': DESCEND_AND_PICK_TIME,
            'transport': TRANSPORT_TIME,
            'release': RELEASE_TIME
        }
        self.attached_left  = False
        self.attached_right = False

        # Timer controllo a 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('PickAndPlaceDemo (doppio attach) avviato @50Hz')

    # ===== Loop di controllo / State machine =====
    def control_loop(self):
        now = time.time()
        elapsed = now - self.state_start

        if self.state == 'approach':
            # Muovi basi verso l’oggetto
            tl, tr = Twist(), Twist()
            tl.linear.x, tl.linear.y = APPROACH_LEFT_BASE_XY_VEL
            tr.linear.x, tr.linear.y = APPROACH_RIGHT_BASE_XY_VEL
            self.left_base_pub.publish(tl)
            self.right_base_pub.publish(tr)
            # Muovi bracci (velocità giunti)
            la, ra = Float64MultiArray(), Float64MultiArray()
            la.data, ra.data = LEFT_ARM_VEL, RIGHT_ARM_VEL
            self.left_arm_pub.publish(la)
            self.right_arm_pub.publish(ra)

            if elapsed > self.durations['approach']:
                self.stop_all_movement()
                self.transition('descend_and_pick')

        elif self.state == 'descend_and_pick':
            # Calata/chiusura paletta
            la, ra = Float64MultiArray(), Float64MultiArray()
            la.data, ra.data = LEFT_ARM_PICK, RIGHT_ARM_PICK
            self.left_arm_pub.publish(la)
            self.right_arm_pub.publish(ra)

            if elapsed > self.durations['descend_and_pick']:
                self.stop_all_movement()
                # ==== ATTACH SINISTRO ====
                ok_left = self.attach_fixed_joint(
                    model1=self.attach_robot_model_left,
                    link1=self.attach_robot_link_left,
                    model2=self.object_name,
                    link2=self.object_link
                )
                self.attached_left = ok_left
                if ok_left:
                    self.get_logger().info('Attach OK (sinistro): giunto fisso creato')
                else:
                    self.get_logger().warn('Attach sinistro fallito: proseguo comunque')

                # ==== ATTACH DESTRO con pausa fisica (minimizza impulso) ====
                self.call_pause_physics()
                # (opzionale) qui potresti allineare il polso destro alla stessa posa di contatto
                ok_right = self.attach_fixed_joint(
                    model1=self.attach_robot_model_right,
                    link1=self.attach_robot_link_right,
                    model2=self.object_name,
                    link2=self.object_link
                )
                self.attached_right = ok_right
                if ok_right:
                    self.get_logger().info('Attach OK (destro): giunto fisso creato')
                else:
                    self.get_logger().warn('Attach destro fallito: proseguo con solo lato sinistro')
                self.call_unpause_physics()

                self.transition('transport')

        elif self.state == 'transport':
            tl, tr = Twist(), Twist()
            tl.linear.x, tl.linear.y = LEFT_TRANSPORT_VEL_XY
            tr.linear.x, tr.linear.y = RIGHT_TRANSPORT_VEL_XY
            self.left_base_pub.publish(tl)
            self.right_base_pub.publish(tr)
            # Con attach fisico non teletrasportiamo l'oggetto

            if elapsed > self.durations['transport']:
                self.stop_all_movement()
                self.transition('release')

        elif self.state == 'release':
            # Stacca in ordine inverso: prima destro, poi sinistro
            if True or self.attached_right: # --- IGNORE --- funziona anche se restituisce False
                ok = self.detach_fixed_joint(
                    model1=self.attach_robot_model_right,
                    link1=self.attach_robot_link_right,
                    model2=self.object_name,
                    link2=self.object_link
                )
                if ok:
                    self.get_logger().info('Detach OK (destro)')
                else:
                    self.get_logger().warn('Detach destro fallito')
                self.attached_right = False

            if True or self.attached_left: # --- IGNORE --- funziona anche se restituisce False
                ok = self.detach_fixed_joint(
                    model1=self.attach_robot_model_left,
                    link1=self.attach_robot_link_left,
                    model2=self.object_name,
                    link2=self.object_link
                )
                if ok:
                    self.get_logger().info('Detach OK (sinistro)')
                else:
                    self.get_logger().warn('Detach sinistro fallito')
                self.attached_left = False

            if elapsed > self.durations['release']:
                self.get_logger().info('Demo completata')
                self.timer.cancel()

    def transition(self, new_state):
        self.state = new_state
        self.state_start = time.time()
        self.get_logger().info(f'Transition -> {new_state}')

    def stop_all_movement(self):
        zero = Twist()
        self.left_base_pub.publish(zero)
        self.right_base_pub.publish(zero)
        z = Float64MultiArray(); z.data = [0.0]*6
        self.left_arm_pub.publish(z)
        self.right_arm_pub.publish(z)

    # ===== Attach/Detach: servizi link-attacher =====
    def attach_fixed_joint(self, model1, link1, model2, link2) -> bool:
        if not self.attach_cli.service_is_ready():
            try: self.attach_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.attach_cli.service_is_ready():
            self.get_logger().warn('ATTACHLINK non disponibile')
            return False

        req = AttachLink.Request()
        # Adatta i campi se il tuo plugin usa nomi diversi
        setattr(req, 'model1_name', model1)
        setattr(req, 'link1_name',  link1)
        setattr(req, 'model2_name', model2)
        setattr(req, 'link2_name',  link2)

        fut = self.attach_cli.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        except Exception:
            pass

        if not fut.done() or fut.result() is None:
            return False

        res = fut.result()
        ok = getattr(res, 'success', None)
        if ok is None:
            ok = getattr(res, 'ok', False)
        return bool(ok)

    def detach_fixed_joint(self, model1, link1, model2, link2) -> bool:
        if not self.detach_cli.service_is_ready():
            try: self.detach_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.detach_cli.service_is_ready():
            self.get_logger().warn('DETACHLINK non disponibile')
            return False

        req = DetachLink.Request()
        setattr(req, 'model1_name', model1)
        setattr(req, 'link1_name',  link1)
        setattr(req, 'model2_name', model2)
        setattr(req, 'link2_name',  link2)

        fut = self.detach_cli.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        except Exception:
            pass

        if not fut.done() or fut.result() is None:
            return False

        res = fut.result()
        ok = getattr(res, 'success', None)
        if ok is None:
            ok = getattr(res, 'ok', False)
        return bool(ok)

    # ===== Pause/Unpause fisica =====
    def call_pause_physics(self):
        if not self.pause_cli.service_is_ready():
            try: self.pause_cli.wait_for_service(timeout_sec=0.5)
            except Exception: pass
        if self.pause_cli.service_is_ready():
            self.pause_cli.call_async(Empty.Request())

    def call_unpause_physics(self):
        if not self.unpause_cli.service_is_ready():
            try: self.unpause_cli.wait_for_service(timeout_sec=0.5)
            except Exception: pass
        if self.unpause_cli.service_is_ready():
            self.unpause_cli.call_async(Empty.Request())

    # ===== (Opzionale) set pose oggetto con Gazebo services =====
    def call_set_model_state(self, model_name, pose: Pose, reference_frame='world') -> bool:
        # Utile solo per eventuale snap iniziale (qui non lo usiamo durante attach)
        if not self.cli_set_model_state.service_is_ready():
            try: self.cli_set_model_state.wait_for_service(timeout_sec=0.5)
            except Exception: pass
        if self.cli_set_model_state.service_is_ready():
            req = SetModelState.Request()
            state = ModelState()
            state.model_name = model_name
            state.pose = pose
            state.reference_frame = reference_frame
            state.twist.linear.x = state.twist.linear.y = state.twist.linear.z = 0.0
            state.twist.angular.x = state.twist.angular.y = state.twist.angular.z = 0.0
            req.model_state = state
            fut = self.cli_set_model_state.call_async(req)
            try:
                rclpy.spin_until_future_complete(self, fut, timeout_sec=0.4)
            except Exception:
                pass
            if fut.done() and fut.result() is not None:
                return bool(getattr(fut.result(), 'success', True))

        if not self.cli_set_entity_state.service_is_ready():
            try: self.cli_set_entity_state.wait_for_service(timeout_sec=0.5)
            except Exception: pass
        if self.cli_set_entity_state.service_is_ready():
            req2 = SetEntityState.Request()
            ent = EntityState()
            ent.name = model_name
            ent.pose = pose
            ent.reference_frame = reference_frame
            req2.state = ent
            fut2 = self.cli_set_entity_state.call_async(req2)
            try:
                rclpy.spin_until_future_complete(self, fut2, timeout_sec=0.4)
            except Exception:
                pass
            if fut2.done() and fut2.result() is not None:
                return bool(getattr(fut2.result(), 'success', True))

        return False


def main():
    parser = argparse.ArgumentParser(
        description='Pick&Place Gazebo Classic (ROS 2) con doppio attach fisico (paletta sinistra e destra).'
    )
    # Oggetto
    parser.add_argument('--object', default='pacco_clone_1',
                        help='Gazebo model name of the object to pick')
    parser.add_argument('--object_link', default='pacco_clone_1::link_1',
                        help='Full link name model::link (es. pacco_clone_1::link_1)')

    # Robot/link usati per ATTACH (sinistro e destro)
    parser.add_argument('--attach_robot_model_left',  default='left_robot',
                        help='Model name del robot sinistro che attacca per primo')
    parser.add_argument('--attach_robot_link_left',   default='ur_left_wrist_3_link',
                        help='Link (end-effector/paletta) del robot sinistro')
    parser.add_argument('--attach_robot_model_right', default='right_robot',
                        help='Model name del robot destro (secondo attach)')
    parser.add_argument('--attach_robot_link_right',  default='ur_right_wrist_3_link',
                        help='Link (end-effector/paletta) del robot destro')

    # Topic movimento
    parser.add_argument('--left_base_topic',  default='/left_summit/cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit/cmd_vel')
    parser.add_argument('--left_arm_topic',   default='/left/ur_left_joint_group_vel_controller/commands')
    parser.add_argument('--right_arm_topic',  default='/right/ur_right_joint_group_vel_controller/commands')

    args = parser.parse_args()

    rclpy.init()
    node = PickAndPlaceDemo(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
