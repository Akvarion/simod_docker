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

# Servizi del link-attacher (assicurati di avere il pacchetto corretto)
# Se i nomi dei tipi nel tuo plugin sono diversi, aggiorna questi import.
from linkattacher_msgs.srv import AttachLink, DetachLink

from gazebo_link_gravity_toggle.srv import SetLinkGravity
from gazebo_collision_toggle.srv import SetCollisionEnabled

# ========= CONFIG GENERALI =========
# Durate (s)
APPROACH_TIME = 20.5
DESCEND_AND_PICK_TIME = 12.0
TRANSPORT_TIME = 20.0
RELEASE_TIME = 10.0

# Velocità base in approach
APPROACH_LEFT_BASE_XY_VEL  = (0.0019, 0.247)
APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.247)

# Velocità bracci (esempio; usa le tue)
LEFT_ARM_VEL   =  [0.078, -0.01, 0.01, -0.01, -0.01, -0.01]
RIGHT_ARM_VEL  = [-0.09, -0.01, 0.01, -0.01, 0.01, 0.022]

# Pose per “calata” e presa (comandi velocità giunti o target semplici)
LEFT_ARM_PICK   =  [-0.001, -0.082, 0.08, 0.0, -0.12, -0.057]
RIGHT_ARM_PICK  =  [0.001, -0.12, 0.08, 0.0, 0.12, 0.051]

# Velocità base nel trasporto
LEFT_TRANSPORT_VEL_XY  = (-0.002, -0.24)
RIGHT_TRANSPORT_VEL_XY = ( 0.02 , -0.24)

class PickAndPlaceDemo(Node):
    def __init__(self, args):
        super().__init__('pick_and_place_demo')

        # Oggetto da manipolare
        self.object_name = args.object             # es. 'pacco_clone_1'
        self.object_link = args.object_link        # es. 'pacco_clone_1::link_1'

        # Robot/EE per attach fisico
        self.attach_robot_model = args.attach_robot_model   # 'left_robot'
        self.attach_robot_link  = args.attach_robot_link    # 'ur_left_wrist_3_link'

        # End-effector frames
        self.left_ee  = args.left_ee
        self.right_ee = args.right_ee

        # Topic
        self.left_base_topic  = args.left_base_topic
        self.right_base_topic = args.right_base_topic
        self.left_arm_topic   = args.left_arm_topic
        self.right_arm_topic  = args.right_arm_topic

        # Publisher
        self.left_base_pub  = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)
        self.left_arm_pub   = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_arm_pub  = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)

        # TF (se vuoi usarlo per debug o overlay in RViz)
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo services (per eventuali snap/fallback o set pose finale)
        self.cli_set_model_state  = self.create_client(SetModelState,   '/gazebo/set_model_state')
        self.cli_get_entity_state = self.create_client(GetEntityState,  '/state/get_entity_state')
        self.cli_set_entity_state = self.create_client(SetEntityState,  '/state/set_entity_state')
        # service per toggle gravità link
        self.toggle_gravity_cli = self.create_client(SetLinkGravity, '/set_link_gravity')
        # service per toggle collisioni link
        self.toggle_collision_cli = self.create_client(SetCollisionEnabled, '/set_collision_enabled')


        # Link Attacher services
        self.attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_cli = self.create_client(DetachLink, '/DETACHLINK')

        # Attendi servizi essenziali con timeout morbido
        deadline = time.time() + 10.0
        while time.time() < deadline:
            ok_model = (self.cli_set_model_state.service_is_ready() or
                        self.cli_set_entity_state.service_is_ready())
            ok_attach = self.attach_cli.service_is_ready() and self.detach_cli.service_is_ready()
            if ok_model and ok_attach:
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
        self.attached = False

        # Timer controllo a 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('PickAndPlaceDemo (attach fisico) avviato @50Hz')

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
                self.set_paletta_collision(False, side='left')  # disabilita collisione paletta sinistra
                self.set_paletta_collision(False, side='right')  # disabilita collisione paletta destra
                self.transition('descend_and_pick')

        elif self.state == 'descend_and_pick':
            # Calata/chiusura paletta
            la, ra = Float64MultiArray(), Float64MultiArray()
            la.data, ra.data = LEFT_ARM_PICK, RIGHT_ARM_PICK
            self.left_arm_pub.publish(la)
            self.right_arm_pub.publish(ra)

            if elapsed > self.durations['descend_and_pick']:
                self.stop_all_movement()

                # # Try to lookup EE pose
                # try:
                #     trans = self.tf_buffer.lookup_transform('world', self.left_ee, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                # except Exception as e:
                #     # On failure, also try to dump known frames from the buffer to help debugging
                #     frames_info = ''
                #     try:
                #         frames_info = self.tf_buffer.all_frames_as_yaml()
                #     except Exception as _e2:
                #         frames_info = f'<failed to list frames: {_e2}>'
                #     self.get_logger().error(f'Failed to get transform for {self.left_ee}:\n        {e}\nAvailable TF frames:\n{frames_info}')
                #     return False

                # pose = Pose()
                # pose.position.x = trans.transform.translation.x
                # pose.position.y = trans.transform.translation.y
                # pose.position.z = trans.transform.translation.z + 0.5
                # pose.orientation = trans.transform.rotation

                # # snap iniziale PRIMA dell’attach (opzionale)
                # self.call_set_model_state(
                #     model_name=self.object_name,
                #     pose=pose,  # personalizza se vuoi
                #     reference_frame='world'
                # )

                # ==== ATTACH FISICO ====
                ok = self.attach_fixed_joint(
                    model1=self.attach_robot_model,
                    link1=self.attach_robot_link,
                    model2=self.object_name,
                    link2=self.object_link
                )
                if ok:
                    self.attached = True
                    self.get_logger().info('Attach OK: giunto fisso creato (paletta sinistra ↔ pacco)')
                else:
                    self.get_logger().warn(f'Attach fallito: proseguo comunque (l’oggetto potrebbe cadere)')
                
                self.set_pacco_gravity(False)

                self.transition('transport')

        elif self.state == 'transport':
            tl, tr = Twist(), Twist()
            tl.linear.x, tl.linear.y = LEFT_TRANSPORT_VEL_XY
            tr.linear.x, tr.linear.y = RIGHT_TRANSPORT_VEL_XY
            self.left_base_pub.publish(tl)
            self.right_base_pub.publish(tr)

            # Con attach fisico NON teletrasportare il pacco: ci pensa la fisica
            # if not self.attached:
            #     (eventuale fallback di follow, sconsigliato mentre c’è joint)

            if elapsed > self.durations['transport']:
                self.stop_all_movement()
                self.transition('release')

        elif self.state == 'release':
            if True or self.attached: # funziona anche se attach fallito
                # ==== DETACH ====
                self.set_paletta_collision(True, side='left')  # riabilita collisione paletta sinistra
                self.set_paletta_collision(True, side='right')  # riabilita collisione paletta destra
                ok = self.detach_fixed_joint(
                    model1=self.attach_robot_model,
                    link1=self.attach_robot_link,
                    model2=self.object_name,
                    link2=self.object_link
                )
                if ok:
                    self.get_logger().info('Detach OK: giunto rimosso')
                else:
                    self.get_logger().warn('Detach fallito (il pacco potrebbe restare agganciato)')

                self.attached = False

                self.set_pacco_gravity(True)  # riattiva gravità sul pacco

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
        # NB: i nomi dei campi potrebbero variare leggermente secondo il plugin.
        # Adatta qui se i tuoi sono diversi (es. model_name_1, link_name_1, ...)
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

        # Il campo di esito può chiamarsi success o ok a seconda del repo
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

    # ===== (Opzionale) set pose oggetto con Gazebo services =====
    def call_set_model_state(self, model_name, pose: Pose, reference_frame='world') -> bool:
        # Utile se vuoi fare uno snap iniziale PRIMA dell’attach
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

        # Fallback: /state/set_entity_state
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
    
    def set_pacco_gravity(self, gravity_on: bool):
        if not self.toggle_gravity_cli.service_is_ready():
            try: self.toggle_gravity_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.toggle_gravity_cli.service_is_ready():
            self.get_logger().warn('toggle_gravity service not ready'); return False
        req = SetLinkGravity.Request()
        req.model_name = ''                 # uso forma scoped sotto
        req.link_name  = 'pacco_clone_1::link_1'
        req.gravity    = gravity_on
        fut = self.toggle_gravity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.done() and fut.result() and fut.result().success


    def set_paletta_collision(self, enable: bool, side='left'):
        if not self.toggle_collision_cli.service_is_ready():
            try:
                self.toggle_collision_cli.wait_for_service(timeout_sec=1.0)
            except Exception:
                self.get_logger().warn('set_collision_enabled wait failed')
        if not self.toggle_collision_cli.service_is_ready():
            self.get_logger().warn('set_collision_enabled not ready')
            return False

        req = SetCollisionEnabled.Request()
        if side == 'left':
            req.model_name = 'left_robot'
            req.link_name  = 'ur_left_wrist_3_link'
        else:
            req.model_name = 'right_robot'
            req.link_name  = 'ur_right_wrist_3_link'

        req.collision_name = ''   # tutte le collision del link
        req.enable = enable

        fut = self.toggle_collision_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not (fut.done() and fut.result()):
            self.get_logger().warn('set_collision_enabled call failed')
            return False

        res = fut.result()
        self.get_logger().info(f"paletta {side}: collisions {'ENABLED' if enable else 'DISABLED'} — {res.message}")
        return bool(res.success)

def main():
    parser = argparse.ArgumentParser(
        description='Pick&Place demo Gazebo Classic (ROS 2) con attach fisico (fixed joint).'
    )
    # Oggetto
    parser.add_argument('--object', default='pacco_clone_1',
                        help='Gazebo model name of the object to pick')
    parser.add_argument('--object_link', default='pacco_clone_1::link_1',
                        help='Full link name model::link (es. pacco_clone_1::link_1)')

    # Robot/link usati per ATTACH (sinistro)
    parser.add_argument('--attach_robot_model', default='left_robot',
                        help='Model name of the robot that will attach the object')
    parser.add_argument('--attach_robot_link', default='ur_left_wrist_3_link',
                        help='Link name (end-effector/paletta) of the attaching robot')

    # Topic movimento
    parser.add_argument('--left_base_topic',  default='/left_summit/cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit/cmd_vel')
    parser.add_argument('--left_arm_topic',   default='/left/ur_left_joint_group_vel_controller/commands')
    parser.add_argument('--right_arm_topic',  default='/right/ur_right_joint_group_vel_controller/commands')

    parser.add_argument('--left_ee', default='ur_left_paletta')
    parser.add_argument('--right_ee', default='ur_right_paletta')

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
