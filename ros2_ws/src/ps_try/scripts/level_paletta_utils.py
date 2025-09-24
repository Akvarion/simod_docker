# --- UTIL ---
import math
from geometry_msgs.msg import TransformStamped
import rclpy

def quat_to_rpy(x, y, z, w):
    # ZYX
    t0 = 2.0*(w*x + y*z); t1 = 1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(t0, t1)
    t2 = 2.0*(w*y - z*x); t2 = 1.0 if t2>1.0 else (-1.0 if t2<-1.0 else t2)
    pitch = math.asin(t2)
    t3 = 2.0*(w*z + x*y); t4 = 1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class WristLeveler:
    """
    Mantiene orizzontale l'EE comandando SOLO gli ultimi due giunti (pitch, roll).
    Uso:
        lvl = WristLeveler(node, ee_frame='ur_left_tool0', base_frame='world',
                           idx_wrist_pitch=4, idx_wrist_roll=5,
                           kp_r=1.5, kd_r=0.1, kp_p=1.5, kd_p=0.1, vmax=1.5)
        lvl.enable(True)                      # quando vuoi attivarlo
        cmd = lvl.apply_on(cmd)               # modifica solo le ultime due voci
    """
    def __init__(self, node, *, ee_frame, base_frame='world',
                 idx_wrist_pitch=4, idx_wrist_roll=5,
                 kp_r=1.5, kd_r=0.1, kp_p=1.5, kd_p=0.1, vmax=1.5):
        self.n = node
        self.ee = ee_frame
        self.base = base_frame
        self.i_wp = idx_wrist_pitch
        self.i_wr = idx_wrist_roll
        self.kp_r, self.kd_r = kp_r, kd_r
        self.kp_p, self.kd_p = kp_p, kd_p
        self.vmax = vmax
        self.roll_des = 0.0
        self.pitch_des = 0.0
        self.enabled = False
        self.prev_e_r = 0.0
        self.prev_e_p = 0.0
        self.prev_t = node.get_clock().now()

    def set_target(self, roll_des=0.0, pitch_des=0.0):
        self.roll_des = roll_des
        self.pitch_des = pitch_des

    def enable(self, on: bool):
        # reset derivativa quando (ri)abiliti
        if on and not self.enabled:
            self.prev_e_r = 0.0
            self.prev_e_p = 0.0
            self.prev_t = self.n.get_clock().now()
        self.enabled = on

    def _compute_wrist_vel(self):
        # dt
        now = self.n.get_clock().now()
        dt = (now - self.prev_t).nanoseconds * 1e-9
        if dt <= 0.0: dt = 1e-3
        self.prev_t = now

        # TF base->EE
        try:
            tf: TransformStamped = self.n.tf_buffer.lookup_transform(
                self.base, self.ee, rclpy.time.Time())
        except Exception as e:
            self.n.get_logger().warn(
                f'[WristLeveler {self.ee}] TF {self.base}->{self.ee} missing: {e}')
            return 0.0, 0.0

        q = tf.transform.rotation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)

        # errori
        e_r = roll  - self.roll_des
        e_p = pitch - self.pitch_des
        de_r = (e_r - self.prev_e_r) / dt
        de_p = (e_p - self.prev_e_p) / dt
        self.prev_e_r, self.prev_e_p = e_r, e_p

        # PD -> velocità giunti (ultimo: roll; penultimo: pitch)
        w_r = - (self.kp_r*e_r + self.kd_r*de_r)
        w_p = - (self.kp_p*e_p + self.kd_p*de_p)

        # clamp
        vmax = self.vmax
        w_r = max(-vmax, min(vmax, w_r))
        w_p = max(-vmax, min(vmax, w_p))
        return w_p, w_r

    def apply_on(self, cmd_list=[0.0]*6):
        """
        Prende un vettore di comandi velocità (len>=max(i_wp,i_wr)+1),
        e scrive SOLO gli ultimi due (pitch, roll) con la correzione PD.
        Ritorna lo stesso vettore (comodità).
        """
        if not self.enabled:
            return cmd_list
        w_p, w_r = self._compute_wrist_vel()
        cmd_list[self.i_wp] = w_p
        cmd_list[self.i_wr] = w_r
        return cmd_list
