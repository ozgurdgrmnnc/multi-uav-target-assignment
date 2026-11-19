"""world_node: senaryonun "gerçek dünyası".

- Hedeflerin gerçek hareketini ve engelleri orijinal modelden üretir.
- Her adımda (step_period s) gürültülü ölçüm yayınlar  -> /wta/targets
  (orijinaldeki gibi ortak bir sensör ağı varsayımı; her İHA kendi UKF'sini çalıştırır).
- Engelleri 10 Hz yayınlar                              -> /wta/obstacles
- Gerçek konumları görselleştirme için yayınlar          -> /wta/targets_truth
- Yakalama hakemidir: görünür hedefe CAPTURE_RADIUS içinde giren İHA onu yakalar.
- Tüm İHA'lar kalkış irtifasına ulaşınca senaryo saatini başlatır.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import String

from iha_wta.core import params as P
from iha_wta.core.geometry import point_inside_any_obstacle
from iha_wta.core.scenario import get_all_obstacles, noisy_measurement, target_position
from iha_wta_msgs.msg import Target, TargetArray, UavState


class WorldNode(Node):
    def __init__(self):
        super().__init__("world_node")
        self.num_targets = self.declare_parameter("num_targets", 3).value
        self.num_uavs = self.declare_parameter("num_uavs", 3).value
        self.step_period = self.declare_parameter("step_period", 1.0).value
        self.max_steps = self.declare_parameter("max_steps", 400).value
        seed = self.declare_parameter("seed", 4).value
        self.rng = np.random.default_rng(seed)

        self.pub_targets = self.create_publisher(TargetArray, "/wta/targets", 10)
        self.pub_truth = self.create_publisher(TargetArray, "/wta/targets_truth", 10)
        self.pub_obstacles = self.create_publisher(PoseArray, "/wta/obstacles", 10)
        self.pub_events = self.create_publisher(String, "/wta/events", 10)
        self.create_subscription(UavState, "/wta/uav_states", self.on_uav_state, 50)

        self.uav_states = {}            # uav_id -> (UavState, alınma zamanı)
        self.active = set(range(self.num_targets))
        self.captured_events = []
        self.started = False
        self.mission_complete = False
        self.t0 = None
        self.step = 0

        self.create_timer(0.1, self.fast_loop)
        self.create_timer(self.step_period, self.step_loop)
        self.get_logger().info(
            f"{self.num_targets} hedef, {self.num_uavs} İHA bekleniyor "
            f"(adım süresi {self.step_period} s)")

    # ---------------------------------------------------------------
    def scenario_time(self):
        """Senaryo zamanı (adım birimi). Başlamadan önce 0'da sabit."""
        if not self.started:
            return 0.0
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9 / self.step_period

    def on_uav_state(self, msg):
        self.uav_states[msg.uav_id] = (msg, self.get_clock().now())

    def event(self, text):
        self.get_logger().info(text)
        self.pub_events.publish(String(data=text))

    def fresh_uav_positions(self):
        now = self.get_clock().now()
        out = {}
        for uid, (msg, stamp) in self.uav_states.items():
            if (now - stamp).nanoseconds < 1e9:
                out[uid] = np.array([msg.position.x, msg.position.y, msg.position.z])
        return out

    # ---------------------------------------------------------------
    def fast_loop(self):
        if not self.started:
            airborne = {uid for uid, (m, _) in self.uav_states.items() if m.airborne}
            if len(airborne) >= self.num_uavs:
                self.started = True
                self.t0 = self.get_clock().now()
                self.event("Tüm İHA'lar havada, senaryo başladı.")

        t = self.scenario_time()
        obstacles = get_all_obstacles(t)
        stamp = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = "world"
        for o in obstacles:
            pose = Pose()
            pose.position = Point(x=float(o[0]), y=float(o[1]), z=float(o[2]))
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_obstacles.publish(pa)

        truth = {tid: target_position(tid, t) for tid in range(self.num_targets)}
        hidden = {tid: point_inside_any_obstacle(p, obstacles) for tid, p in truth.items()}
        self.pub_truth.publish(self.make_array(truth, hidden, t, stamp))

        if self.started and not self.mission_complete:
            self.check_capture(truth, hidden, t)

    def check_capture(self, truth, hidden, t):
        uavs = self.fresh_uav_positions()
        if not uavs:
            return
        for tid in sorted(self.active):
            if hidden[tid]:
                continue
            dists = {uid: float(np.linalg.norm(p - truth[tid])) for uid, p in uavs.items()}
            best = min(dists, key=dists.get)
            if dists[best] < P.CAPTURE_RADIUS:
                self.active.discard(tid)
                self.captured_events.append((t, tid, best, dists[best]))
                self.event(f"YAKALANDI: hedef {tid}, İHA {best}, adım {t:.1f}, "
                           f"mesafe {dists[best]:.2f} m")

        if not self.active:
            self.finish("GÖREV TAMAM: tüm hedefler yakalandı.")
        elif t > self.max_steps:
            self.finish(f"SÜRE DOLDU: kalan hedefler {sorted(self.active)}")

    def finish(self, text):
        self.mission_complete = True
        self.event(text)
        for t, tid, uid, d in self.captured_events:
            self.get_logger().info(f"  adım={t:.1f} hedef={tid} iha={uid} mesafe={d:.2f}")

    def step_loop(self):
        """Her adımda bir gürültülü ölçüm yayınla (UKF bu hızda çalışır)."""
        t = self.scenario_time()
        obstacles = get_all_obstacles(t)
        meas, hidden = {}, {}
        if self.started:
            self.step += 1
            for tid in self.active:
                p = target_position(tid, t)
                meas[tid] = noisy_measurement(p, self.rng)
                hidden[tid] = point_inside_any_obstacle(p, obstacles)
        self.pub_targets.publish(
            self.make_array(meas, hidden, t, self.get_clock().now().to_msg(), only_active=True))

    def make_array(self, positions, hidden, t, stamp, only_active=False):
        arr = TargetArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "world"
        arr.step = self.step
        arr.sim_step = float(t)
        arr.started = self.started
        arr.mission_complete = self.mission_complete
        for tid, p in positions.items():
            if only_active and tid not in self.active:
                continue
            arr.targets.append(Target(
                id=int(tid),
                position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                hidden=bool(hidden[tid]),
                active=tid in self.active,
            ))
        return arr


def main():
    rclpy.init()
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
