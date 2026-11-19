"""uav_agent_node: tek bir İHA'nın dağıtık karar + uçuş node'u.

Her İHA için ayrı bir süreç çalışır. Merkezî bir atama node'u yoktur:
  - Hedef ölçümleri (/wta/targets) ile KENDİ UKF'lerini günceller.
  - Her aktif hedef için aday üretir (intercept/loiter + A* + SciPy yumuşatma).
  - Diğer İHA'ların claim'lerini (/wta/claims) okuyup kendi hedefini seçer,
    kendi claim'ini yayınlar.
  - PX4'e offboard hız komutu gönderir; acil kaçınma 10 Hz'de önceliklidir.

İki iş parçacığı (thread):
  - ROS executor: 10 Hz kontrol döngüsü + abonelikler (hafif, asla bloklanmaz)
  - planner thread: ağır planlama (A*/SLSQP birkaç saniye sürebilir)
"""
import math
import threading
import time
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node

from iha_wta.core import params as P
from iha_wta.core.geometry import emergency_avoidance_direction
from iha_wta.core.ukf import UnscentedKalmanFilter3D
from iha_wta.core.wta import (ClaimInfo, build_candidate_for_target, build_ordered_wta_list,
                              claim_visible, select_candidate_targets, select_target)
from iha_wta.frames import (PX4_PUB_QOS, PX4_SUB_QOS, enu_heading_to_ned_yaw, enu_to_ned,
                            ned_to_enu, px4_topic)
from iha_wta_msgs.msg import Claim, TargetArray, UavState

NAN3 = [float("nan")] * 3


class UavAgentNode(Node):
    def __init__(self):
        super().__init__("uav_agent")
        self.uid = self.declare_parameter("uav_id", 0).value
        sx, sy, alt = P.uav_start(self.uid)
        self.spawn = np.array([
            self.declare_parameter("spawn_x", sx).value,
            self.declare_parameter("spawn_y", sy).value,
            0.0,
        ])
        self.start_alt = self.declare_parameter("start_alt", alt).value
        self.step_period = self.declare_parameter("step_period", 1.0).value
        ns = self.declare_parameter("px4_namespace", f"uav_{self.uid}").value
        # PX4 SITL'de MAV_SYS_ID = instance + 1
        self.sys_id = self.declare_parameter("px4_sys_id", self.uid + 1).value

        self.speed = P.UAV_SPEED / self.step_period          # m/s
        self.lookahead = P.UAV_SPEED                          # yol takibi: 1 adım ileri

        # --- PX4 ---
        self.pub_mode = self.create_publisher(
            OffboardControlMode, px4_topic(ns, "in", "offboard_control_mode", OffboardControlMode), PX4_PUB_QOS)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, px4_topic(ns, "in", "trajectory_setpoint", TrajectorySetpoint), PX4_PUB_QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, px4_topic(ns, "in", "vehicle_command", VehicleCommand), PX4_PUB_QOS)
        self.create_subscription(
            VehicleLocalPosition, px4_topic(ns, "out", "vehicle_local_position", VehicleLocalPosition),
            self.on_local_position, PX4_SUB_QOS)
        self.create_subscription(
            VehicleStatus, px4_topic(ns, "out", "vehicle_status", VehicleStatus),
            self.on_status, PX4_SUB_QOS)

        # --- İHA'lar arası iletişim ---
        self.pub_state = self.create_publisher(UavState, "/wta/uav_states", 10)
        self.pub_claim = self.create_publisher(Claim, "/wta/claims", 10)
        self.pub_path = self.create_publisher(Path, f"/wta/uav_{self.uid}/path", 10)
        self.create_subscription(UavState, "/wta/uav_states", self.on_uav_state, 50)
        self.create_subscription(Claim, "/wta/claims", self.on_claim, 50)
        self.create_subscription(TargetArray, "/wta/targets", self.on_targets, 50)
        self.create_subscription(PoseArray, "/wta/obstacles", self.on_obstacles, 10)

        # --- Durum (lock ile korunan, iki thread'in paylaştığı veriler) ---
        self.lock = threading.Lock()
        self.local_pos = None           # NED (yerel)
        self.local_vel = np.zeros(3)
        self.world_pos = None           # ENU (dünya)
        self.armed = False
        self.offboard = False
        self.obstacles = np.zeros((0, 3))
        self.others = {}                # uav_id -> (pos ENU, zaman)
        self.claims = {}                # uav_id -> (Claim, zaman)
        self.meas_queue = []
        self.world_started = False
        self.mission_complete = False

        self.path = None                # takip edilen yol (ENU)
        self.target_id = -1
        self.plan_mode = "IDLE"
        self.prev_claim_tid = -1
        self.last_claim = None          # son karar; 2 Hz tekrar yayınlanır

        # Uçuş durum makinesi: ARMING -> TAKEOFF -> READY -> MISSION -> DONE
        self.state = "ARMING"
        self.mode = "ARMING"
        self.tick = 0
        self.hold_ned = None
        self.yaw = float("nan")

        self.ukfs = {}
        self.active = set()
        self.hidden = {}

        self.create_timer(0.1, self.control_loop)
        self.running = True
        self.planner = threading.Thread(target=self.planner_loop, daemon=True)
        self.planner.start()
        self.get_logger().info(
            f"İHA {self.uid}: PX4 ns=/{ns} sys_id={self.sys_id} doğuş={self.spawn[:2]} "
            f"başlangıç irtifası={self.start_alt} m, hız={self.speed:.2f} m/s")

    # =================================================================
    # Abonelikler
    # =================================================================
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_local_position(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return
        ned = np.array([msg.x, msg.y, msg.z], dtype=float)
        with self.lock:
            self.local_pos = ned
            self.local_vel = np.array([msg.vx, msg.vy, msg.vz], dtype=float)
            self.world_pos = self.spawn + ned_to_enu(ned)

    def on_status(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def on_uav_state(self, msg):
        if msg.uav_id == self.uid:
            return
        with self.lock:
            self.others[msg.uav_id] = (
                np.array([msg.position.x, msg.position.y, msg.position.z]), self.now_s())

    def on_claim(self, msg):
        if msg.uav_id == self.uid:
            return
        with self.lock:
            self.claims[msg.uav_id] = (msg, self.now_s())

    def on_targets(self, msg):
        self.world_started = msg.started
        self.mission_complete = msg.mission_complete
        if msg.started:
            with self.lock:
                self.meas_queue.append(msg)

    def on_obstacles(self, msg):
        obs = np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses]).reshape(-1, 3)
        with self.lock:
            self.obstacles = obs

    def fresh_others(self, max_age=1.0):
        now = self.now_s()
        return {uid: p for uid, (p, t) in self.others.items() if now - t < max_age}

    # =================================================================
    # 10 Hz kontrol döngüsü
    # =================================================================
    def control_loop(self):
        with self.lock:
            local = None if self.local_pos is None else self.local_pos.copy()
            pos = None if self.world_pos is None else self.world_pos.copy()
        if local is None:
            return
        self.tick += 1

        if self.state == "ARMING":
            if self.hold_ned is None:
                self.hold_ned = np.array([local[0], local[1], -self.start_alt])
            self.send_position(self.hold_ned)
            # ~1 s setpoint akışından sonra offboard + arm iste (1 Hz tekrar).
            if self.tick >= 10 and self.tick % 10 == 0 and not (self.armed and self.offboard):
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.armed and self.offboard:
                self.state = self.mode = "TAKEOFF"
                self.get_logger().info("Arm + offboard tamam, kalkış.")

        elif self.state == "TAKEOFF":
            self.send_position(self.hold_ned)
            if abs(local[2] - self.hold_ned[2]) < 0.5:
                self.state = self.mode = "READY"
                self.get_logger().info(f"{self.start_alt} m irtifada, görev başlangıcı bekleniyor.")

        elif self.state == "READY":
            self.send_position(self.hold_ned)
            if self.world_started:
                self.state = "MISSION"

        elif self.state == "MISSION":
            if self.mission_complete:
                self.state = self.mode = "DONE"
                self.hold_ned = local.copy()
                self.get_logger().info("Görev tamamlandı, olduğu yerde bekliyor.")
            else:
                v = self.mission_velocity(pos)
                self.send_velocity(v)

        elif self.state == "DONE":
            self.send_position(self.hold_ned)

        self.publish_state(pos)
        # Planlama turu uzun sürse bile claim'imiz diğerlerinde zaman aşımına
        # düşmesin diye son kararı 2 Hz'de tekrar yayınla.
        if self.last_claim is not None and self.tick % 5 == 0:
            self.last_claim.header.stamp = self.get_clock().now().to_msg()
            self.pub_claim.publish(self.last_claim)

    def mission_velocity(self, pos):
        """Acil kaçınma > yol takibi. Dönüş: ENU hız vektörü (m/s)."""
        with self.lock:
            obstacles = self.obstacles.copy()
            others = list(self.fresh_others().values())
            path = None if self.path is None else self.path.copy()
            plan_mode = self.plan_mode

        avoid, emergency, severity, _ = emergency_avoidance_direction(pos, obstacles, others)
        if emergency:
            v = avoid * self.speed * P.EMERGENCY_SPEED_MULTIPLIER
            self.mode = f"EMERGENCY-AVOID-{severity}"
        elif path is not None:
            v = self.follow_path(pos, path)
            self.mode = plan_mode
        else:
            v = np.zeros(3)
            self.mode = "WAIT-PLAN"

        # Çalışma alanı sınırlarından dışarı doğru hareket etme.
        for a in range(3):
            if (pos[a] <= P.LOWER[a] and v[a] < 0) or (pos[a] >= P.UPPER[a] and v[a] > 0):
                v[a] = 0.0
        return v

    def follow_path(self, pos, path):
        """Pure-pursuit: yol üzerinde en yakın noktadan `lookahead` kadar
        ilerideki noktaya doğru UAV hızında git. Orijinaldeki "path[1]-path[0]
        yönünde UAV_SPEED kadar ilerle" adımının sürekli zamandaki karşılığı."""
        i = int(np.argmin(np.linalg.norm(path - pos, axis=1)))
        remaining = self.lookahead
        prev = path[i]
        carrot = path[-1]
        for j in range(i + 1, len(path)):
            seg = float(np.linalg.norm(path[j] - prev))
            if seg >= remaining:
                carrot = prev + (path[j] - prev) * (remaining / seg)
                break
            remaining -= seg
            prev = path[j]
        d = carrot - pos
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return np.zeros(3)
        return d / n * min(self.speed, n)    # yolun sonuna yaklaşırken yavaşla

    # =================================================================
    # PX4'e gönderim
    # =================================================================
    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_offboard_mode(self, position, velocity):
        m = OffboardControlMode()
        m.timestamp = self.timestamp_us()
        m.position = position
        m.velocity = velocity
        self.pub_mode.publish(m)

    def send_position(self, ned):
        self.send_offboard_mode(position=True, velocity=False)
        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp_us()
        sp.position = [float(x) for x in ned]
        sp.velocity = NAN3
        sp.yaw = self.yaw
        self.pub_sp.publish(sp)

    def send_velocity(self, v_enu):
        self.send_offboard_mode(position=False, velocity=True)
        if math.hypot(v_enu[0], v_enu[1]) > 0.5:
            self.yaw = enu_heading_to_ned_yaw(v_enu)
        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp_us()
        sp.position = NAN3
        sp.velocity = [float(x) for x in enu_to_ned(v_enu)]
        sp.yaw = self.yaw
        self.pub_sp.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self.timestamp_us()
        c.command = command
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.target_system = self.sys_id
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def publish_state(self, pos):
        s = UavState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "world"
        s.uav_id = self.uid
        s.position.x, s.position.y, s.position.z = (float(x) for x in pos)
        with self.lock:
            vel = ned_to_enu(self.local_vel)
        s.velocity.x, s.velocity.y, s.velocity.z = (float(x) for x in vel)
        s.target_id = int(self.target_id)
        s.mode = self.mode
        s.airborne = self.state in ("READY", "MISSION", "DONE")
        self.pub_state.publish(s)

    # =================================================================
    # Planlama thread'i
    # =================================================================
    def planner_loop(self):
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            try:
                if self.state == "MISSION":
                    self.plan_once()
            except Exception:
                self.get_logger().error("Planlama hatası:\n" + traceback.format_exc())
            time.sleep(max(0.05, self.step_period - (time.monotonic() - t0)))

    def plan_once(self):
        t_start = time.monotonic()
        with self.lock:
            msgs, self.meas_queue = self.meas_queue, []
            pos = self.world_pos.copy()
            obstacles = self.obstacles.copy()
            others = self.fresh_others()

        # 1. Ölçümlerle kendi UKF'lerini güncelle (kaçırılan adımlar sırayla işlenir).
        for msg in msgs:
            self.active = {t.id for t in msg.targets if t.active}
            for t in msg.targets:
                ukf = self.ukfs.setdefault(
                    t.id, UnscentedKalmanFilter3D(dt=P.DT, target_id=t.id))
                ukf.step([t.position.x, t.position.y, t.position.z])
                self.hidden[t.id] = t.hidden
        active = {tid for tid in self.active if tid in self.ukfs}
        if not active:
            return
        est = {tid: self.ukfs[tid].position() for tid in active}

        # 2. Aday üret (diğer İHA'lar da engel sayılır, orijinaldeki gibi).
        plan_obs = np.vstack([obstacles] + [p for p in others.values()]).reshape(-1, 3)
        cands = [build_candidate_for_target(self.uid, tid, pos, self.hidden.get(tid, False), est[tid],
                                            self.ukfs[tid], plan_obs)
                 for tid in select_candidate_targets(pos, active, est)]
        ordered = build_ordered_wta_list(cands, active)

        # 3. Dağıtık WTA: görünür claim'lere göre hedef seç. Claim'ler aday
        #    üretiminden SONRA okunur; planlama uzun sürdüyse bile karar güncel
        #    bilgiyle verilir.
        with self.lock:
            now = self.now_s()
            claims = {uid: c for uid, (c, t) in self.claims.items()
                      if now - t < 3.0 * self.step_period}
        other_claims = []
        for uid, c in claims.items():
            if c.target_id not in active:
                continue
            if P.USE_COMM_RADIUS_FOR_WTA and (uid not in others or not claim_visible(others[uid], pos)):
                continue    # iletişim menzili dışındaki İHA'nın claim'i duyulmaz
            other_claims.append(ClaimInfo(c.uav_id, c.target_id, c.cost, c.continuing))
        sel, status = select_target(self.uid, ordered, active, self.prev_claim_tid, other_claims)

        if sel is None:
            # Fallback: en yakın hedef tahminine doğru düz git (claim etmeden).
            tid = min(active, key=lambda k: np.linalg.norm(est[k] - pos))
            path, mode, cost, claim_tid = np.vstack([pos, est[tid]]), "LOCAL-FALLBACK-NEAREST", 1e18, -1
        else:
            tid, path, mode, cost = sel["target_id"], sel["path"], sel["mode"], sel["base_cost"]
            claim_tid = tid

        c = Claim()
        c.header.stamp = self.get_clock().now().to_msg()
        c.uav_id = self.uid
        c.target_id = int(claim_tid)
        c.cost = float(cost)
        c.continuing = status == "SERVICING-CONTINUED"
        c.status = status
        self.pub_claim.publish(c)
        self.last_claim = c

        if claim_tid != self.prev_claim_tid:
            order = [cd["target_id"] for cd in ordered]
            self.get_logger().info(
                f"WTA sırası={order} -> claim {f'T{claim_tid}' if claim_tid >= 0 else 'yok'} "
                f"({status}, {mode}, maliyet={cost if claim_tid >= 0 else float('nan'):.2f}, "
                f"planlama {time.monotonic() - t_start:.1f} s)")
        self.prev_claim_tid = claim_tid

        with self.lock:
            self.path = np.asarray(path, dtype=float)
            self.plan_mode = mode
            self.target_id = tid

        pm = Path()
        pm.header.stamp = c.header.stamp
        pm.header.frame_id = "world"
        for p in self.path:
            ps = PoseStamped()
            ps.header = pm.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(x) for x in p)
            ps.pose.orientation.w = 1.0
            pm.poses.append(ps)
        self.pub_path.publish(pm)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = UavAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
