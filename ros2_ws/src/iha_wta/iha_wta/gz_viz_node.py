"""gz_viz_node: senaryoyu Gazebo GUI'sinde marker olarak çizer.

Engeller, hedefler, İHA'ların planlı yolları ve atama çizgileri yalnızca
görseldir; fiziksel çarpışma yoktur. (Engeller algoritma için "sanal" tehlike
bölgeleridir, tıpkı orijinal koddaki gibi.)

Gazebo ile doğrudan gz-transport Python bağlantısıyla konuşur:
  /marker_array  (gz.msgs.Marker_V) -> GUI'deki MarkerManager eklentisi
  /gui/move_to/pose (gz.msgs.GUICamera) -> kamerayı sahneye çevirir
  /gui/track (gz.msgs.CameraTrack)      -> PX4'ün drone takibini kapatır
GUI kapalıysa (HEADLESS) istekler zaman aşımına uğrar ve sessizce atlanır.

Not: Gazebo Harmonic'in ogre2 motoru TEXT marker çizmez; bu yüzden etiket
yok, kimlikler renkle ayırt edilir (aşağıdaki LEGEND).
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.cameratrack_pb2 import CameraTrack
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Path
from rclpy.node import Node

from iha_wta.core import params as P
from iha_wta_msgs.msg import TargetArray, UavState

UAV_COLORS = [(0.22, 0.74, 0.97), (0.13, 0.77, 0.37), (0.98, 0.45, 0.09),
              (0.65, 0.55, 0.98), (0.96, 0.25, 0.37)]
TARGET_COLORS = [(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (1.0, 0.65, 0.0),
                 (0.0, 1.0, 0.0), (0.5, 0.0, 0.5), (0.6, 0.3, 0.1)]
LEGEND = ("Renkler -> İHA: U0 mavi, U1 yeşil, U2 turuncu, U3 mor, U4 pembe | "
          "Hedef: T0 kırmızı, T1 eflatun, T2 sarı-turuncu, T3 yeşil | "
          "beyaz çizgi = planlı yol, renkli ince çizgi = geçmiş iz, "
          "İHA'dan hedefe çizgi = claim, gri küçük küre = yakalanmış hedef")


def set_color(marker, rgb, alpha=1.0):
    for c in (marker.material.ambient, marker.material.diffuse):
        c.r, c.g, c.b, c.a = rgb[0], rgb[1], rgb[2], alpha
    e = marker.material.emissive
    e.r, e.g, e.b, e.a = rgb[0] * 0.4, rgb[1] * 0.4, rgb[2] * 0.4, alpha


class GzVizNode(Node):
    def __init__(self):
        super().__init__("gz_viz_node")
        self.num_uavs = self.declare_parameter("num_uavs", 3).value
        self.gz = GzNode()
        self.track_pub = self.gz.advertise("/gui/track", CameraTrack)

        self.obstacles = np.zeros((0, 3))
        self.targets = []
        self.uavs = {}
        self.paths = {}
        self.trails = {}                # uav_id -> geçmiş konumlar (iz çizgisi)
        self.prev_obstacle_count = 0
        self.camera_set = False

        self.create_subscription(PoseArray, "/wta/obstacles", self.on_obstacles, 10)
        self.create_subscription(TargetArray, "/wta/targets_truth", self.on_targets, 10)
        self.create_subscription(UavState, "/wta/uav_states", self.on_uav, 50)
        for uid in range(self.num_uavs):
            self.create_subscription(
                Path, f"/wta/uav_{uid}/path", lambda m, uid=uid: self.on_path(uid, m), 10)
        self.create_timer(0.2, self.draw)
        self.get_logger().info(LEGEND)

    def on_obstacles(self, msg):
        self.obstacles = np.array([[p.position.x, p.position.y, p.position.z]
                                   for p in msg.poses]).reshape(-1, 3)

    def on_targets(self, msg):
        self.targets = list(msg.targets)

    def on_uav(self, msg):
        self.uavs[msg.uav_id] = msg
        p = (msg.position.x, msg.position.y, msg.position.z)
        trail = self.trails.setdefault(msg.uav_id, deque(maxlen=400))
        if not trail or math.dist(trail[-1], p) > 1.0:
            trail.append(p)

    def on_path(self, uid, msg):
        self.paths[uid] = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                           for p in msg.poses]

    # ---------------------------------------------------------------
    def new_marker(self, mv, ns, mid, mtype):
        m = mv.marker.add()
        m.ns = ns
        # Gazebo'da id=0 "yeni id üret" demektir: her çağrıda yeni marker
        # eklenir ve üst üste birikir. Bu yüzden id'ler 1'den başlar.
        m.id = mid + 1
        m.action = Marker.ADD_MODIFY
        m.type = mtype
        m.visibility = Marker.GUI
        m.pose.orientation.w = 1.0
        return m

    def sphere(self, mv, ns, mid, p, diameter, rgb, alpha):
        m = self.new_marker(mv, ns, mid, Marker.SPHERE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.scale.x = m.scale.y = m.scale.z = float(diameter)
        set_color(m, rgb, alpha)

    def delete(self, mv, ns, mid):
        m = mv.marker.add()
        m.ns, m.id, m.action = ns, mid + 1, Marker.DELETE_MARKER

    def line(self, mv, ns, mid, mtype, points, rgb):
        m = self.new_marker(mv, ns, mid, mtype)
        for p in points:
            q = m.point.add()
            q.x, q.y, q.z = (float(v) for v in p)
        set_color(m, rgb)

    def draw(self):
        mv = Marker_V()
        if not self.camera_set:
            self.camera_set = self.move_camera()
            if self.camera_set:
                # Yeni açılmış GUI: önceki çalıştırmalardan kalan marker'ları sil.
                clear = mv.marker.add()
                clear.action = Marker.DELETE_ALL
        # Engeller: SAFETY_RADIUS yarıçaplı, içindeki dronlar görünsün diye çok saydam
        for i, o in enumerate(self.obstacles):
            self.sphere(mv, "obstacles", i, o, 2 * P.SAFETY_RADIUS, (0.55, 0.62, 0.30), 0.10)
        for i in range(len(self.obstacles), self.prev_obstacle_count):
            self.delete(mv, "obstacles", i)
        self.prev_obstacle_count = len(self.obstacles)

        # Hedefler: aktif = renkli, yakalanmış = gri ve küçük
        target_pos = {}
        for t in self.targets:
            p = (t.position.x, t.position.y, t.position.z)
            target_pos[t.id] = p
            col = TARGET_COLORS[t.id % len(TARGET_COLORS)]
            if t.active:
                self.sphere(mv, "targets", t.id, p, 3.0, col, 1.0)
                # yakalama yarıçapı
                self.sphere(mv, "capture", t.id, p, 2 * P.CAPTURE_RADIUS, col, 0.15)
            else:
                self.sphere(mv, "targets", t.id, p, 1.5, (0.5, 0.5, 0.5), 0.6)
                self.delete(mv, "capture", t.id)

        # İHA'lar: saydam küre (drone içinden görünsün) + yerden direk,
        # geçmiş iz, planlı yol (beyaz), atanan hedefe çizgi
        for uid, s in self.uavs.items():
            col = UAV_COLORS[uid % len(UAV_COLORS)]
            p = (s.position.x, s.position.y, s.position.z)
            self.sphere(mv, "uavs", uid, p, 3.0, col, 0.35)
            self.line(mv, "poles", uid, Marker.LINE_LIST, [(p[0], p[1], 0.0), p], col)
            trail = self.trails.get(uid, ())
            if len(trail) >= 2:
                self.line(mv, "trails", uid, Marker.LINE_STRIP, list(trail) + [p], col)
            if len(self.paths.get(uid, [])) >= 2:
                self.line(mv, "paths", uid, Marker.LINE_STRIP, self.paths[uid], (1.0, 1.0, 1.0))
            if s.target_id in target_pos:
                self.line(mv, "assign", uid, Marker.LINE_LIST, [p, target_pos[s.target_id]], col)
            else:
                self.delete(mv, "assign", uid)

        ok, _ = self.gz.request("/marker_array", mv, Marker_V, Boolean, 200)
        if not ok:
            # GUI kapalı ya da yeniden açılıyor: açıldığında kamerayı tekrar ayarla.
            self.camera_set = False

    def move_camera(self):
        """Kamerayı dronların doğduğu köşenin arkasına, alanın ortasına bakacak
        şekilde konumla. (Fareyle istediğin gibi değiştirebilirsin.)"""
        # PX4 açılışta kamerayı bir drone'a kilitler (FOLLOW); serbest bırak.
        self.track_pub.publish(CameraTrack(track_mode=CameraTrack.NONE))
        cam = GUICamera()
        cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = -35.0, -35.0, 45.0
        pitch, yaw = 0.42, math.pi / 4                 # merkeze doğru, hafif aşağı bakış
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cam.pose.orientation.w = cy * cp
        cam.pose.orientation.x = -sy * sp
        cam.pose.orientation.y = cy * sp
        cam.pose.orientation.z = sy * cp
        ok, _ = self.gz.request("/gui/move_to/pose", cam, GUICamera, Boolean, 500)
        return ok


def main():
    rclpy.init()
    node = GzVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
