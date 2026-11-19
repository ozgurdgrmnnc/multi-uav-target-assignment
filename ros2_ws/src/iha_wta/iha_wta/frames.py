"""Koordinat çerçeveleri ve PX4 topic isimleri.

NED / ENU:
  PX4 (NED):        x = Kuzey, y = Doğu,  z = Aşağı
  ROS/Gazebo (ENU): x = Doğu,  y = Kuzey, z = Yukarı
  Dönüşüm kendi tersidir: (a, b, c) -> (b, a, -c)

Her PX4 instance'ının yerel NED orijini, Gazebo'da doğduğu noktadır.
Algoritmanın kullandığı ortak "dünya" çerçevesi ise Gazebo ENU dünyasıdır:
  dünya_ENU = doğuş_noktası_ENU + ned_to_enu(yerel_NED)
"""
import math

import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]], dtype=float)


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]], dtype=float)


def enu_heading_to_ned_yaw(v_enu):
    """Hareket yönüne bakan PX4 yaw açısı (Kuzey'den saat yönünde)."""
    return math.atan2(v_enu[0], v_enu[1])


def px4_topic(namespace, direction, name, msg_type=None):
    """PX4 uXRCE-DDS topic adı.

    PX4 main, MESSAGE_VERSION > 0 olan mesajların adına `_v<sürüm>` ekler
    (örn. vehicle_local_position_v1). Sürümü mesaj tanımından okuyoruz ki
    px4_msgs güncellenince isimler kendiliğinden doğru kalsın.
    """
    version = getattr(msg_type, "MESSAGE_VERSION", 0) if msg_type is not None else 0
    suffix = f"_v{version}" if version else ""
    ns = f"/{namespace}" if namespace else ""
    return f"{ns}/fmu/{direction}/{name}{suffix}"


# QoS (Quality of Service): yayıncı ve abonenin "teslimat sözleşmesi".
# PX4'ün /fmu/out/* yayıncıları BEST_EFFORT'tur (kayıp olabilir, tekrar
# gönderilmez). Abone RELIABLE isterse QoS uyuşmaz ve HİÇ mesaj gelmez,
# hata da vermez. Abone tarafında VOLATILE seçmek her yayıncıyla uyumludur.
PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# /fmu/in/* için px4_ros_com örneklerindeki profil.
PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
