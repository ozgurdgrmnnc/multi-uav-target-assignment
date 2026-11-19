"""ROS / Gazebo olmadan algoritmayı ayrık zamanda çalıştırır ve sonuç
figürünü üretir (docs/images/offline_sonuc.png).

Kullanım (depo kökünden):
    python3 tools/offline_sim.py --uavs 3 --targets 3 --steps 150

Her İHA kendi UKF'sini çalıştırır ve yalnızca diğer İHA'ların bir önceki
adımda yayınladığı claim'leri görür; ROS'taki uav_agent_node ile aynı karar
mantığıdır (iha_wta.core).
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "iha_wta"))

from iha_wta.core import params as P  # noqa: E402
from iha_wta.core.geometry import emergency_avoidance_direction, point_inside_any_obstacle  # noqa: E402
from iha_wta.core.scenario import get_all_obstacles, noisy_measurement, target_position  # noqa: E402
from iha_wta.core.ukf import UnscentedKalmanFilter3D  # noqa: E402
from iha_wta.core.wta import (ClaimInfo, build_candidate_for_target,  # noqa: E402
                              build_ordered_wta_list, select_candidate_targets, select_target)

# Referans kategorik palet (sabit sıra): İHA'lar slot 1-3, hedefler slot 4-6
UAV_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#e34948"]
TARGET_COLORS = ["#eda100", "#e87ba4", "#6250d6", "#008300", "#e34948", "#2a78d6"]
TEXT, TEXT2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def simulate(num_uavs, num_targets, steps, seed=4):
    rng = np.random.default_rng(seed)
    uavs = np.array([P.uav_start(i) for i in range(num_uavs)], dtype=float)
    ukfs = [{t: UnscentedKalmanFilter3D(dt=P.DT, target_id=t) for t in range(num_targets)}
            for _ in range(num_uavs)]
    active = set(range(num_targets))
    claims = {}
    hist = {"uav": [uavs.copy()], "target": [], "claim": [], "captures": [], "obstacles": None}

    for t in range(steps):
        obstacles = get_all_obstacles(t)
        truth = {tid: target_position(tid, t) for tid in range(num_targets)}
        hidden = {tid: point_inside_any_obstacle(truth[tid], obstacles) for tid in active}

        new_claims, paths = {}, {}
        for uid in range(num_uavs):
            for tid in active:
                ukfs[uid][tid].step(noisy_measurement(truth[tid], rng))
            est = {tid: ukfs[uid][tid].position() for tid in active}
            plan_obs = np.vstack([obstacles] + [uavs[j] for j in range(num_uavs) if j != uid])
            cands = [build_candidate_for_target(uid, tid, uavs[uid], hidden[tid], est[tid],
                                                ukfs[uid][tid], plan_obs)
                     for tid in select_candidate_targets(uavs[uid], active, est)]
            ordered = build_ordered_wta_list(cands, active)
            prev = claims.get(uid)
            others = [c for j, c in claims.items() if j != uid and c.target_id in active]
            sel, status = select_target(uid, ordered, active, prev.target_id if prev else -1, others)
            if sel is None:
                tid = min(active, key=lambda k: np.linalg.norm(est[k] - uavs[uid]))
                paths[uid] = np.vstack([uavs[uid], est[tid]])
                new_claims[uid] = ClaimInfo(uid, -1, 1e18, False)
            else:
                paths[uid] = sel["path"]
                new_claims[uid] = ClaimInfo(uid, sel["target_id"], sel["base_cost"],
                                            status == "SERVICING-CONTINUED")
        claims = new_claims

        new_uavs = uavs.copy()
        for uid in range(num_uavs):
            others = [uavs[j] for j in range(num_uavs) if j != uid]
            avoid, emerg, _, _ = emergency_avoidance_direction(uavs[uid], obstacles, others)
            if emerg:
                new_uavs[uid] = uavs[uid] + avoid * P.UAV_SPEED * P.EMERGENCY_SPEED_MULTIPLIER
            else:
                d = paths[uid][1] - paths[uid][0]
                n = np.linalg.norm(d)
                if n > 1e-9:
                    new_uavs[uid] = uavs[uid] + d / n * min(P.UAV_SPEED, n)
            new_uavs[uid] = P.clip_to_grid(new_uavs[uid])
        uavs = new_uavs

        for tid in sorted(active):
            dists = np.linalg.norm(uavs - truth[tid], axis=1)
            if not hidden[tid] and dists.min() < P.CAPTURE_RADIUS:
                active.discard(tid)
                hist["captures"].append((t, tid, int(dists.argmin()), truth[tid].copy()))

        hist["uav"].append(uavs.copy())
        hist["target"].append(truth)
        hist["claim"].append({uid: c.target_id for uid, c in claims.items()})
        hist["obstacles"] = obstacles
        print(f"\radım {t + 1}/{steps}  aktif hedef: {sorted(active)}   ", end="", flush=True)
        if not active:
            break
    print()
    return hist


def plot(hist, num_uavs, num_targets, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": TEXT2,
                         "xtick.color": TEXT2, "ytick.color": TEXT2, "text.color": TEXT})
    fig, (ax, axt) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [1.15, 1]},
                                  facecolor=SURFACE)
    uav_hist = np.array(hist["uav"])
    n_steps = len(hist["target"])
    capture_step = {tid: t for t, tid, _, _ in hist["captures"]}

    # --- Panel A: üstten görünüş ---
    ax.set_facecolor(SURFACE)
    for o in hist["obstacles"]:
        ax.add_patch(Circle(o[:2], P.SAFETY_RADIUS, facecolor="#d9d8d3", edgecolor="none",
                            alpha=0.45, zorder=0))
    for tid in range(num_targets):
        end = capture_step.get(tid, n_steps - 1) + 1
        pts = np.array([hist["target"][k][tid] for k in range(end)])
        col = TARGET_COLORS[tid % len(TARGET_COLORS)]
        ax.plot(pts[:, 0], pts[:, 1], "--", color=col, lw=2, zorder=2)
        ax.annotate(f"T{tid}", pts[0, :2], xytext=(6, 6), textcoords="offset points",
                    color=TEXT2, fontsize=9)
        ax.plot(*pts[0, :2], "o", ms=8, mfc=SURFACE, mec=col, mew=2, zorder=3)
    for uid in range(num_uavs):
        col = UAV_COLORS[uid % len(UAV_COLORS)]
        ax.plot(uav_hist[:, uid, 0], uav_hist[:, uid, 1], "-", color=col, lw=2,
                label=f"İHA {uid}", zorder=4)
        ax.plot(*uav_hist[0, uid, :2], "s", ms=8, color=col, mec=SURFACE, mew=2, zorder=5)
    # Yakalama etiketleri: her etiket en yakın diğer yakalama noktasının ters
    # yönüne konur, böylece birbirine yakın yakalamalarda yazılar çakışmaz.
    caps = hist["captures"]
    for t, tid, uid, p in caps:
        ax.plot(p[0], p[1], "X", ms=13, color=UAV_COLORS[uid], mec=SURFACE, mew=2, zorder=6)
        others = [c[3][:2] for c in caps if c[1] != tid]
        away = np.array([1.0, -1.0])
        if others:
            near = min(others, key=lambda q: np.linalg.norm(q - p[:2]))
            if np.linalg.norm(p[:2] - near) > 1e-6:
                away = (p[:2] - near) / np.linalg.norm(p[:2] - near)
        off = 16 * away
        if p[1] > P.grid_y - 20:      # üst kenardaki yakalama: etiket içeri (aşağı)
            off[1] = -abs(off[1]) - 6
        ax.annotate(f"T{tid} ← İHA {uid}\nadım {t}", p[:2], xytext=tuple(off),
                    textcoords="offset points", fontsize=8, color=TEXT2,
                    ha="left" if off[0] >= 0 else "right", va="bottom" if off[1] >= 0 else "top",
                    bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec="none", alpha=0.85),
                    zorder=7)
    ax.set_xlim(-5, P.grid_x + 5)
    ax.set_ylim(-5, P.grid_y + 5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (Doğu) [m]")
    ax.set_ylabel("y (Kuzey) [m]")
    ax.grid(color=GRID, lw=0.8)
    ax.set_title("Üstten görünüş: yörüngeler ve yakalama noktaları", loc="left", fontsize=11)
    handles, labels = ax.get_legend_handles_labels()
    from matplotlib.lines import Line2D
    handles += [Line2D([], [], ls="--", color=TEXT2, lw=2),
                Line2D([], [], ls="", marker="X", ms=10, color=TEXT2),
                Line2D([], [], ls="", marker="o", ms=12, mfc="#d9d8d3", mec="none")]
    labels += ["hedef yörüngesi", "yakalama", "engel (son konum)"]
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3,
              fontsize=8, frameon=False)

    # --- Panel B: claim zaman çizelgesi ---
    axt.set_facecolor(SURFACE)
    for uid in range(num_uavs):
        seq = [hist["claim"][k].get(uid, -1) for k in range(n_steps)]
        start = 0
        for k in range(1, n_steps + 1):
            if k == n_steps or seq[k] != seq[start]:
                tid = seq[start]
                if tid >= 0:
                    axt.barh(uid, k - start, left=start, height=0.6,
                             color=TARGET_COLORS[tid % len(TARGET_COLORS)],
                             edgecolor=SURFACE, linewidth=2)
                    if k - start >= 8:
                        axt.text(start + (k - start) / 2, uid, f"T{tid}", ha="center",
                                 va="center", fontsize=9, color=TEXT)
                start = k
    for t, tid, uid, _ in hist["captures"]:
        axt.plot(t, uid, "X", ms=11, color=TEXT, mec=SURFACE, mew=1.5, zorder=3)
    axt.set_yticks(range(num_uavs), [f"İHA {u}" for u in range(num_uavs)])
    axt.invert_yaxis()
    axt.set_xlabel("adım")
    axt.set_xlim(0, n_steps)
    axt.grid(axis="x", color=GRID, lw=0.8)
    axt.set_axisbelow(True)
    for s in ("top", "right", "left"):
        axt.spines[s].set_visible(False)
    axt.set_title("Dağıtık WTA: hangi İHA hangi hedefi claim etti", loc="left", fontsize=11,
                  pad=22)
    axt.text(0, 1.02, "renk = claim edilen hedef, boşluk = claim yok, X = yakalama "
             "(yarıçapa giren her İHA yakalayabilir)", transform=axt.transAxes,
             fontsize=8, color=TEXT2)

    fig.suptitle(f"Çevrimdışı sonuç: {num_uavs} İHA, {num_targets} hedef, "
                 f"{len(hist['captures'])}/{num_targets} hedef yakalandı, {n_steps} adım",
                 x=0.01, ha="left", fontsize=13, fontweight="bold")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"Kaydedildi: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uavs", type=int, default=3)
    ap.add_argument("--targets", type=int, default=3)
    ap.add_argument("--steps", type=int, default=150)
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "images" / "offline_sonuc.png")
    args = ap.parse_args()

    t0 = time.time()
    hist = simulate(args.uavs, args.targets, args.steps, args.seed)
    for t, tid, uid, _ in hist["captures"]:
        print(f"  adım {t:3d}: hedef {tid} <- İHA {uid}")
    print(f"süre: {time.time() - t0:.0f} s")
    plot(hist, args.uavs, args.targets, args.out)


if __name__ == "__main__":
    main()
