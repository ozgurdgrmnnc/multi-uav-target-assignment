#!/usr/bin/env bash
# Dağıtık çoklu İHA simülasyonunu tmux içinde başlatır.
#
#   ~/ws_px4/scripts/start_sim.sh [İHA_SAYISI] [HEDEF_SAYISI]
#
# Pencereler (Ctrl+b n / Ctrl+b p ile gezin, Ctrl+b d ile ayrıl):
#   agent   : MicroXRCEAgent (PX4 <-> ROS 2 köprüsü)
#   px4_i   : her İHA için ayrı PX4 SITL instance'ı
#   ros     : world_node + gz_viz_node + uav_agent_node x N
# Kapatmak için: ~/ws_px4/scripts/stop_sim.sh
#
# Seçenekler (ortam değişkeni): HEADLESS=1 -> Gazebo GUI açılmaz,
#                                NO_ATTACH=1 -> tmux'a bağlanmadan çık
set -e

N=${1:-3}
T=${2:-3}
PX4_DIR=~/PX4-Autopilot
WS=~/ws_px4
SESSION=wta

if tmux has-session -t $SESSION 2>/dev/null; then
    echo "'$SESSION' tmux oturumu zaten açık. Önce kapat: ~/ws_px4/scripts/stop_sim.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

# İHA doğuş noktaları tek kaynaktan: iha_wta.core.params.UAV_STARTS
spawn_xy() {
    python3 -c "from iha_wta.core.params import uav_start; x,y,_=uav_start($1); print(f'{x},{y}')"
}

# Her instance için ortak PX4 ortamı:
#   PX4_UXRCE_DDS_NS : topic'ler /uav_i/fmu/... altında olsun (instance 0 dahil)
#   PX4_PARAM_*      : açılışta parametre ata (GCS ve RC şartı kapalı)
#   PX4_GZ_NO_FOLLOW : PX4 Gazebo kamerasını drone takibine kilitlemesin
px4_cmd() {
    local i=$1 pose=$2 extra=$3
    echo "cd $PX4_DIR && $extra PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
PX4_GZ_MODEL_POSE=$pose PX4_UXRCE_DDS_NS=uav_$i \
PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_COM_RC_IN_MODE=4 PX4_GZ_NO_FOLLOW=1 \
./build/px4_sitl_default/bin/px4 -i $i; bash"
}

tmux new-session -d -s $SESSION -n agent "MicroXRCEAgent udp4 -p 8888; bash"

# Instance 0 Gazebo'yu (sunucu + GUI) başlatır.
tmux new-window -t $SESSION -n px4_0 "$(px4_cmd 0 "$(spawn_xy 0)" "PX4_GZ_WORLD=default HEADLESS=${HEADLESS:-}")"

# Diğerleri çalışan Gazebo'ya bağlanır (STANDALONE).
for ((i = 1; i < N; i++)); do
    tmux new-window -t $SESSION -n px4_$i \
        "sleep $((8 + 3 * i)); $(px4_cmd $i "$(spawn_xy $i)" "PX4_GZ_STANDALONE=1")"
done

tmux new-window -t $SESSION -n ros \
    "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && sleep 10 && \
ros2 launch iha_wta wta.launch.py num_uavs:=$N num_targets:=$T; bash"

tmux select-window -t $SESSION:ros
if [ -z "${NO_ATTACH:-}" ]; then
    tmux attach -t $SESSION
else
    echo "Başlatıldı. Bağlanmak için: tmux attach -t $SESSION"
fi
