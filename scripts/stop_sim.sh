#!/usr/bin/env bash
# start_sim.sh ile açılan her şeyi kapatır. Gazebo sunucusu PX4 tarafından
# arka planda (&) başlatıldığı için tmux kapanınca kendiliğinden kapanmaz.
tmux kill-session -t wta 2>/dev/null
pkill -f "^gz sim" 2>/dev/null
pkill -x px4 2>/dev/null
pkill -x MicroXRCEAgent 2>/dev/null
echo "Simülasyon kapatıldı."
