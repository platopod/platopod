#!/usr/bin/env bash
# run_demo.sh — bring up the full Plato Pod tactical stack for one scenario.
#
# Usage:
#   ./tools/run_demo.sh <scenario_yaml> <iphone_ip>
#
# Example:
#   ./tools/run_demo.sh \
#     /ros2_ws/config/exercises/cbrn-recon-patrol.yaml \
#     192.168.1.233
#
# Override geo origin / scale_factor / ports via environment:
#   GEO_LAT=-35.293935 GEO_LON=149.166421 SCALE=150 \
#       ./tools/run_demo.sh ...
#
# Stop everything with: ./tools/stop_demo.sh

set -euo pipefail

EX="${1:?scenario YAML path is required}"
IPHONE_IP="${2:?iPhone (or ATAK device) IP is required}"

GEO_LAT="${GEO_LAT:--35.293935}"
GEO_LON="${GEO_LON:-149.166421}"
SCALE="${SCALE:-150}"
COT_PORT="${COT_PORT:-4242}"
INBOUND_PORT="${INBOUND_PORT:-4243}"

echo "→ stopping any prior platopod processes…"
pkill -f "ros2 launch plato_pod" 2>/dev/null || true
pkill -f "ros2 run plato_pod"    2>/dev/null || true
pkill -f world_state_node        2>/dev/null || true
pkill -f cot_bridge_node         2>/dev/null || true
pkill -f arena_model_node        2>/dev/null || true
pkill -f engagement_node         2>/dev/null || true
pkill -f los_python_node         2>/dev/null || true
pkill -f opfor_node              2>/dev/null || true
pkill -f registry_node           2>/dev/null || true
pkill -f virtual_sim_node        2>/dev/null || true
sleep 2

echo "→ starting platform with $EX → $IPHONE_IP:$COT_PORT"

ros2 run plato_pod arena_model_node --ros-args -p exercise_file:="$EX" &
ros2 run plato_pod registry_node &
ros2 launch plato_pod world_state.launch.py exercise_file:="$EX" &
ros2 run plato_pod virtual_sim_node &
sleep 2

ros2 launch plato_pod los.launch.py backend:=python &
ros2 launch plato_pod engagement.launch.py exercise_file:="$EX" &
sleep 1

ros2 launch plato_pod cot_bridge.launch.py \
    exercise_file:="$EX" \
    transport:=udp_unicast \
    target_host:="$IPHONE_IP" \
    target_port:="$COT_PORT" \
    geo_origin_lat:="$GEO_LAT" \
    geo_origin_lon:="$GEO_LON" \
    scale_factor:="$SCALE" \
    inbound_port:="$INBOUND_PORT" &

sleep 5
echo "→ running nodes:"
ros2 node list
echo
echo "→ tail logs with:  ros2 topic echo /engagement_events  (or any topic)"
echo "→ stop with:       ./tools/stop_demo.sh"
echo
echo "Now spawn robots, fire weapons, watch iTAK."
wait
