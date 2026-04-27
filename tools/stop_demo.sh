#!/usr/bin/env bash
# stop_demo.sh — kill every Plato Pod ROS2 process started by run_demo.sh.

pkill -f "ros2 launch plato_pod" 2>/dev/null || true
pkill -f "ros2 run plato_pod"    2>/dev/null || true
pkill -f world_state_node        2>/dev/null || true
pkill -f cot_bridge_node         2>/dev/null || true
pkill -f arena_model_node        2>/dev/null || true
pkill -f engagement_node         2>/dev/null || true
pkill -f los_python_node         2>/dev/null || true
pkill -f los_gazebo_node         2>/dev/null || true
pkill -f opfor_node              2>/dev/null || true
pkill -f registry_node           2>/dev/null || true
pkill -f virtual_sim_node        2>/dev/null || true
pkill -f api_gateway_node        2>/dev/null || true
sleep 1
echo "Stopped. Active platopod processes:"
ps aux | grep -E "plato_pod|world_state|cot_bridge|arena_model|engagement_node|los_python|opfor_node|registry_node|virtual_sim" | grep -v grep || echo "  (none)"
