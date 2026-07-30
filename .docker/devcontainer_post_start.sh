#!/usr/bin/env bash
# .docker/devcontainer_post_start.sh
#
# Runs every time the devcontainer starts.
# Distro-agnostic — reads $ROS_DISTRO set by the base image.

DISTRO="${ROS_DISTRO:?ROS_DISTRO is not set — this script expects the ROS base image environment}"

source "/opt/ros/${DISTRO}/setup.bash"
source /ros2_ws/install/setup.bash 2>/dev/null || true

# ── Convenience aliases ───────────────────────────────────────
alias cb='cd /ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash'
alias cbs='cd /ros2_ws && colcon build --symlink-install --packages-select'
alias ct='cd /ros2_ws && colcon test --packages-select'
alias rolling_launch='ros2 launch rolling_bonxai_ros rolling_bonxai_mapping.launch.py'
alias rolling_launch_sim='ros2 launch rolling_bonxai_ros rolling_bonxai_mapping.launch.py use_sim_time:=true'
alias rolling_stats='ros2 topic echo /rolling_bonxai/quick_occupancy_stats'
alias rolling_chunks='ros2 topic echo /rolling_bonxai/chunk_stats'
alias rolling_state='ros2 topic echo /rolling_bonxai/transition_state'
alias px4_topics='ros2 topic list | grep fmu'
alias clean_map='ros2 service call /rolling_bonxai/clean_memory std_srvs/srv/Trigger'

echo "╔═══════════════════════════════════════════════════════╗"
printf "║  Rolling Bonxai devcontainer — ROS 2 %-17s║\n" "${DISTRO^}"
echo "╠═══════════════════════════════════════════════════════╣"
echo "║  cb             full rebuild                          ║"
echo "║  cbs <pkg>      rebuild one package                   ║"
echo "║  ct  <pkg>      run tests for one package             ║"
echo "║  rolling_launch launch RollingBonxaiServer            ║"
echo "║  rolling_launch_sim  same, with use_sim_time:=true    ║"
echo "║  rolling_stats  echo occupancy stats                  ║"
echo "║  rolling_chunks echo chunk stats                      ║"
echo "║  rolling_state  echo transition state                 ║"
echo "║  px4_topics     list /fmu/* topics                    ║"
echo "║  clean_map      call clean_memory service             ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo ""
echo "  Distro    : ${DISTRO}"
echo "  Workspace : /ros2_ws"
echo "  Source    : /ros2_ws/src/Rolling-Bonxai"
echo ""
