#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /home/user/workspace/install/setup.bash ]; then
  source /home/user/workspace/install/setup.bash
fi

UNITREE_MUJOCO_DIR="${UNITREE_MUJOCO_DIR:-/home/user/workspace/third_party/unitree_mujoco}"
UNITREE_RL_LAB_DIR="${UNITREE_RL_LAB_DIR:-/home/user/workspace/third_party/unitree_rl_lab}"
MUJOCO_HOME="${MUJOCO_HOME:-/opt/mujoco/mujoco-3.3.6}"

if [ -d "${UNITREE_MUJOCO_DIR}/simulate" ] && [ -d "${MUJOCO_HOME}" ]; then
  ln -sfn "${MUJOCO_HOME}" "${UNITREE_MUJOCO_DIR}/simulate/mujoco"

  if [ "${G1_SIM_AUTOBUILD:-1}" = "1" ] && [ ! -x "${UNITREE_MUJOCO_DIR}/simulate/build/unitree_mujoco" ]; then
    cmake -S "${UNITREE_MUJOCO_DIR}/simulate" -B "${UNITREE_MUJOCO_DIR}/simulate/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${UNITREE_MUJOCO_DIR}/simulate/build" -j"$(nproc)"
  fi
fi

G1_RL_CONTROLLER_DIR="${UNITREE_RL_LAB_DIR}/deploy/robots/g1_29dof"
if [ -d "${G1_RL_CONTROLLER_DIR}" ] && [ "${G1_SIM_AUTOBUILD_RL_CONTROLLER:-1}" = "1" ] && [ ! -x "${G1_RL_CONTROLLER_DIR}/build/g1_ctrl" ]; then
  export CPLUS_INCLUDE_PATH="/opt/unitree_robotics/include:/opt/unitree_robotics/include/ddscxx:/opt/unitree_robotics/include/iceoryx/v2.0.2:/usr/local/include:/usr/local/include/ddscxx:/usr/local/include/iceoryx/v2.0.2:${CPLUS_INCLUDE_PATH:-}"
  export LIBRARY_PATH="/opt/unitree_robotics/lib:/usr/local/lib:${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
  cmake -S "${G1_RL_CONTROLLER_DIR}" -B "${G1_RL_CONTROLLER_DIR}/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${G1_RL_CONTROLLER_DIR}/build" -j"$(nproc)"
fi

exec "$@"
