#!/bin/bash
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/KweonTJ/Turtlebot3_Platooning.git}"
BRANCH="${BRANCH:-main}"
PUSH_DISABLED_URL="${PUSH_DISABLED_URL:-DISABLED_ON_TURTLEBOT_PULL_ONLY}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(basename "${SCRIPT_DIR}")" = "src" ]; then
  WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"
  SRC_DIR="${SCRIPT_DIR}"
else
  WORKSPACE_DIR="${SCRIPT_DIR}"
  SRC_DIR="${WORKSPACE_DIR}/src"
fi

source /opt/ros/humble/setup.bash

mkdir -p "${WORKSPACE_DIR}"

if [ ! -d "${SRC_DIR}/.git" ]; then
  if [ -d "${SRC_DIR}" ] && [ "$(find "${SRC_DIR}" -mindepth 1 -maxdepth 1 | wc -l)" -gt 0 ]; then
    BACKUP_DIR="${WORKSPACE_DIR}/src.backup.$(date +%Y%m%d_%H%M%S)"
    echo "Existing non-git src found. Moving it to ${BACKUP_DIR}"
    mv "${SRC_DIR}" "${BACKUP_DIR}"
  fi

  echo "Cloning ${REPO_URL} into ${SRC_DIR}"
  git clone --branch "${BRANCH}" "${REPO_URL}" "${SRC_DIR}"
  cd "${SRC_DIR}"
else
  cd "${SRC_DIR}"
  git -c http.version=HTTP/1.1 -c protocol.version=1 \
    fetch --prune origin "+refs/heads/${BRANCH}:refs/remotes/origin/${BRANCH}"

  git reset --hard "origin/${BRANCH}"
  git clean -fd
fi

git remote set-url --push origin "${PUSH_DISABLED_URL}"
echo "origin push URL disabled; this TurtleBot workspace is pull-only."

cd "${WORKSPACE_DIR}"

colcon build --symlink-install \
  --allow-overriding dynamixel_sdk \
  --packages-skip \
    follower_vision \
    turtlebot3_gazebo \
    turtlebot3_manipulation_gazebo \
    turtlebot3_simulations

source "${WORKSPACE_DIR}/install/setup.bash"

echo "Follower TurtleBot synced from GitHub and rebuilt."
