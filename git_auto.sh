#!/bin/bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRANCH="${BRANCH:-$(git -C "${REPO_DIR}" branch --show-current)}"
POLL_INTERVAL_S="${POLL_INTERVAL_S:-5}"
DEBOUNCE_S="${DEBOUNCE_S:-3}"
AUTO_PUSH="${AUTO_PUSH:-0}"
COMMIT_PREFIX="${COMMIT_PREFIX:-auto: update}"
SERVICE_NAME="${SERVICE_NAME:-turtlebot3-platooning-git-auto.service}"
SERVICE_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/systemd/user"
SERVICE_FILE="${SERVICE_DIR}/${SERVICE_NAME}"

if [ -z "${BRANCH}" ]; then
  echo "Detached HEAD state; aborting auto commit watcher."
  exit 1
fi

cd "${REPO_DIR}"

print_usage() {
  cat <<EOF
Usage: $0 [watch|once|install|uninstall|status]

Commands:
  watch      Watch this repository and auto-commit changes. This is the default.
  once       Commit current pending changes once, then exit.
  install    Install and start a user systemd service for automatic watching.
  uninstall  Stop and remove the user systemd service.
  status     Show the user systemd service status.

Environment:
  AUTO_PUSH=1          Push commits to origin/${BRANCH} after each auto commit.
  POLL_INTERVAL_S=5    Poll interval when inotifywait is unavailable.
  DEBOUNCE_S=3         Delay after a filesystem event before committing.
EOF
}

install_user_service() {
  mkdir -p "${SERVICE_DIR}"
  cat > "${SERVICE_FILE}" <<EOF
[Unit]
Description=TurtleBot3 Platooning git auto commit watcher
After=network-online.target

[Service]
Type=simple
WorkingDirectory=${REPO_DIR}
Environment=BRANCH=${BRANCH}
Environment=AUTO_PUSH=${AUTO_PUSH}
Environment=POLL_INTERVAL_S=${POLL_INTERVAL_S}
Environment=DEBOUNCE_S=${DEBOUNCE_S}
ExecStart=${REPO_DIR}/git_auto.sh watch
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
EOF

  systemctl --user daemon-reload
  systemctl --user enable --now "${SERVICE_NAME}"
  echo "Installed and started user service: ${SERVICE_NAME}"
  echo "Check status with: systemctl --user status ${SERVICE_NAME}"
}

uninstall_user_service() {
  systemctl --user disable --now "${SERVICE_NAME}" 2>/dev/null || true
  rm -f "${SERVICE_FILE}"
  systemctl --user daemon-reload
  echo "Removed user service: ${SERVICE_NAME}"
}

commit_pending_changes() {
  if git diff --quiet && git diff --cached --quiet && [ -z "$(git ls-files --others --exclude-standard)" ]; then
    return 0
  fi

  git add -A

  if git diff --cached --quiet; then
    return 0
  fi

  local now
  now="$(date '+%Y-%m-%d %H:%M:%S')"
  git commit -m "${COMMIT_PREFIX} ${now}"

  if [ "${AUTO_PUSH}" = "1" ]; then
    git push origin "${BRANCH}"
  fi
}

watch_with_inotify() {
  inotifywait -m -r \
    --exclude '(^|/)(\.git|build|install|log|__pycache__|\.pytest_cache|\.colcon)(/|$)|(~$)|(\.swp$)|(\.tmp$)' \
    -e close_write,create,delete,move,attrib \
    "${REPO_DIR}" |
  while read -r _path _event _file; do
    sleep "${DEBOUNCE_S}"
    commit_pending_changes
  done
}

watch_with_polling() {
  while true; do
    sleep "${POLL_INTERVAL_S}"
    commit_pending_changes
  done
}

ACTION="${1:-watch}"
case "${ACTION}" in
  -h|--help|help)
    print_usage
    exit 0
    ;;
  once)
    commit_pending_changes
    exit 0
    ;;
  install)
    install_user_service
    exit 0
    ;;
  uninstall)
    uninstall_user_service
    exit 0
    ;;
  status)
    systemctl --user status "${SERVICE_NAME}"
    exit 0
    ;;
  watch)
    ;;
  *)
    echo "Unknown command: ${ACTION}"
    print_usage
    exit 2
    ;;
esac

echo "=== Git Auto Commit Watcher Start ==="
echo "repo=${REPO_DIR}"
echo "branch=${BRANCH}"
echo "auto_push=${AUTO_PUSH}"

commit_pending_changes

if command -v inotifywait >/dev/null 2>&1; then
  watch_with_inotify
else
  echo "inotifywait not found; falling back to polling every ${POLL_INTERVAL_S}s."
  watch_with_polling
fi
