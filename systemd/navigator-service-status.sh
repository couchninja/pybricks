#!/usr/bin/env bash
set -uo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
log_file="$repo_root/logs/navigator.log"
logrotate_conf=/etc/logrotate.d/navigator

systemctl status navigator --no-pager || true

echo
echo "--- logrotate ---"
if [[ ! -f "$logrotate_conf" ]]; then
  echo "Not installed ($logrotate_conf missing)."
  echo "Install with: pixi run nav-service-autostart"
  exit 0
fi

echo "Config: $logrotate_conf"
cat "$logrotate_conf"
echo

state_file=/var/lib/logrotate/status
if entry=$(grep -F "$log_file" "$state_file" 2>/dev/null); then
  echo "State: $entry"
else
  echo "State: no entry for $log_file (not rotated yet, or unreadable $state_file)"
fi

echo
echo "Log files:"
ls -lh "$repo_root/logs"/navigator.log* 2>/dev/null || echo "  (none)"
