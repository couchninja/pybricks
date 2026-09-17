#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
unit_src="$repo_root/systemd/navigator.service"
unit_dst=/etc/systemd/system/navigator.service
logrotate_src="$repo_root/systemd/navigator.logrotate"
logrotate_dst=/etc/logrotate.d/navigator

mkdir -p "$repo_root/logs"
sudo install -m 644 "$unit_src" "$unit_dst"
sed "s|@REPO_ROOT@|$repo_root|g" "$logrotate_src" | sudo tee "$logrotate_dst" > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable navigator.service

echo "navigator.service enabled (starts on boot)."
echo "  logrotate:  $logrotate_dst (10M, keep 7, copytruncate)"
echo "  start now:  pixi run nav-service-start"
echo "  stop:       pixi run nav-service-stop"
echo "  status:     pixi run nav-service-status"
echo "  logs:       pixi run nav-service-logs"
