#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
unit_src="$repo_root/systemd/navigator.service"
unit_dst=/etc/systemd/system/navigator.service

mkdir -p "$repo_root/logs"
sudo install -m 644 "$unit_src" "$unit_dst"
sudo systemctl daemon-reload
sudo systemctl enable navigator.service

echo "navigator.service enabled (starts on boot)."
echo "  start now:  pixi run nav-service-start"
echo "  stop:       pixi run nav-service-stop"
echo "  status:     pixi run nav-service-status"
echo "  logs:       pixi run nav-service-logs"
