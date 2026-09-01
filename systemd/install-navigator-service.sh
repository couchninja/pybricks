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
echo "  start now:  pixi run navigator-start"
echo "  stop:       pixi run navigator-stop"
echo "  status:     pixi run navigator-status"
echo "  logs:       pixi run navigator-logs"
