#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
unit_src="$repo_root/systemd/navigator.service"
unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_dst="$unit_dir/navigator.service"
logrotate_src="$repo_root/systemd/navigator.logrotate"
logrotate_dst=/etc/logrotate.d/navigator
pixi_bin="${PIXI_BIN:-$HOME/.pixi/bin}"

mkdir -p "$repo_root/logs" "$unit_dir"
log_file="$repo_root/logs/navigator.log"
if [[ -f "$log_file" ]] && [[ ! -w "$log_file" ]]; then
  echo "Fixing log file ownership (sudo)..."
  sudo chown "$(whoami):$(whoami)" "$log_file"
fi
sed -e "s|@REPO_ROOT@|$repo_root|g" -e "s|@PIXI_BIN@|$pixi_bin|g" \
  "$unit_src" > "$unit_dst"

systemctl --user daemon-reload
systemctl --user enable navigator.service

user="$(whoami)"
if ! loginctl show-user "$user" -p Linger --value 2>/dev/null | grep -qx yes; then
  echo "Enabling linger for $user (sudo, user services start at boot)..."
  sudo loginctl enable-linger "$user"
fi

if command -v logrotate >/dev/null 2>&1; then
  echo "Installing logrotate config (sudo)..."
  sed "s|@REPO_ROOT@|$repo_root|g" "$logrotate_src" | sudo tee "$logrotate_dst" > /dev/null
else
  echo "logrotate not found; skipping $logrotate_dst"
fi

echo "navigator.service enabled for $user (user systemd, starts at boot)."
echo "  unit:       $unit_dst"
echo "  linger:     enabled"
if [[ -f "$logrotate_dst" ]]; then
  echo "  logrotate:  $logrotate_dst (10M, keep 7, copytruncate)"
fi
echo "  start now:  pixi run nav-service-start"
echo "  stop:       pixi run nav-service-stop"
echo "  status:     pixi run nav-service-status"
echo "  logs:       pixi run nav-service-logs"
