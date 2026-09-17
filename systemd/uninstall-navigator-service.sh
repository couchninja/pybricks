#!/usr/bin/env bash
set -euo pipefail

unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_dst="$unit_dir/navigator.service"
logrotate_dst=/etc/logrotate.d/navigator

systemctl --user disable --now navigator.service 2>/dev/null || true
rm -f "$unit_dst"
systemctl --user daemon-reload

if [[ -f "$logrotate_dst" ]]; then
  sudo rm -f "$logrotate_dst"
fi

echo "navigator.service disabled (user unit removed)."
