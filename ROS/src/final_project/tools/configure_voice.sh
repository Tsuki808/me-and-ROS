#!/usr/bin/env bash
# Optional mode 3 dependencies; never changes the ROS Python interpreter.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$project_dir/../.." && pwd)"
sudo -n apt-get install -y python3.8 python3.8-venv alsa-utils
python3.8 -m venv "$workspace_dir/.venv-voice"
"$workspace_dir/.venv-voice/bin/python" -m pip install --upgrade 'pip<25.1'
"$workspace_dir/.venv-voice/bin/python" -m pip install -r "$project_dir/voice/requirements.txt"
echo 'Voice dependencies ready. Configure ~/.config/final_project/xfyun.env with chmod 600.'
