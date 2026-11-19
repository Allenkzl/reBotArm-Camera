#!/usr/bin/env bash
# Prepare a dedicated, clean upstream source checkout. Does not build or start anything.
set -euo pipefail
package=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
overlays="$package/deployment/recomputer"
upstream_commit=455d008e82530b1b6f645e2427f89881c2aa071e
if [[ $# != 1 || ! -d "$1" ]]; then
  echo "Usage: $0 PATH_TO_CLEAN_UPSTREAM_CHECKOUT" >&2
  exit 2
fi
target=$(cd "$1" && pwd)
if [[ $(git -C "$target" rev-parse HEAD) != "$upstream_commit" ||
      -n $(git -C "$target" status --porcelain) ]]; then
  echo "Use a clean checkout at $upstream_commit." >&2
  exit 2
fi
git -C "$target" apply --check "$overlays/upstream.patch"
git -C "$target" apply "$overlays/upstream.patch"
destination="$target/deployment/recomputer"
mkdir -p "$destination/models" "$target/gravity_console"
for file in hardware-camera-j1-j6.yaml sdk-camera-j1-j6.yaml; do
  python3 - "$overlays/$file" "$destination/$file" "$target" <<'PY'
import sys
from pathlib import Path

source, output, target = sys.argv[1:]
text = Path(source).read_text()
base = "/home/com/ReBot_Arm_DigitalTwin_RS/deployment/recomputer/"
Path(output).write_text(text.replace(base, f"{target}/deployment/recomputer/"))
PY
done
cp "$package/config/recomputer_home.json" "$destination/home.json"
cp "$overlays/models/ReBot_Arm_RS_Camera_950g.urdf" "$destination/models/"
ln -s ../../rebotarm_ros2_RS/third_party/reBotArm_control_py/urdf/RS/meshes "$destination/meshes"
install -m 755 "$overlays"/scripts/*.sh "$target/scripts/"
cp "$overlays"/controller-tests/*.py "$target/rebotarm_ros2_RS/src/rebotarmcontroller/test/"
mkdir -p "$target/camera_workbench/static"
cp "$package"/camera_workbench/*.py "$target/camera_workbench/"
cp "$package"/camera_workbench/static/{app.js,i18n.js,index.html,styles.css} "$target/camera_workbench/static/"
touch "$target/gravity_console/__init__.py"
cp "$package"/gravity_console/{camera_bridge,shooting_groups}.py "$target/gravity_console/"
printf '%s\n' "Source overlay installed: $target"
