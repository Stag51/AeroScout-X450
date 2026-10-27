# Shared setup, sourced by the other scripts.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_aero/install/setup.bash" ] && source "$HOME/ws_aero/install/setup.bash"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
MODEL=aeroscout_x450_0
