#!/usr/bin/env bash
#
# One-shot Linux dev + simulation setup for the autonomous VTOL brain.
#
# Target: Ubuntu 24.04 (works on both WSL2/x86_64 on Windows and an ARM64
# Ubuntu VM on Apple-Silicon macOS -- apt resolves the right architecture).
#
# It installs, idempotently:
#   1. build tools + Python 3.12 venv for this repo
#   2. ArduPilot source, built for SITL with the ArduPlane (QuadPlane) vehicle
#   3. Gazebo Harmonic + the ardupilot_gazebo plugin (camera-in-the-loop sim)
#
# Re-running is safe: each step checks whether it is already done.
#
# Usage:
#   ./scripts/setup_linux.sh                 # everything
#   ./scripts/setup_linux.sh --no-gazebo     # skip the (large) Gazebo step
#   SKIP_ARDUPILOT=1 ./scripts/setup_linux.sh
#
set -euo pipefail

# --- config -----------------------------------------------------------------
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_BRANCH="${ARDUPILOT_BRANCH:-Plane-4.5}"   # QuadPlane lives in ArduPlane; newest stable Plane branch
GZ_PLUGIN_DIR="${GZ_PLUGIN_DIR:-$HOME/ardupilot_gazebo}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WANT_GAZEBO=1
[[ "${1:-}" == "--no-gazebo" ]] && WANT_GAZEBO=0

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*"; }

# --- sanity -----------------------------------------------------------------
if ! grep -qi ubuntu /etc/os-release 2>/dev/null; then
  warn "This script targets Ubuntu. Detected: $(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-unknown}")."
  warn "Continuing, but package names may differ."
fi
ARCH="$(uname -m)"
say "Host arch: $ARCH   (x86_64 = WSL2/Windows, aarch64/arm64 = Mac VM)"

# --- 1. base tooling --------------------------------------------------------
say "Installing base build tools + Python"
sudo apt-get update -y
sudo apt-get install -y \
  git build-essential ccache \
  python3 python3-venv python3-pip python3-dev \
  rapidjson-dev libtool automake autoconf \
  libxml2-dev libxslt1-dev

# --- 2. this repo's Python venv ---------------------------------------------
say "Creating this repo's Python venv at $REPO_DIR/.venv"
if [[ ! -d "$REPO_DIR/.venv" ]]; then
  python3 -m venv "$REPO_DIR/.venv"
fi
# shellcheck disable=SC1091
source "$REPO_DIR/.venv/bin/activate"
pip install --upgrade pip
pip install -r "$REPO_DIR/requirements.txt"
say "Repo Python deps installed. (Perception libs -- YOLO, tracker -- get added as we build those modules.)"

# --- 3. ArduPilot (SITL + QuadPlane) ---------------------------------------
if [[ "${SKIP_ARDUPILOT:-0}" != "1" ]]; then
  if [[ ! -d "$ARDUPILOT_DIR" ]]; then
    say "Cloning ArduPilot ($ARDUPILOT_BRANCH) into $ARDUPILOT_DIR"
    git clone --recurse-submodules -b "$ARDUPILOT_BRANCH" \
      https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  else
    say "ArduPilot already present at $ARDUPILOT_DIR (skipping clone)"
  fi

  cd "$ARDUPILOT_DIR"
  git submodule update --init --recursive

  say "Installing ArduPilot's prerequisites (this pulls MAVProxy, empy, etc.)"
  # ArduPilot ships its own dependency installer; USER can be unset in WSL.
  USER="${USER:-$(whoami)}" Tools/environment_install/install-prereqs-ubuntu.sh -y || \
    warn "install-prereqs returned nonzero; often fine if deps already present."

  # shellcheck disable=SC1090
  source "$HOME/.profile" || true

  say "Building ArduPlane SITL (the QuadPlane vehicle)"
  ./waf configure --board sitl
  ./waf plane
  say "ArduPilot SITL built. Launch later with: sim_vehicle.py -v ArduPlane -f quadplane --console --map"
else
  say "SKIP_ARDUPILOT=1 -- skipping ArduPilot build"
fi

# --- 4. Gazebo Harmonic + ardupilot_gazebo ----------------------------------
if [[ "$WANT_GAZEBO" == "1" ]]; then
  say "Installing Gazebo Harmonic"
  sudo apt-get install -y curl lsb-release gnupg
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update -y
  sudo apt-get install -y gz-harmonic

  say "Building the ardupilot_gazebo plugin"
  sudo apt-get install -y libgz-sim8-dev rapidjson-dev cmake
  if [[ ! -d "$GZ_PLUGIN_DIR" ]]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$GZ_PLUGIN_DIR"
  fi
  cd "$GZ_PLUGIN_DIR"
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j"$(nproc)"

  say "Gazebo plugin built. Add these to your ~/.bashrc (or run per-shell):"
  cat <<EOF

  export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_PLUGIN_DIR/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
  export GZ_SIM_RESOURCE_PATH=$GZ_PLUGIN_DIR/models:$GZ_PLUGIN_DIR/worlds:\${GZ_SIM_RESOURCE_PATH:-}

EOF
  if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    warn "On the Apple-Silicon Mac VM, Gazebo's 3D view needs GPU passthrough or"
    warn "software rendering (LIBGL_ALWAYS_SOFTWARE=1). Headless sensor sim works either way."
  fi
else
  say "--no-gazebo -- skipping Gazebo (you can rerun without the flag later)"
fi

say "Done. Next: docs/dev-environment.md walks through the first simulated flight."
