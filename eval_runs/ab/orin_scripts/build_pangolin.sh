#!/usr/bin/env bash
set -e
PW=jetson
echo "[0] apt-get update (best effort)"
echo "$PW" | sudo -S apt-get update 2>&1 | tail -3 || true
echo "[1] install prerequisites"
echo "$PW" | sudo -S apt-get install -y \
  libgl1-mesa-dev libglew-dev libegl1-mesa-dev libwayland-dev \
  libxkbcommon-dev wayland-protocols libpython3-dev libepoxy-dev \
  cmake build-essential ninja-build 2>&1 | tail -10
echo "[2] clone Pangolin v0.8"
cd ~
rm -rf Pangolin
git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
echo "[3] cmake configure"
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_PANGOLIN_PYTHON=OFF
echo "[4] build (-j4)"
cmake --build build -j4
echo "[5] install"
echo "$PW" | sudo -S cmake --install build
echo "$PW" | sudo -S ldconfig
echo "PANGOLIN_BUILD_DONE_OK"
