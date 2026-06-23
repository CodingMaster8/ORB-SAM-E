#!/usr/bin/env bash
# Rebuild the pod environment after a RunPod STOP (which wipes the container
# disk). Everything under /workspace persists; this script reinstalls the
# system + Python environment that lives on the ephemeral container disk.
#
# Usage (on the pod, after Start):
#   bash /workspace/resume_setup.sh
#
# Safe to re-run: each step checks before doing expensive work.

set -uo pipefail

export WORK=/workspace
export LANG=en_US.UTF-8
REPO="$WORK/ros2_ws/src/ORB-SAM-E"

echo "==> [1/8] Persistent paths + weights symlink"
grep -q "export WORK=$WORK" ~/.bashrc 2>/dev/null || echo "export WORK=$WORK" >> ~/.bashrc
# Recreate the /root/weights -> /workspace/weights symlink (symlink lived on container disk)
if [ ! -e /root/weights ] && [ -d "$WORK/weights" ]; then
    ln -s "$WORK/weights" /root/weights
    echo "    linked /root/weights -> $WORK/weights"
fi

echo "==> [2/8] Base apt packages"
apt-get update -y
apt-get install -y curl wget git vim build-essential cmake pkg-config \
    ninja-build software-properties-common locales sudo
locale-gen en_US en_US.UTF-8 >/dev/null 2>&1
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 >/dev/null 2>&1

echo "==> [3/8] ROS 2 Jazzy (skip if already present)"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt-get update -y
    apt-get install -y ros-jazzy-desktop ros-dev-tools \
        ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-opencv \
        ros-jazzy-rqt-image-view ros-jazzy-sensor-msgs ros-jazzy-std-msgs
else
    echo "    /opt/ros/jazzy already present"
fi
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc 2>/dev/null || \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo "==> [4/8] System libraries (OpenCV, Eigen, Boost, GL, xvfb)"
apt-get install -y \
    libeigen3-dev libopencv-dev python3-opencv \
    libboost-all-dev libssl-dev \
    libglew-dev libglfw3-dev libgtk-3-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    python3-pip python3-dev python3-numpy python3-venv \
    xvfb mesa-utils libgl1-mesa-dri

echo "==> [5/8] Pangolin (reinstall from persisted build, else rebuild)"
if ! ldconfig -p | grep -q libpango; then
    if [ -d "$WORK/Pangolin/build" ]; then
        cmake --install "$WORK/Pangolin/build"
    else
        cd "$WORK" && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
        cd "$WORK/Pangolin"
        yes | ./scripts/install_prerequisites.sh recommended
        cmake -B build && cmake --build build -j"$(nproc)" && cmake --install build
    fi
    ldconfig
else
    echo "    Pangolin already installed"
fi
grep -q "LD_LIBRARY_PATH=/usr/local/lib" ~/.bashrc 2>/dev/null || \
    echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

echo "==> [6/8] Python packages (pip lives on container disk -> reinstall)"
pip3 install -e "$REPO/efficientsam3_arm"
pip3 install pillow natsort matplotlib scipy einops
[ -f "$REPO/metrics/requirements.txt" ] && pip3 install -r "$REPO/metrics/requirements.txt"

echo "==> [7/8] Source ROS + workspace (colcon install/ persisted on /workspace)"
source /opt/ros/jazzy/setup.bash
if [ -f "$WORK/ros2_ws/install/setup.bash" ]; then
    source "$WORK/ros2_ws/install/setup.bash"
    grep -q "source $WORK/ros2_ws/install/setup.bash" ~/.bashrc 2>/dev/null || \
        echo "source $WORK/ros2_ws/install/setup.bash" >> ~/.bashrc
else
    echo "    NOTE: no colcon install found; rebuild with:"
    echo "    cd $WORK/ros2_ws && colcon build --symlink-install"
fi

echo "==> [8/8] Sanity checks"
python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
echo -n "ROS_DISTRO="; echo "${ROS_DISTRO:-<not sourced>}"
ros2 pkg list 2>/dev/null | grep -E "efficientsam3_ros2|ros2_orb_slam3" || \
    echo "    (workspace packages not found - may need colcon build)"
ls -lh "$WORK/weights/" 2>/dev/null

echo ""
echo "Resume complete. Open a new shell (or 'source ~/.bashrc') so ROS is sourced."
