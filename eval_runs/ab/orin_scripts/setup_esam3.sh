#!/usr/bin/env bash
# EfficientSAM3 environment on the Orin: cuSPARSELt + venv + torch-jetson + deps.
set -e
PW=jetson

echo "=== [1/4] cuSPARSELt (system dep for torch 2.8 jetson wheels) ==="
if ! ldconfig -p | grep -q libcusparseLt; then
  cd /tmp
  wget -q https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb
  echo "$PW" | sudo -S dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb
  echo "$PW" | sudo -S cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/
  echo "$PW" | sudo -S apt-get update 2>&1 | tail -2
  echo "$PW" | sudo -S apt-get -y install cusparselt-cuda-12 2>&1 | tail -3
  rm -f cusparselt-local-tegra-repo-*.deb
else
  echo "cusparselt already present"
fi

echo "=== [2/4] venv ==="
python3 -m venv --system-site-packages ~/venvs/esam3
source ~/venvs/esam3/bin/activate
pip install -q --upgrade pip

echo "=== [3/4] torch + torchvision (jetson-ai-lab jp6/cu126) ==="
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 2>&1 | tail -5

echo "=== [4/4] EfficientSAM3 python deps ==="
pip install "numpy>=1.26,<2" timm "ftfy==6.1.1" regex iopath typing_extensions huggingface_hub pillow tqdm 2>&1 | tail -3

echo "=== verify ==="
python - <<PY
import torch, torchvision, numpy
print("torch", torch.__version__)
print("torchvision", torchvision.__version__)
print("numpy(venv)", numpy.__version__)
print("cuda_available", torch.cuda.is_available())
print("device", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")
PY
echo "SETUP_ESAM3_DONE"
