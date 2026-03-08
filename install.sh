#!/bin/bash
echo "╔══╣ Install: speech_recognition_nemo (STARTING) ╠══╗"

export PIP_BREAK_SYSTEM_PACKAGES=1

SCRIPT_DIR=$(pwd)
set -e

echo "--- Updating apt package lists and installing system dependencies ---"
export DEBIAN_FRONTEND=noninteractive
sudo apt update -y
sudo apt install pulseaudio-utils ffmpeg libc++1 -y

echo "--- Installing Python packages ---"
pip3 install -U pip setuptools wheel
pip3 install typing_extensions

pip3 install nemo_toolkit[asr]
echo "Finish to install NeMo"

echo "--- Cloning ROS packages ---"
cd ..
SOBITS_MSGS_REPO="sobits_interfaces"
if [ ! -d "$SOBITS_MSGS_REPO" ]; then
    git clone -b ${ROS_DISTRO}-devel https://github.com/TeamSOBITS/sobits_interfaces.git
fi

cd "$SCRIPT_DIR"

echo "--- Installing VAD ---"
pip3 install git+https://github.com/TEN-framework/ten-vad.git

echo "--- Install numba & numpy & coverage ---"
pip3 install --force-reinstall numba==0.61.2
pip3 install --force-reinstall "numpy==1.26.4"
pip3 install --force-reinstall coverage==6.2

echo "--- Downloading NeMo ASR models ---"
if [ -d "$SCRIPT_DIR/speech_recognition_nemo" ]; then
    cd "$SCRIPT_DIR/speech_recognition_nemo"
    python3 model_download.py
    echo "NeMo ASR models downloaded."
else
    echo "Warning: speech_recognition_nemo directory not found."
fi

echo "--- Restoring Build Tools for ROS 2 Jazzy compatibility ---"
pip3 install -U setuptools pip wheel colcon-common-extensions

echo "╚══╣ Install: speech_recognition_nemo (FINISHED) ╠══╝"