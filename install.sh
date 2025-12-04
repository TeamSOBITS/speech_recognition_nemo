#!/bin/bash
echo "╔══╣ Install: speech_recognition_nemo (STARTING) ╠══╗"

SCRIPT_DIR=$(pwd)

set -e

echo "--- Updating apt package lists and installing system dependencies ---"
export DEBIAN_FRONTEND=noninteractive

sudo apt update -y

sudo apt install pulseaudio-utils -y

sudo apt install ffmpeg -y

echo "System dependencies installed."

echo "--- Installing Python packages via pip3 ---"

pip3 install typing_extensions
pip3 install nemo_toolkit[asr]
echo "Finish to install NeMo"

echo "--- Cloning ROS packages ---"
cd ..

SOBITS_MSGS_REPO="sobits_interfaces"
if [ ! -d "$SOBITS_MSGS_REPO" ]; then
    echo "Cloning $SOBITS_MSGS_REPO repository..."
    git clone -b humble-devel https://github.com/TeamSOBITS/sobits_interfaces.git
    echo "$SOBITS_MSGS_REPO cloned successfully."
else
    echo "$SOBITS_MSGS_REPO repository already exists. Skipping clone."
fi

cd "$SCRIPT_DIR" || { echo "Error: Could not return to $SCRIPT_DIR"; exit 1; }
pip3 uninstall setuptools -y
pip3 install setuptools==65.5.1

echo "--- Installing VAD ---"
pip3 install -U --force-reinstall -v git+https://github.com/TEN-framework/ten-vad.git
sudo apt install libc++1 -y

echo "--- Install numba ---"
pip3 install --force-reinstall numba==0.61.2

echo "--- Install coverage ---"
pip3 install --force-reinstall coverage==6.2

pip3 install --force-reinstall numpy== 1.24.4

echo "--- Downloading NeMo ASR models ---"
cd "$SCRIPT_DIR/speech_recognition_nemo" || { echo "Error: Could not change to $SCRIPT_DIR"; exit 1; }
python3 model_download.py
echo "NeMo ASR models downloaded."

echo "╚══╣ Install: speech_recognition_nemo (FINISHED) ╠══╝"
