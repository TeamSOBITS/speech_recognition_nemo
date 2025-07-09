#!/bin/bash
echo "╔══╣ Install: speech_recognition_nemo (STARTING) ╠══╗"

# Keep the current directory for later use
SCRIPT_DIR=$(pwd)

# Exit immediately if a command exits with a non-zero status.
set -e

# --- System Package Installation ---
echo "--- Updating apt package lists and installing system dependencies ---"
export DEBIAN_FRONTEND=noninteractive # Skip interactive apt prompts

sudo apt update -y

# Install apt packages one by one, automatically answering 'yes' to prompts
yes | sudo apt install -y ros-humble-vision-msgs
echo "System dependencies installed."

# --- Python Package Installation ---
echo "--- Installing Python packages via pip3 ---"

# Install pip packages one by one
pip3 install nemo_toolkit[asr]
echo "Finish to install NeMo"

# --- Clone ROS Packages ---
echo "--- Cloning ROS packages ---"
cd .. # Go up one directory to clone sibling repositories

SOBITS_MSGS_REPO="sobits_msgs"
# Check if the repository already exists
if [ ! -d "$SOBITS_MSGS_REPO" ]; then
    echo "Cloning $SOBITS_MSGS_REPO repository..."
    git clone -b humble-devel https://github.com/TeamSOBITS/sobits_msgs.git
    echo "$SOBITS_MSGS_REPO cloned successfully."
else
    echo "$SOBITS_MSGS_REPO repository already exists. Skipping clone."
fi

# Return to the original script directory
cd "$SCRIPT_DIR" || { echo "Error: Could not return to $SCRIPT_DIR"; exit 1; }
pip3 uninstall setuptools -y
pip3 install setuptools==65.5.1

# --- Download NeMo Models ---
echo "--- Downloading NeMo ASR models ---"
# Change to the script's directory
cd "$SCRIPT_DIR/speech_recognition_nemo" || { echo "Error: Could not change to $SCRIPT_DIR"; exit 1; }
python3 model_download.py
echo "NeMo ASR models downloaded."

echo "╚══╣ Install: speech_recognition_nemo (FINISHED) ╠══╝"