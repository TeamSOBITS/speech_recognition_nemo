#!/bin/bash
echo "╔══╣ Install: speech_recognition_nemo (STARTING) ╠══╗"

set -e  # 途中で失敗したら即終了

# apt の自動確認スキップ
export DEBIAN_FRONTEND=noninteractive

sudo apt update -y

sudo apt install -y ros-humble-vision-msgs

sudo apt install -y libportaudio2 libportaudiocpp0 portaudio19-dev


# Python パッケージのインストールと整備
pip3 install nemo_toolkit[asr]

pip3 install sounddevice

pip3 install playsound

pip3 install pyaudio

pip3 install -q soundfile pygame

# ROS パッケージのクローン
cd ~/colcon_ws/src/
if [ ! -d "sobits_msgs" ]; then
    git clone -b humble-devel https://github.com/TeamSOBITS/sobits_msgs.git
else
    echo "sobits_msgs リポジトリはすでに存在します。スキップします。"
fi

echo "╚══╣ Install: speech_recognition_nemo (FINISHED) ╠══╝"