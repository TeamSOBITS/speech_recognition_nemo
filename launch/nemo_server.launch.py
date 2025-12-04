from launch import LaunchDescription
from launch_ros.actions import Node

params = {
    'model_name': 'nvidia/parakeet-tdt-0.6b-v2',
    'mic_volume': '',
    'use_feedback': True,
    'vad_name': 'ten_vad',
    'hop_size': 256,
    'threshold': 0.5,
    'min_wipe_duration': 0.2,
    'extra_audio_duration_sec': 0.2,
    "use_echo_cancel": False,
    "noise_suppression": False,
    "analog_gain_control": False,
    "digital_gain_control": False,
}

nemo_server_node = Node(
    package='speech_recognition_nemo',
    executable='nemo_server',
    name='nemo_asr_action_server',
    parameters=[params],
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([
        nemo_server_node
    ])