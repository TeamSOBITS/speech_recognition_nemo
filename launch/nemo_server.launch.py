from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='nvidia/parakeet-tdt-0.6b-v2',
            description='NeMo ASR model name'
        ),
        DeclareLaunchArgument(
            'use_feedback',
            default_value='True',
            description='Whether to use feedback'
        ),
        DeclareLaunchArgument(
            'vad_name',
            default_value='ten_vad',
            description='VAD model name'
        ),
        DeclareLaunchArgument(
            'hop_size',
            default_value='256',
            description='VAD hop size'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.5',
            description='VAD threshold'
        ),
        DeclareLaunchArgument(
            'min_wipe_duration',
            default_value='0.2',
            description='Minimum duration for a speech segment to be considered valid for WIP feedback'
        ),
        DeclareLaunchArgument(
            'extra_audio_duration_sec',
            default_value='0.2',
            description='Extra audio duration to include before and after speech for WIP feedback'
        ),
        Node(
            package='speech_recognition_nemo',
            executable='nemo_server',
            name='nemo_asr_action_server',
            parameters=[
                {'model_name': LaunchConfiguration('model_name')},
                {'vad_name': LaunchConfiguration('vad_name')},
                {'hop_size': LaunchConfiguration('hop_size')},
                {'threshold': LaunchConfiguration('threshold')},
                {'use_feedback': LaunchConfiguration('use_feedback')},
                {'min_wipe_duration': LaunchConfiguration('min_wipe_duration')},
                {'extra_audio_duration_sec': LaunchConfiguration('extra_audio_duration_sec')}
            ],
            output='screen'
        ),
    ])