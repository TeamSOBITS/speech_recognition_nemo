from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([ 
        # Declare arguments for NeMo ASR Node parameters
        DeclareLaunchArgument(
            'model_name',
            default_value='nvidia/parakeet-tdt-0.6b-v2', # Default to Japanese model
            description='NeMo ASR model name'
            # 英語モデル  : nvidia/parakeet-tdt-0.6b-v2
            # 日本語モデル: nvidia/parakeet-tdt_ctc-0.6b-ja
        ),

        # Launch the NemoServer Node
        Node(
            package='speech_recognition_nemo',
            executable='nemo_server',
            name='nemo_asr_action_server',
            parameters=[
                {'model_name': LaunchConfiguration('model_name')},
            ],
            output='screen'
        ),
    ])