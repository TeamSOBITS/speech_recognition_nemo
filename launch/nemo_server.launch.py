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
        DeclareLaunchArgument(
            'sample_rate',
            default_value='48000',
            description='Audio sample rate'
        ),
        DeclareLaunchArgument(
            'chunk_size',
            default_value='1024',
            description='Audio chunk size'
        ),
        DeclareLaunchArgument(
            'channels',
            default_value='1',
            description='Number of audio channels'
        ),

        # Launch the NemoServer Node
        Node(
            package='speech_recognition_nemo',
            executable='nemo_server',
            name='nemo_asr_action_server',
            parameters=[
                {'model_name': LaunchConfiguration('model_name')},
                {'sample_rate': LaunchConfiguration('sample_rate')},
                {'chunk_size': LaunchConfiguration('chunk_size')},
                {'channels': LaunchConfiguration('channels')},
            ],
            output='screen'
        ),
    ])