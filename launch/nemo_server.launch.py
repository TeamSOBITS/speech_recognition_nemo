from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='nvidia/parakeet-tdt-0.6b-v2',
        description='STT Model Name (en: "nvidia/parakeet-tdt-0.6b-v2", jp: "nvidia/parakeet-tdt_ctc-0.6b-ja").'
    )

    mic_volume_arg = DeclareLaunchArgument(
        'mic_volume',
        default_value='',
        description='Mic volume config'
    )

    use_feedback_arg = DeclareLaunchArgument(
        'use_feedback',
        default_value='True',
        description='True/False of using feedback'
    )

    vad_name_arg = DeclareLaunchArgument(
        'vad_name',
        default_value='ten_vad',
        description='Voice Activity Detection Name'
    )

    hop_size_arg = DeclareLaunchArgument(
        'hop_size',
        default_value='256',
        description='Hop Size'
    )

    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Threshold of VAD(?)'
    )

    min_wipe_duration_arg = DeclareLaunchArgument(
        'min_wipe_duration',
        default_value='0.2',
        description='min wipe duration'
    )

    extra_audio_duration_sec_arg = DeclareLaunchArgument(
        'extra_audio_duration_sec',
        default_value='0.2',
        description='Extra Audio Duration Sec'
    )

    use_echo_cancel_arg = DeclareLaunchArgument(
        'use_echo_cancel',
        default_value='False',
        description='Enable Self Sounds Canceling'
    )

    noise_suppression_arg = DeclareLaunchArgument(
        'noise_suppression',
        default_value='False',
        description='Extra Audio Duration Sec'
    )

    analog_gain_control_arg = DeclareLaunchArgument(
        'analog_gain_control',
        default_value='False',
        description='Analog Gain Control'
    )

    digital_gain_control_arg = DeclareLaunchArgument(
        'digital_gain_control',
        default_value='False',
        description='Digital Gain Control'
    )

    nemo_server_node = Node(
        package='speech_recognition_nemo',
        executable='nemo_server',
        name='nemo_asr_action_server',
        parameters=[
            {
                'model_name': LaunchConfiguration("model_name"),
                'mic_volume': LaunchConfiguration("mic_volume"),
                'use_feedback': LaunchConfiguration("use_feedback"),
                'vad_name': LaunchConfiguration("vad_name"),
                'hop_size': LaunchConfiguration("hop_size"),
                'threshold': LaunchConfiguration("threshold"),
                'min_wipe_duration': LaunchConfiguration("min_wipe_duration"),
                'extra_audio_duration_sec': LaunchConfiguration("extra_audio_duration_sec"),
                'use_echo_cancel': LaunchConfiguration("use_echo_cancel"),
                'noise_suppression': LaunchConfiguration("noise_suppression"),
                'analog_gain_control': LaunchConfiguration("analog_gain_control"),
                'digital_gain_control': LaunchConfiguration("digital_gain_control"),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        model_name_arg,
        mic_volume_arg,
        use_feedback_arg,
        vad_name_arg,
        hop_size_arg,
        threshold_arg,
        min_wipe_duration_arg,
        extra_audio_duration_sec_arg,
        use_echo_cancel_arg,
        noise_suppression_arg,
        analog_gain_control_arg,
        digital_gain_control_arg,
        nemo_server_node,
    ])