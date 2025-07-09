import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from sobits_interfaces.action import SpeechRecognition
from ament_index_python.packages import get_package_share_directory

import nemo.collections.asr as nemo_asr

import subprocess
import re
import os
import threading
import time

class NemoServer(Node):
    SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
    
    PULSEAUDIO_SOURCE_NAME_PATTERN = re.compile(r'^\s*(?:Name|名前):\s*(.+)\s*$')
    PULSEAUDIO_SAMPLE_SPEC_PATTERN = re.compile(r'^\s*(?:Sample Specification|サンプル仕様):\s*(\S+)\s+(\d+)ch\s+(\d+)Hz')

    def __init__(self):
        super().__init__('nemo_server')
        AUDIO_OUTPUT_DIR = os.path.join(get_package_share_directory('speech_recognition_nemo'), 'sound_file')
        self.wav_path = os.path.join(AUDIO_OUTPUT_DIR, "output.wav")
        
        os.makedirs(AUDIO_OUTPUT_DIR, exist_ok=True)
        self.get_logger().info(f"Speech recognition output directory: {AUDIO_OUTPUT_DIR}")

        YELLOW = '\033[93m'
        ENDC = '\033[0m'

        self.default_audio_source, self.default_sample_rate, self.default_channels = self.get_pulseaudio_source_info()

        if self.default_audio_source:
            self.get_logger().info(f"Current microphone (PulseAudio default source): {self.default_audio_source}")
            if self.default_sample_rate is not None and self.default_channels is not None:
                self.get_logger().info(f"  Sample Rate: {self.default_sample_rate} Hz, Channels: {self.default_channels}")
            else:
                self.get_logger().warn(f"  Could not retrieve sample rate or channels for PulseAudio source '{self.default_audio_source}'.")
        else:
            self.get_logger().warn("Could not determine PulseAudio default input source. Speech recognition may not function correctly.")

        self.declare_parameter('model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        self.nemo_model_name = self.get_parameter('model_name').get_parameter_value().string_value

        self.get_logger().info(f"Loading NeMo model: {self.nemo_model_name}")
        try:
            self.model = nemo_asr.models.ASRModel.from_pretrained(self.nemo_model_name)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load NeMo model: {e}")
            self.model = None
            return

        self.action_server = ActionServer(
            self, SpeechRecognition, "speech_recognition",
            execute_callback=self._execute_speech_to_text,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(f"Microphone: {YELLOW}{self.default_audio_source}{ENDC}")
        self.get_logger().info(f"{YELLOW}NeMo Server is READY and waiting for requests.{ENDC}")

    def _goal_callback(self, goal_request):
        self.get_logger().info("Received goal request.")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    async def _execute_speech_to_text(self, goal_handle):
        response = SpeechRecognition.Result()
        if self.model is None:
            self.get_logger().error("NeMo model is not loaded.")
            goal_handle.abort()
            response.result_text = "System Error"
            return response
        
        duration = goal_handle.request.timeout_sec
        self.get_logger().info(f"Received action goal. Recording duration: {duration} seconds")

        try:
            if not goal_handle.request.silent_mode:
                start_sound_thread = threading.Thread(target=self._play_sound_with_ffplay, args=('start_sound.mp3',))
                start_sound_thread.start()
                start_sound_thread.join()
            else:
                self.get_logger().info("Silent mode enabled, skipping start sound playback.")

            self.record_and_resample(self.default_audio_source, duration, self.wav_path, self.default_sample_rate, self.default_channels)

            self.get_logger().info("Starting transcription.")
            transcribe_start_time = time.time()
            result = self.model.transcribe([self.wav_path])
            text = result[0].text if result else ""
            transcribe_elapsed = time.time() - transcribe_start_time

            if not text:
                response.result_text = "No speech recognized."
                self.get_logger().warn("Transcription result is empty.")
            else:
                response.result_text = text
                self.get_logger().info(f"Recognition Result: {text}")
            self.get_logger().info(f"Transcription complete (processing time: {transcribe_elapsed:.2f} seconds)")

        except Exception as e:
            self.get_logger().error(f"Error during recognition process: {e}")
            goal_handle.abort()
            response.result_text = f"Recognition Error: {e}"
            return response
        finally:
            if not goal_handle.request.silent_mode:
                threading.Thread(target=self._play_sound_with_ffplay, args=('end_sound.mp3',)).start()

        goal_handle.succeed()
        return response

    def _play_sound_with_ffplay(self, filename):
        sound_path = os.path.join(self.SOUND_FILES_PATH, filename)
        try:
            command = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', sound_path]
            subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError:
            self.get_logger().warn("ffplay command not found. Ensure FFmpeg is installed and in your PATH.")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to execute ffplay for '{filename}': {e.stderr.decode(errors='ignore').strip()}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error during sound playback (ffplay) for '{filename}': {e}")

    def get_pulseaudio_source_info(self):
        default_source_name = None

        try:
            info_result = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True)

            for line in info_result.stdout.splitlines():
                if "Default Source:" in line or "デフォルトソース:" in line:
                    default_source_name = line.split(':', 1)[1].strip()
                    break
            
            if not default_source_name:
                self.get_logger().warn("Could not find default source name from PulseAudio info.")
                return None, None, None

            list_result = subprocess.run(['pactl', 'list', 'sources'], capture_output=True, text=True, check=True)
            
            all_source_blocks = []
            current_block_lines = []

            for line in list_result.stdout.splitlines():
                if line.strip().startswith("Source #"):
                    if current_block_lines:
                        all_source_blocks.append(current_block_lines)
                    current_block_lines = [line]
                else:
                    current_block_lines.append(line)
            if current_block_lines:
                all_source_blocks.append(current_block_lines)

            for block_lines in all_source_blocks:
                for line in block_lines:
                    name_match = self.PULSEAUDIO_SOURCE_NAME_PATTERN.match(line)
                    if name_match:
                        current_block_source_name = name_match.group(1).strip()
                        if current_block_source_name == default_source_name:
                            self.get_logger().info(f"Identified PulseAudio source info block by exact match. Found source name: '{current_block_source_name}'")
                            rate, channels = self.parse_sample_rate_and_channels(block_lines)
                            if rate is not None and channels is not None:
                                return default_source_name, rate, channels
                            else:
                                self.get_logger().warn(f"  Could not find sample rate or channels within info block for exact match source '{current_block_source_name}'.")
                                return default_source_name, None, None

            self.get_logger().info(f"No exact match found in Phase 1. Phase 2: Checking for partial matches...")
            for block_lines in all_source_blocks:
                for line in block_lines:
                    name_match = self.PULSEAUDIO_SOURCE_NAME_PATTERN.match(line)
                    if name_match:
                        current_block_source_name = name_match.group(1).strip()
                        if default_source_name in current_block_source_name or current_block_source_name in default_source_name:
                            self.get_logger().info(f"Identified PulseAudio source info block by partial match. Target source name: '{default_source_name}', Found source name: '{current_block_source_name}'")
                            rate, channels = self.parse_sample_rate_and_channels(block_lines)
                            if rate is not None and channels is not None:
                                return default_source_name, rate, channels
                            else:
                                self.get_logger().warn(f"  Could not find sample rate or channels within info block for partial match source '{current_block_source_name}'.")
                                return default_source_name, None, None

            self.get_logger().warn(f"Could not find detailed info block for PulseAudio default source '{default_source_name}' (neither exact nor partial match).")
            return default_source_name, None, None

        except FileNotFoundError:
            self.get_logger().error("pactl command not found. Ensure PulseAudio is installed and in your PATH.")
            return None, None, None
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to execute pactl command: {e.stderr.strip()}")
            return None, None, None
        except Exception as e:
            self.get_logger().warn(f"Unexpected error while getting PulseAudio source info: {e}")
            return None, None, None

    def parse_sample_rate_and_channels(self, lines):
        for line in lines:
            m = self.PULSEAUDIO_SAMPLE_SPEC_PATTERN.match(line)
            if m:
                return int(m.group(3)), int(m.group(2))
        return None, None

    def record_and_resample(self, source_name, duration_sec, output_wav, sample_rate, channels):
        parec_cmd = [
            'parec',
            '-d', source_name,
            '--format=s16le',
            '--channels=1',
            '--rate', str(sample_rate),
            '--file-format=raw',
        ]
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 's16le',
            '-ar', str(sample_rate),
            '-ac', str(1),
            '-i', 'pipe:0',
            '-t', str(duration_sec),
            '-ar', '16000',
            '-ac', '1',
            '-y',
            output_wav,
            '-loglevel', 'quiet'
        ]

        parec_proc = None
        ffmpeg_proc = None
        
        try:
            parec_proc = subprocess.Popen(parec_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=parec_proc.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            ffmpeg_proc.wait(timeout=duration_sec + 5) 

            if parec_proc.poll() is None:
                parec_proc.terminate()
                parec_proc.wait(timeout=2) 

            parec_stderr = parec_proc.stderr.read().decode(errors='ignore').strip()
            ffmpeg_stderr = ffmpeg_proc.stderr.read().decode(errors='ignore').strip()
            
            if parec_stderr:
                self.get_logger().warn(f"Error output from parec:\n{parec_stderr}")
            if ffmpeg_stderr and "Conversion failed!" in ffmpeg_stderr:
                self.get_logger().warn(f"Error output from ffmpeg:\n{ffmpeg_stderr}")

            if ffmpeg_proc.returncode != 0:
                raise RuntimeError(f"ffmpeg process exited with an error (return code: {ffmpeg_proc.returncode})")
        except FileNotFoundError as e:
            self.get_logger().error(f"Required command for recording not found: {e.filename}. Ensure 'parec' and 'ffmpeg' are installed and in your PATH.")
            raise
        except subprocess.TimeoutExpired:
            self.get_logger().error("Recording process timed out.")
            if parec_proc and parec_proc.poll() is None:
                parec_proc.kill()
            if ffmpeg_proc and ffmpeg_proc.poll() is None:
                ffmpeg_proc.kill()
            raise RuntimeError("Recording timeout")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during recording process: {e}")
            raise
        finally:
            if parec_proc and parec_proc.poll() is None:
                self.get_logger().warn(f"Forcefully terminating still-running parec process in finally block.")
                parec_proc.kill() 
                parec_proc.wait() 
            if ffmpeg_proc and ffmpeg_proc.poll() is None:
                self.get_logger().warn(f"Forcefully terminating still-running ffmpeg process in finally block.")
                ffmpeg_proc.kill() 
                ffmpeg_proc.wait() 

        try:
            ffprobe_cmd = [
                'ffprobe',
                '-v', 'error',
                '-show_entries', 'format=duration',
                '-of', 'default=noprint_wrappers=1:nokey=1',
                output_wav
            ]
            ffprobe_result = subprocess.run(ffprobe_cmd, capture_output=True, text=True, check=True)
            actual_duration = float(ffprobe_result.stdout.strip())
        except FileNotFoundError:
            self.get_logger().warn("ffprobe command not found. Cannot verify actual length of recorded file.")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to execute ffprobe: {e.stderr.strip()}")
        except ValueError:
            self.get_logger().warn(f"ffprobe returned invalid duration data.")


def main(args=None):
    rclpy.init(args=args)
    node = NemoServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()