import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sobits_interfaces.action import SpeechRecognition

from ament_index_python.packages import get_package_share_directory

import nemo.collections.asr as nemo_asr
import subprocess
import numpy as np
import torch
import threading
import queue
import time
import wave
import os
import re
import glob
from scipy.signal import resample_poly 

from .vad import VadProcessor

class NemoServer(Node):
    def __init__(self):
        super().__init__('nemo_server')

        self.PULSEAUDIO_SOURCE_NAME_PATTERN = re.compile(r'^\s*(?:Name|名前):\s*(.+)\s*$')
        self.PULSEAUDIO_SAMPLE_SPEC_PATTERN = re.compile(r'^\s*(?:Sample Specification|サンプル仕様):\s*(\S+)\s+(\d+)ch\s+(\d+)Hz')

        self.declare_parameter('model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        # Parameters for VAD logic (Declared once here)
        self.declare_parameter('min_wipe_duration', 0.1)

        self.SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
        share_dir = get_package_share_directory('speech_recognition_nemo')
        self.sound_file_directory = os.path.join(os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..')),
                                                 'src', 'speech_recognition_nemo', 'sound_file')
        os.makedirs(self.sound_file_directory, exist_ok=True)
        self.wav_path = os.path.join(self.sound_file_directory, 'output.wav')
        self.get_logger().info(f"Output path: {self.wav_path}")
        self.file_counter = 0 # Counter for sequential file naming

        # Get PulseAudio info
        source_name, sample_rate, channels = self.get_pulseaudio_source_info()

        if source_name is None:
            self.get_logger().warn("Failed to get default microphone. Using fallback settings.")
            self.source_name = "default"  # Default source that works on many systems
            self.sample_rate = 44100      # Common microphone sample rate
            self.channels = 2             # Common stereo channel count
        else:
            self.source_name = source_name
            self.sample_rate = sample_rate if sample_rate is not None else 44100
            self.channels = channels if channels is not None else 2
        
        # Log the final settings
        self.get_logger().info(f"Microphone: {self.source_name}, Sample rate: {self.sample_rate} Hz, Channels: {self.channels}")

        try:
            self.get_logger().info(f"Loading model: {self.model_name}")
            self.model = nemo_asr.models.EncDecRNNTBPEModel.from_pretrained(
                model_name=self.model_name
            ).to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))
        except Exception as e:
            self.get_logger().fatal(f"Model loading failed: {e}")
            return

        self.vad_processor = VadProcessor(self)
        self.hop_size, self.vad_chunk_size_bytes = self.vad_processor.get_hop_size()

        self.action_server = ActionServer(
            self,
            SpeechRecognition,
            "speech_recognition",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        YELLOW = '\033[93m'
        ENDC = '\033[0m'
        self.get_logger().info(f"Microphone: {YELLOW}{self.source_name}{ENDC}")
        self.get_logger().info(f"Sample Rate: {self.sample_rate} Hz, Channels: {self.channels}")
        self.get_logger().info(f"{YELLOW}NeMo Server is READY and waiting for requests.{ENDC}")

        self.parec_proc = None

    def get_pulseaudio_source_info(self):
        try:
            info = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True)
            default_source = None
            for line in info.stdout.splitlines():
                if "Default Source:" in line or "デフォルトソース:" in line:
                    default_source = line.split(':', 1)[1].strip()
                    break
            if not default_source:
                self.get_logger().warn("Default PulseAudio source not found.")
                return None, None, None

            list_sources = subprocess.run(['pactl', 'list', 'sources'], capture_output=True, text=True, check=True)
            blocks = []
            current_block = []
            for line in list_sources.stdout.splitlines():
                if line.strip().startswith("Source #"):
                    if current_block:
                        blocks.append(current_block)
                    current_block = [line]
                else:
                    current_block.append(line)
            if current_block:
                blocks.append(current_block)

            for block in blocks:
                for line in block:
                    m = self.PULSEAUDIO_SOURCE_NAME_PATTERN.match(line)
                    if m and m.group(1).strip() == default_source:
                        rate, channels = self.parse_sample_rate_and_channels(block)
                        return default_source, rate, channels

            self.get_logger().warn(f"Could not find detailed info for source '{default_source}'.")
            return default_source, None, None

        except Exception as e:
            self.get_logger().error(f"PulseAudio source info error: {e}")
            return None, None, None

    def parse_sample_rate_and_channels(self, lines):
        for line in lines:
            m = self.PULSEAUDIO_SAMPLE_SPEC_PATTERN.match(line)
            if m:
                return int(m.group(3)), int(m.group(2))
        return None, None

    def play_sound(self, filename):
        path = os.path.join(self.SOUND_FILES_PATH, filename)
        try:
            subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', path],
                           check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError:
            self.get_logger().warn(f"ffplay not found: {filename}")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to play sound: {e}")
        except Exception as e:
            self.get_logger().warn(f"Error during sound playback: {e}")

    def resample_audio(self, audio_np: np.ndarray, orig_sr: int, target_sr: int, channels: int):
        if channels > 1:
            audio_np = audio_np.reshape(-1, channels)
            audio_np = audio_np.mean(axis=1)
        return resample_poly(audio_np, target_sr, orig_sr)

    def goal_callback(self, goal_request):
        self.get_logger().info("Goal received")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        timeout_sec = goal_handle.request.timeout_sec
        silent = goal_handle.request.silent_mode

        self.get_logger().info(f"Recording started for {timeout_sec} seconds")

        if not silent:
            start_sound_thread = threading.Thread(target=self.play_sound, args=('start_sound.mp3',), daemon=True)
            start_sound_thread.start()

        self.file_counter = 0 # Counter for sequential file naming
        self._cleanup_files()

        audio_q = queue.Queue()
        raw_audio_chunks = []
        feedback_buffer_chunks = []
        
        # Set chunk size based on VAD hop size
        chunk_size_bytes = self.vad_chunk_size_bytes
        
        def capture():
            try:
                proc = subprocess.Popen([
                    'parec', '-d', self.source_name,
                    '--format=s16le',
                    '--channels', str(self.channels),
                    '--rate', str(self.sample_rate),
                    '--file-format=raw',
                ], stdout=subprocess.PIPE)
                self.parec_proc = proc
                
                while True:
                    if not rclpy.ok() or self.parec_proc.poll() is not None:
                        break
                    chunk = self.parec_proc.stdout.read(chunk_size_bytes)
                    if not chunk:
                        break
                    audio_q.put(chunk)

            except Exception as e:
                self.get_logger().error(f"Recording error: {e}")
            finally:
                audio_q.put(None)

        thread = threading.Thread(target=capture, daemon=True)
        thread.start()

        start = time.time()
        response = SpeechRecognition.Result()

        is_speaking = False
        is_potential_speaking = False
        potential_speech_start_time = None
        
        # Manage audio buffer for VAD processing
        vad_audio_buffer = np.array([], dtype=np.int16)
        
        # Parameters for VAD logic
        min_wipe_duration = self.get_parameter('min_wipe_duration').get_parameter_value().double_value
        
        while rclpy.ok():
            now = time.time()
            if now - start >= timeout_sec:
                self.get_logger().info("Timeout reached.")
                break
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
                if self.parec_proc:
                    self.parec_proc.terminate()
                return response
            
            try:
                chunk = audio_q.get(timeout=0.1)
            except queue.Empty:
                continue
            if chunk is None:
                break
            
            # Keep raw audio for final WAV file
            raw_audio_chunks.append(chunk)
            
            # Resample audio for VAD processing
            current_audio_np = np.frombuffer(chunk, dtype=np.int16)
            resampled_data = self.resample_audio(current_audio_np, self.sample_rate, 16000, self.channels)
            vad_audio_buffer = np.concatenate([vad_audio_buffer, resampled_data.astype(np.int16)])
            
            # Process VAD for each hop size chunk
            while len(vad_audio_buffer) >= self.hop_size:
                vad_chunk = vad_audio_buffer[:self.hop_size]
                vad_audio_buffer = vad_audio_buffer[self.hop_size:]

                is_voice_now = self.vad_processor.vad_processor(vad_chunk.tobytes())

                if is_voice_now:
                    # 音声が検出された場合
                    if not is_potential_speaking:
                        is_potential_speaking = True
                        potential_speech_start_time = now
                    
                    if not is_speaking and (now - potential_speech_start_time) > min_wipe_duration:
                        is_speaking = True
                        self.get_logger().info("Speech detected, starting new segment.")
                        feedback_buffer_chunks = raw_audio_chunks[:]
                    
                    if is_speaking:
                        feedback_buffer_chunks.append(chunk)
                
                else:
                    # 音声が検出されなかった場合
                    is_potential_speaking = False
                    
                    if is_speaking:
                        # 発話終了と判断し、フィードバック処理を実行
                        self.get_logger().info("Speech segment ended. Processing feedback.")
                        
                        # バッファに溜まったデータを結合し、認識にかける
                        if feedback_buffer_chunks:
                            # Save the feedback audio segment
                            self.file_counter += 1
                            feedback_wav_path = os.path.join(self.sound_file_directory, f'feedback_{self.file_counter}.wav')
                            if self._save_buffer_to_wav(feedback_buffer_chunks, feedback_wav_path, self.sample_rate, self.channels):
                                self.get_logger().info(f"Feedback audio saved to: {feedback_wav_path}")
                            
                            feedback_audio_data = np.frombuffer(b''.join(feedback_buffer_chunks), dtype=np.int16).astype(np.float32) / 32768.0
                            if feedback_audio_data.size > 0:
                                resampled = self.resample_audio(feedback_audio_data, self.sample_rate, 16000, self.channels)
                                with torch.no_grad():
                                    feedback_result = self.model.transcribe([resampled])
                                feedback_text = feedback_result[0].text if feedback_result and feedback_result[0].text.strip() else "No speech recognized."
                                self.get_logger().info(f"Feedback Result: {feedback_text}")
                                
                                # Use action feedback instead of a separate publisher
                                feedback = SpeechRecognition.Feedback()
                                # Correcting the attribute name to 'addition_text' based on your provided action definition
                                feedback.addition_text = feedback_text
                                goal_handle.publish_feedback(feedback)
                        
                        # VAD状態とバッファをリセット
                        is_speaking = False
                        feedback_buffer_chunks = []
        # End of recording loop
        if self.parec_proc:
            self.parec_proc.terminate()
        thread.join(timeout=2.0)
        
        # 最終的な音声ファイルと認識処理（タイムアウト時）
        if raw_audio_chunks:
            self._save_buffer_to_wav(raw_audio_chunks, self.wav_path, self.sample_rate, self.channels)
            self.get_logger().info(f"Final audio saved to: {self.wav_path}")

        try:
            audio_data = np.frombuffer(b''.join(raw_audio_chunks), dtype=np.int16).astype(np.float32) / 32768.0
            if audio_data.size == 0:
                response.result_text = "No audio recorded."
            else:
                resampled = self.resample_audio(audio_data, self.sample_rate, 16000, self.channels)
                with torch.no_grad():
                    result = self.model.transcribe([resampled])
                response.result_text = result[0].text if result and result[0].text.strip() else "No speech recognized."
                self.get_logger().info(f"Final Result: {response.result_text}, Score: {result[0].score:.2f}")
        except Exception as e:
            response.result_text = f"Recognition error: {e}"
            self.get_logger().error(f"Final recognition failed: {e}")

        if not silent:
            threading.Thread(target=self.play_sound, args=('end_sound.mp3',), daemon=True).start()

        goal_handle.succeed()
        return response

    def _cleanup_files(self):
        wip_files = glob.glob(os.path.join(self.sound_file_directory, "*.wav"))
        for f in wip_files:
            try:
                os.remove(f)
                self.get_logger().info(f"Removed old WAV file: {f}")
            except Exception as e:
                self.get_logger().warn(f"Failed to remove WAV file {f}: {e}")
    
    def _save_buffer_to_wav(self, frames, file_path, sample_rate, channels):
        if not frames:
            self.get_logger().warn("No frames to save.")
            return False
        
        try:
            with wave.open(file_path, 'wb') as wf:
                wf.setnchannels(channels)
                wf.setsampwidth(2) # 16bit = 2 bytes
                wf.setframerate(sample_rate)
                wf.writeframes(b"".join(frames))
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save WAV file: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = NemoServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
