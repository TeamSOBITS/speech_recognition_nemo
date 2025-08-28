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

class NemoServer(Node):
    def __init__(self):
        super().__init__('nemo_server')

        self.PULSEAUDIO_SOURCE_NAME_PATTERN = re.compile(r'^\s*(?:Name|名前):\s*(.+)\s*$')
        self.PULSEAUDIO_SAMPLE_SPEC_PATTERN = re.compile(r'^\s*(?:Sample Specification|サンプル仕様):\s*(\S+)\s+(\d+)ch\s+(\d+)Hz')

        self.declare_parameter('model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

        self.SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
        share_dir = get_package_share_directory('speech_recognition_nemo')
        self.sound_file_directory = os.path.join(os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..')),
                                                 'src', 'speech_recognition_nemo', 'sound_file')
        os.makedirs(self.sound_file_directory, exist_ok=True)
        self.wav_path = os.path.join(self.sound_file_directory, 'output.wav')
        self.get_logger().info(f"Output path: {self.wav_path}")

        self.source_name, self.sample_rate, self.channels = self.get_pulseaudio_source_info()
        if self.source_name is None:
            self.get_logger().fatal("Failed to get default microphone")
            return

        if self.sample_rate is None or self.channels is None:
            self.sample_rate = 16000
            self.channels = 1
            self.get_logger().warn("Failed to get sample rate or channel info. Using default: 16kHz / Mono")

        self.get_logger().info(f"Microphone: {self.source_name}, Sample rate: {self.sample_rate} Hz, Channels: {self.channels}")

        try:
            self.get_logger().info(f"Loading model: {self.model_name}")
            self.model = nemo_asr.models.EncDecRNNTBPEModel.from_pretrained(
                model_name=self.model_name
            ).to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))
        except Exception as e:
            self.get_logger().fatal(f"Model loading failed: {e}")
            return

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
            self.get_logger().warn(f"⚠️ ffplay not found: {filename}")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"⚠️ Failed to play sound: {e}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error during sound playback: {e}")

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
        feedback_rate = max(goal_handle.request.feedback_rate, 0.1)
        silent = goal_handle.request.silent_mode

        self.get_logger().info(f"Recording started for {timeout_sec} seconds (feedback interval: {feedback_rate}s)")

        if not silent:
            start_sound_thread = threading.Thread(target=self.play_sound, args=('start_sound.mp3',), daemon=True)
            start_sound_thread.start()

        self._cleanup_files()

        audio_q = queue.Queue()
        raw_audio_chunks = []
        feedback_buffer_chunks = []
        feedback_count = 0
        
        bytes_per_sample = 2  # s16le
        chunk_duration = 0.5
        chunk_size_bytes = int(self.sample_rate * self.channels * bytes_per_sample * chunk_duration)
        feedback_buffer_limit = int(self.sample_rate * self.channels * bytes_per_sample * feedback_rate)

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

            raw_audio_chunks.append(chunk)
            feedback_buffer_chunks.append(chunk)

            # フィードバック処理
            current_buffer_size = sum(len(c) for c in feedback_buffer_chunks)
            if current_buffer_size >= feedback_buffer_limit:
                audio_to_transcribe_raw = b"".join(feedback_buffer_chunks)
                
                # feedback_buffer_limitを超えた分を次のバッファの先頭に移動
                remainder_size = current_buffer_size - feedback_buffer_limit
                if remainder_size > 0:
                    remainder_chunk = audio_to_transcribe_raw[-remainder_size:]
                    audio_to_transcribe_raw = audio_to_transcribe_raw[:-remainder_size]
                    feedback_buffer_chunks = [remainder_chunk]
                else:
                    feedback_buffer_chunks = []

                # フィードバック用音声ファイルを保存
                feedback_count += 1
                feedback_filename = f"feedback_{feedback_count:03d}.wav"
                feedback_filepath = os.path.join(self.sound_file_directory, feedback_filename)
                self._save_buffer_to_wav([audio_to_transcribe_raw], feedback_filepath, self.sample_rate, self.channels)

                audio_to_transcribe_np = np.frombuffer(audio_to_transcribe_raw, dtype=np.int16).astype(np.float32) / 32768.0

                try:
                    resampled = self.resample_audio(audio_to_transcribe_np, self.sample_rate, 16000, self.channels)
                    with torch.no_grad():
                        result = self.model.transcribe([resampled])

                    if result and result[0].text.strip():
                        fb = SpeechRecognition.Feedback()
                        fb.addition_text = result[0].text
                        goal_handle.publish_feedback(fb)
                        self.get_logger().info(f"Feedback: {result[0].text}, Score: {result[0].score:.2f}")

                except Exception as e:
                    self.get_logger().warn(f"Recognition failed during feedback: {e}")

        # 録音プロセスの終了
        if self.parec_proc:
            self.parec_proc.terminate()
        thread.join(timeout=2.0)
        
        # 最終音声ファイルの保存
        if raw_audio_chunks:
            self._save_buffer_to_wav(raw_audio_chunks, self.wav_path, self.sample_rate, self.channels)
            self.get_logger().info(f"Final audio saved to: {self.wav_path}")

        # 最終的な音声認識
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