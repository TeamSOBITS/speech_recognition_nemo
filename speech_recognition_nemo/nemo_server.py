import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

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
        AUDIO_OUTPUT_DIR = os.path.join(os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..')),
                                        'src', 'speech_recognition_nemo', 'sound_file')
        os.makedirs(AUDIO_OUTPUT_DIR, exist_ok=True)
        self.wav_path = os.path.join(AUDIO_OUTPUT_DIR, 'output.wav')
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

        audio_q = queue.Queue()
        buffer_audio = []
        all_audio = []
        chunk_duration = 0.5
        chunk_size = int(self.sample_rate * self.channels * 2 * chunk_duration)

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
                    chunk = proc.stdout.read(chunk_size)
                    if not chunk or len(chunk) < chunk_size:
                        break
                    audio_np = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
                    audio_q.put(audio_np)
                    all_audio.append(chunk)
                audio_q.put(None)
            except Exception as e:
                self.get_logger().error(f"Recording error: {e}")
                audio_q.put(None)

        thread = threading.Thread(target=capture, daemon=True)
        thread.start()

        start = time.time()
        response = SpeechRecognition.Result()

        while rclpy.ok():
            now = time.time()
            if now - start >= timeout_sec:
                break
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal canceled")
                goal_handle.canceled()
                return response
            try:
                audio = audio_q.get(timeout=0.1)
            except queue.Empty:
                continue
            if audio is None:
                break

            buffer_audio.append(audio)

            if sum(len(a) for a in buffer_audio) >= int(self.sample_rate * feedback_rate):
                audio_concat = np.concatenate(buffer_audio)
                buffer_audio = []

                try:
                    resampled = self.resample_audio(audio_concat, self.sample_rate, 16000, self.channels)
                    start_infer = time.time()
                    with torch.no_grad():
                        result = self.model.transcribe([resampled])
                    if result and result[0].text:
                        fb = SpeechRecognition.Feedback()
                        fb.addition_text = result[0].text
                        goal_handle.publish_feedback(fb)
                        self.get_logger().info(f"Feedback: {result[0].text}, Score: {result[0].score:.2f}, Inference time: {time.time() - start_infer:.3f} sec")
                except Exception as e:
                    self.get_logger().warn(f"Recognition failed: {e}")

            time.sleep(0.01)
        try:
            with wave.open(self.wav_path, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(all_audio))
            self.get_logger().info(f"Audio saved to: {self.wav_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save audio: {e}")

        try:
            audio_data = np.frombuffer(b''.join(all_audio), dtype=np.int16).astype(np.float32) / 32768.0
            resampled = self.resample_audio(audio_data, self.sample_rate, 16000, self.channels)
            start_infer = time.time()
            with torch.no_grad():
                result = self.model.transcribe([resampled])
            response.result_text = result[0].text if result and result[0].text else "No speech recognized."
            self.get_logger().info(f"Result: {response.result_text}, Score: {result[0].score:.2f}, Inference time: {time.time() - start_infer:.3f} sec")
        except Exception as e:
            response.result_text = f"Recognition error: {e}"

        if not silent:
            threading.Thread(target=self.play_sound, args=('end_sound.mp3',), daemon=True).start()

        goal_handle.succeed()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NemoServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()