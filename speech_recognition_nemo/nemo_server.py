import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sobits_interfaces.action import SpeechRecognition
from ament_index_python.packages import get_package_share_directory

import numpy as np
import time
import os
import wave
import importlib
import traceback
import threading
from concurrent.futures import ThreadPoolExecutor
from . import audio_utils
try:
    from .vad import VadProcessor, VadSegmenter
    VAD_AVAILABLE = True
except ImportError:
    VAD_AVAILABLE = False

class STTActionServer(Node):
    def __init__(self):
        super().__init__('stt_action_server')

        self.declare_parameter('stt_name', 'nemo')
        self.stt_name = self.get_parameter('stt_name').get_parameter_value().string_value
        
        self.declare_parameter('use_feedback', True)
        self.declare_parameter('min_wipe_duration', 0.2)
        self.declare_parameter('extra_audio_duration_sec', 0.2)
        self.declare_parameter('max_speech_duration', 30.0)
        
        self.declare_parameter('vad_name', 'ten_vad')
        self.declare_parameter('hop_size', 256)
        self.declare_parameter('threshold', 0.5)

        self.package_name = 'speech_recognition_nemo'
        share_dir = get_package_share_directory(self.package_name)
        self.sound_file_directory = os.path.join(share_dir, 'sound_file')
        os.makedirs(self.sound_file_directory, exist_ok=True)

        self._stt_model_instance = None
        try:
            module_name = f'.engines.{self.stt_name}_engine'
            stt_model_module = importlib.import_module(module_name, package=self.package_name)
            class_name = f"{self.stt_name.capitalize()}Engine"
            ModelClass = getattr(stt_model_module, class_name)
            self._stt_model_instance = ModelClass(node=self)
            
            stream_info = f"Streamable: {self._stt_model_instance.is_streamable}, ExtVAD: {self._stt_model_instance.use_external_vad}"
            self.get_logger().info(f"Loaded STT Engine: '{self.stt_name}' ({stream_info})")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load engine '{self.stt_name}': {e}\n{traceback.format_exc()}")
            raise RuntimeError("Engine load failed.")

        self._init_audio_components()
        self.executor_pool = ThreadPoolExecutor(max_workers=1)

        self._action_server = ActionServer(
            self, SpeechRecognition, "speech_recognition",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        YELLOW, ENDC = '\033[93m', '\033[0m'
        self.get_logger().info(f"{YELLOW}STT Action Server ({self.stt_name}) is READY{ENDC}")

    def _init_audio_components(self):
        self.declare_parameter('use_echo_cancel', False)
        self.declare_parameter('noise_suppression', False)
        self.declare_parameter('analog_gain_control', False)
        self.declare_parameter('digital_gain_control', False)
        self.declare_parameter('mic_volume', '100%')

        if self._stt_model_instance.use_external_vad:
            if not VAD_AVAILABLE:
                self.get_logger().error("VAD module is required by engine but not found!")
                raise RuntimeError("VAD import failed.")
            self.vad_processor = VadProcessor(self)
            self.hop_size, self.vad_chunk_size_bytes, _ = self.vad_processor.get_specs()
        else:
            self.vad_processor = None
            self.vad_chunk_size_bytes = 1024 

        self.audio_sys = audio_utils.AudioSystem(
            self.get_logger(),
            self.get_parameter('use_echo_cancel').value,
            self.get_parameter('noise_suppression').value,
            self.get_parameter('analog_gain_control').value,
            self.get_parameter('digital_gain_control').value,
            str(self.get_parameter('mic_volume').value)
        )
        self.player = audio_utils.AudioPlayer(self.get_logger(), os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3'))
        self.storage = audio_utils.AudioStorage(self.get_logger(), self.sound_file_directory)

    def goal_callback(self, goal_request):
        if self.audio_sys.is_running:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle): return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.storage.cleanup_files()
        timeout_sec = goal_handle.request.timeout_sec
        feedback_rate = getattr(goal_handle.request, 'feedback_rate', 1.0)
        use_feedback = self.get_parameter('use_feedback').value
        
        if not goal_handle.request.silent_mode:
            self.player.play('start_sound.mp3')

        self.audio_sys.start_recording(chunk_size=self.vad_chunk_size_bytes)
        self.storage.start_write_session("final_output.wav")
        
        if self._stt_model_instance.is_streamable:
            self._stt_model_instance.init_stream()

        start_time = time.time()
        last_audio_time = time.time()
        audio_started = False
        file_counter = 0

        segmenter = None
        if self._stt_model_instance.use_external_vad and VAD_AVAILABLE:
            segmenter = VadSegmenter(
                self.vad_processor, 
                self.get_parameter('min_wipe_duration').value,
                self.get_parameter('max_speech_duration').value,
                self.get_parameter('extra_audio_duration_sec').value
            )

        def process_recognition(wav_path):
            try:
                text = self._stt_model_instance.transcribe(wav_path)
                if text and use_feedback:
                    self.get_logger().info(f"\033[96m[Feedback]\033[0m {text}")
                    fb = SpeechRecognition.Feedback()
                    fb.addition_text = text
                    goal_handle.publish_feedback(fb)
            except Exception as e:
                self.get_logger().error(f"Async Transcribe Error: {e}")
            finally:
                if os.path.exists(wav_path):
                    os.remove(wav_path)

        try:
            while rclpy.ok():
                now = time.time()
                if audio_started:
                    if timeout_sec > 0 and (now - start_time) > timeout_sec: break
                    if (now - last_audio_time) > 5.0: break
                elif (now - start_time) > 10.0: break

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return SpeechRecognition.Result()

                chunk_np = self.audio_sys.read(timeout=0.01)
                if chunk_np is None: continue
                
                if not audio_started:
                    audio_started = True
                    start_time = time.time()

                last_audio_time = now
                self.storage.write_chunk(chunk_np)

                if self._stt_model_instance.is_streamable:
                    text = self._stt_model_instance.put_chunk(chunk_np)
                    if text and use_feedback:
                        self.get_logger().info(f"\033[94m[Stream]\033[0m {text}")
                        fb = SpeechRecognition.Feedback()
                        fb.addition_text = text
                        goal_handle.publish_feedback(fb)
                
                elif segmenter:
                    speech_segment = segmenter.push_chunk(chunk_np, feedback_rate)
                    if speech_segment:
                        file_counter += 1
                        path = os.path.join(self.sound_file_directory, f'feedback_{file_counter}.wav')
                        if self._save_wav(speech_segment, path):
                            self.executor_pool.submit(process_recognition, path)

        finally:
            self.audio_sys.stop()
            self.storage.close_write_session()

        if not goal_handle.request.silent_mode:
            self.player.play('end_sound.mp3')

        final_path = os.path.join(self.sound_file_directory, "final_output.wav")
        final_text = self._stt_model_instance.transcribe(final_path)

        GREEN, ENDC = '\033[92m', '\033[0m'
        self.get_logger().info(f"{GREEN} FINAL RESULT: {final_text}{ENDC}")
        
        goal_handle.succeed()
        return SpeechRecognition.Result(result_text=final_text)

    def _save_wav(self, data_list, path):
        try:
            with wave.open(path, 'wb') as wf:
                wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(16000)
                for c in data_list: wf.writeframes(c.tobytes())
            return True
        except Exception as e:
            self.get_logger().error(f"Save WAV failed: {e}")
            return False

    def destroy_node(self):
        if hasattr(self, 'audio_sys'): self.audio_sys.stop()
        if hasattr(self, 'executor_pool'): self.executor_pool.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    server = STTActionServer()
    executor.add_node(server)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()