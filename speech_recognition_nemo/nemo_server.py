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
import threading
import queue
import time
import os
from collections import deque
import torch
from . import audio_utils

class NemoServer(Node):
    def __init__(self):
        super().__init__('nemo_server')

        self.declare_parameter('model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        self.declare_parameter('min_wipe_duration', 0.2)
        self.declare_parameter('use_feedback', True)        
        self.declare_parameter('extra_audio_duration_sec', 0.2)
        self.declare_parameter('max_speech_duration', 30.0)
        self.declare_parameter("device", "")

        self.declare_parameter('use_echo_cancel', False)
        self.declare_parameter('noise_suppression', False)
        self.declare_parameter('analog_gain_control', False)
        self.declare_parameter('digital_gain_control', False)
        try:
            self.declare_parameter('mic_volume', '')
        except rclpy.exceptions.InvalidParameterTypeException:
            self.declare_parameter('mic_volume', 100)

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.use_feedback_enabled = self.get_parameter('use_feedback').get_parameter_value().bool_value
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.use_echo_cancel = self.get_parameter('use_echo_cancel').get_parameter_value().bool_value
        self.noise_suppression = self.get_parameter('noise_suppression').get_parameter_value().bool_value
        self.analog_gain_control = self.get_parameter('analog_gain_control').get_parameter_value().bool_value
        self.digital_gain_control = self.get_parameter('digital_gain_control').get_parameter_value().bool_value

        param = self.get_parameter('mic_volume')
        mic_volume_raw = str(param.value) if param.value is not None else ""

        if mic_volume_raw.strip() != "" and mic_volume_raw.strip() != "None":
            self.mic_volume = mic_volume_raw if '%' in mic_volume_raw else f"{mic_volume_raw}%"
            self.get_logger().info(f"Setting microphone volume to: {self.mic_volume}")
        else:
            self.mic_volume = ""
            self.get_logger().info("Microphone volume: Using system default (no change).")
    
        self.aec_module_index = None
        self.original_default_sink = None
        self.source_to_modify = None
        self.original_mic_volume = None

        self.SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
        share_dir = get_package_share_directory('speech_recognition_nemo')
        self.sound_file_directory = os.path.join(share_dir, 'sound_file')
        os.makedirs(self.sound_file_directory, exist_ok=True)

        try:
            audio_config = audio_utils.configure_pulseaudio(
                self.get_logger(),
                self.use_echo_cancel,
                self.noise_suppression,
                self.analog_gain_control,
                self.digital_gain_control,
                self.mic_volume
            )
            
            self.source_name = audio_config["source_name"]
            self.sample_rate = audio_config["sample_rate"]
            self.channels = audio_config["channels"]
            self.aec_module_index = audio_config["aec_module_index"]
            self.original_default_sink = audio_config["original_default_sink"]
            self.source_to_modify = audio_config["source_to_modify"]
            self.original_mic_volume = audio_config["original_mic_volume"]
            self.echo_cancel_source = audio_config["echo_cancel_source"]
            self.echo_cancel_sink = audio_config["echo_cancel_sink"]
            self.use_echo_cancel = audio_config["use_echo_cancel"]

            try:
                load_start = time.time()
                if self.device:
                    device = self.device
                else:
                    device = "cuda:0" if torch.cuda.is_available() else "cpu"
                self.model = nemo_asr.models.EncDecRNNTBPEModel.from_pretrained(
                    model_name=self.model_name
                ).to(torch.device(device))
                self.get_logger().info(f"Model '{self.model_name}' loaded on {self.model.device}.")
                
            except Exception as e:
                self.get_logger().fatal(f"Model loading failed: {e}")
                return
            load_end = time.time()
            self.get_logger().info(f"Time to model loading: {load_end - load_start:.2f} sec")
            
            self.vad_processor = None
            if self.use_feedback_enabled:
                from .vad import VadProcessor
                self.vad_processor = VadProcessor(self)
                self.hop_size, self.vad_chunk_size_bytes, self.vad_name = self.vad_processor.get_hop_size()
                self.get_logger().info(f"VAD model loaded.")

            self.action_server = ActionServer(
                self, SpeechRecognition, "speech_recognition",
                execute_callback=self.execute_callback,
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
            )
            YELLOW = '\033[93m'
            ENDC = '\033[0m'
            self.get_logger().info(f"Microphone: {YELLOW}{self.source_name}{ENDC}")
            self.get_logger().info(f"Sample Rate: {self.sample_rate} Hz, Channels: {self.channels}")
            self.get_logger().info(f"{YELLOW}Nemo Server is READY and waiting for requests.{ENDC}")
        except BaseException:
            self.cleanup()
            raise
    
    def goal_callback(self, goal_request):
        self.get_logger().info("Goal received")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        audio_utils.cleanup_wav_files(self.sound_file_directory, self.get_logger())

        timeout_sec = goal_handle.request.timeout_sec
        self.get_logger().info(f"Recording started for {timeout_sec} seconds")
        
        if timeout_sec < 0 and not self.use_feedback_enabled:
            self.get_logger().error("Infinite feedback mode (timeout_sec < 0) requires use_feedback=True.")
            goal_handle.abort()
            return SpeechRecognition.Result(result_text="Error: Invalid configuration (timeout < 0 without feedback)")

        silent = goal_handle.request.silent_mode
        feedback_rate = goal_handle.request.feedback_rate

        audio_q = queue.Queue()
        all_audio_buffer = []
        chunk_size_bytes = self.vad_chunk_size_bytes if self.use_feedback_enabled else 256 * 2 * 1

        if not silent:
            start_sound_thread = threading.Thread(target=audio_utils.play_sound, args=('start_sound.mp3', self.SOUND_FILES_PATH, self.get_logger()))
            start_sound_thread.start()
            start_sound_thread.join()
        proc_container = {"proc": None}

        def capture():
            try:
                proc = subprocess.Popen([
                    'parec', '-d', self.echo_cancel_source if self.use_echo_cancel else self.source_name,
                    '--format=s16le',
                    '--channels', str(self.channels),
                    '--rate', str(self.sample_rate),
                    '--file-format=raw',
                ], stdout=subprocess.PIPE)
                proc_container["proc"] = proc
                
                while True:
                    if not rclpy.ok() or proc.poll() is not None:
                        break
                    chunk = proc.stdout.read(chunk_size_bytes)
                    if not chunk:
                        break
                    audio_q.put(chunk)
            except Exception as e:
                self.get_logger().error(f"Recording error: {e}")
            finally:
                audio_q.put(None)
        thread = threading.Thread(target=capture, daemon = True)
        thread.start()

        start = time.time()
        response = SpeechRecognition.Result()
        recording_stopping = False
        audio_started = False
        
        vad_audio_buffer = np.array([], dtype=np.int16)
        audio_buffer = []
        pre_audio_buffer = None
        post_audio_buffer = None
        is_speaking = False
        file_counter = 0
        min_wipe_duration = 0.0
        max_speech_duration = 30.0

        if self.use_feedback_enabled:
            min_wipe_duration = self.get_parameter('min_wipe_duration').get_parameter_value().double_value
            extra_audio_duration_sec = self.get_parameter('extra_audio_duration_sec').get_parameter_value().double_value
            max_speech_duration = self.get_parameter('max_speech_duration').get_parameter_value().double_value
            extra_audio_buffer_size = int(extra_audio_duration_sec* 16000 / self.hop_size)
            pre_audio_buffer = deque(maxlen=extra_audio_buffer_size)
            post_audio_buffer = deque(maxlen=extra_audio_buffer_size)

            def process_feedback():
                nonlocal file_counter, audio_buffer
                
                duration_sec = len(audio_buffer) * self.hop_size / 16000
                if duration_sec < min_wipe_duration:
                    audio_buffer = []
                    post_audio_buffer.clear()
                    return

                self.get_logger().info("Speech segment ended. Processing feedback.")

                file_counter += 1
                feedback_wav_path = os.path.join(self.sound_file_directory, f'feedback_{file_counter}.wav')
                combined_buffer = audio_buffer + list(post_audio_buffer)

                if audio_utils.save_buffer_to_wav(combined_buffer, feedback_wav_path, 16000, 1, self.get_logger()):
                    try:
                        with torch.no_grad():
                            result = self.model.transcribe(
                                [feedback_wav_path], 
                            )
                            text = result[0].text if hasattr(result[0], 'text') else result[0]
                        feedback = SpeechRecognition.Feedback()
                        if not text:
                            feedback.addition_text = "No speech recognized."
                            self.get_logger().warn("No speech recognized.")   
                        else:
                            safe_text = text.encode('utf-8', errors='ignore').decode('utf-8', errors='ignore')
                            feedback.addition_text = safe_text
                            self.get_logger().info(f"Feedback Result: {safe_text}")
                        goal_handle.publish_feedback(feedback)
                    except Exception as e:
                        self.get_logger().error(f"Feedback error: {e}")
                            
                audio_buffer = []
                post_audio_buffer.clear()

        while rclpy.ok():
            if not recording_stopping:
                now = time.time()
                if audio_started:
                    if timeout_sec > 0 and now - start > timeout_sec:
                        self.get_logger().info("Timeout reached. Stopping recording...")
                        if proc_container["proc"]:
                            proc_container["proc"].terminate()
                        recording_stopping = True
                elif now - start > 10.0:
                    self.get_logger().warn("Timeout waiting for audio stream start.")
                    if proc_container["proc"]:
                        proc_container["proc"].terminate()
                    recording_stopping = True

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                
                if proc_container["proc"]:
                    proc = proc_container["proc"]
                    self.get_logger().info("Terminating recording process due to cancellation...")
                    try:
                        proc.terminate()
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn("Process did not terminate. Killing it...")
                        proc.kill()
                        proc.wait()
                
                goal_handle.canceled()
                return response
            try:
                chunk = audio_q.get(timeout=0.1)
            except queue.Empty:
                continue
            if chunk is None:
                break
            
            if not audio_started:
                audio_started = True
                start = time.time()
                self.get_logger().info("Audio stream started.")
            
            current_audio_np = np.frombuffer(chunk, dtype=np.int16)
            resampled_data = audio_utils.resample_audio(current_audio_np, self.sample_rate, 16000, self.channels)
            
            if timeout_sec >= 0:
                all_audio_buffer.append(resampled_data)

            if self.use_feedback_enabled:
                vad_audio_buffer = np.concatenate([vad_audio_buffer, resampled_data.astype(np.int16)])
                
                if len(vad_audio_buffer) >= self.hop_size:
                    vad_chunk = vad_audio_buffer[:self.hop_size]
                    vad_audio_buffer = vad_audio_buffer[self.hop_size:]

                    is_voice_now = self.vad_processor.vad_processor(vad_chunk, feedback_rate)
                    if is_voice_now:
                        if not is_speaking:
                            audio_buffer.extend(list(pre_audio_buffer))
                            pre_audio_buffer.clear()
                        audio_buffer.append(vad_chunk)
                        is_speaking = True

                        if (len(audio_buffer) * self.hop_size / 16000) > max_speech_duration:
                            self.get_logger().warn(f"Max speech duration ({max_speech_duration}s) exceeded. Forcing feedback.")
                            is_speaking = False
                            process_feedback()
                    else:
                        if is_speaking:
                            post_audio_buffer.append(vad_chunk)  
                            if self.vad_name == "None" or len(post_audio_buffer) == post_audio_buffer.maxlen:
                                is_speaking = False
                                process_feedback()
                        else:
                            pre_audio_buffer.append(vad_chunk)

        if proc_container["proc"]:
            proc = proc_container["proc"]
            try:
                proc.terminate()
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Process did not terminate in time. Killing it...")
                proc.kill() 
                proc.wait() 

        thread.join(timeout=2.0)
        if not silent:
            threading.Thread(target=audio_utils.play_sound, args=('end_sound.mp3', self.SOUND_FILES_PATH, self.get_logger())).start()
        response.result_text = "No audio recorded."

        if timeout_sec < 0:
            response.result_text = "Feedback session ended."

        if all_audio_buffer:
            final_audio_data_int16 = np.concatenate(all_audio_buffer).astype(np.int16)
            final_wav_path = os.path.join(self.sound_file_directory, "final_output.wav")
            if audio_utils.save_buffer_to_wav(final_audio_data_int16, final_wav_path, 16000, 1, self.get_logger()):
                self.get_logger().info(f"Final audio saved to {final_wav_path}")
            else:
                self.get_logger().warn(f"Failed to save final audio to {final_wav_path}")

            if final_audio_data_int16.size > 0:
                try:
                    if self.model is None:
                        raise RuntimeError("Model not loaded")
                    with torch.no_grad():
                        result = self.model.transcribe(
                            [final_wav_path], 
                        )
                        text = result[0].text if hasattr(result[0], 'text') else result[0]

                    if not text:
                        response.result_text = "No speech recognized."
                        self.get_logger().warn("No speech recognized.")
                    else:
                        safe_text = text.encode('utf-8', errors='ignore').decode('utf-8', errors='ignore')
                        response.result_text = safe_text
                        self.get_logger().info(f"Final Result: {safe_text}")
                except torch.cuda.OutOfMemoryError:
                    error_msg = "Error: CUDA Out of Memory"
                    self.get_logger().error(error_msg)
                    response.result_text = error_msg
                except Exception as e:
                    error_msg = str(e)
                    if "out of memory" in error_msg.lower():
                        response.result_text = "Error: VRAM Out of Memory"
                    else:
                        response.result_text = f"Error: {error_msg}"
                    self.get_logger().error(f"Final recognition error: {e}")
        goal_handle.succeed()
        return response

    def cleanup(self):
        self.get_logger().info("Cleaning up PulseAudio settings...")
        
        audio_utils.cleanup_pulse_audio(
            self.get_logger(),
            self.source_to_modify,
            self.mic_volume,
            self.original_mic_volume,
            self.original_default_sink,
            self.aec_module_index
        )

        self.original_mic_volume = None
        self.original_default_sink = None
        self.aec_module_index = None

def main(args=None):
    rclpy.init(args=args)
    nemo_server = None
    try:
        nemo_server = NemoServer()
        executor = MultiThreadedExecutor()
        executor.add_node(nemo_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if nemo_server is not None:
            nemo_server.cleanup()
            nemo_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()