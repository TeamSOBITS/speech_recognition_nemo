import numpy as np
import logging
import time
from collections import deque
from rclpy.node import Node

try:
    from ten_vad import TenVad
except ImportError:
    TenVad = None

logger = logging.getLogger(__name__)

class VadProcessor:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        self.vad_name = self.node.get_parameter_or('vad_name', 'ten_vad').get_parameter_value().string_value
        self.hop_size = self.node.get_parameter_or('hop_size', 256).get_parameter_value().integer_value
        self.threshold = self.node.get_parameter_or('threshold', 0.5).get_parameter_value().double_value

        if not self.node.has_parameter('vad_name'):
            self.vad_name = 'ten_vad'
            self.hop_size = 256
            self.threshold = 0.5

        self.vad_chunk_size_bytes = 2 * 1 * self.hop_size
        self.vad_model = None
        self.last_false_time = time.time()

        if self.vad_name == 'ten_vad' and TenVad is not None:
            try:
                self.vad_model = TenVad(self.hop_size, self.threshold)
                self.logger.info("VAD model 'ten_vad' loaded.")
            except Exception as e:
                self.logger.warn(f"Failed to load TenVad: {e}")
                self.vad_model = None
        else:
            if self.vad_name != 'ten_vad':
                self.logger.warn(f"Unknown VAD name: {self.vad_name}. Falling back to time-based logic.")
            elif TenVad is None:
                self.logger.warn("TenVad module not found. Falling back to time-based logic.")
            self.vad_model = None

    def get_specs(self):
        return self.hop_size, self.vad_chunk_size_bytes, self.vad_name
    
    def is_speech(self, resampled_data_bytes, feedback_rate=1.0):
        if self.vad_model:
            audio_np = np.frombuffer(resampled_data_bytes, dtype=np.int16)
            if audio_np.shape[0] < self.hop_size:
                return False
            process_chunk = audio_np[:self.hop_size]
            prob, _ = self.vad_model.process(process_chunk)
            is_voice = prob > self.threshold
            status = "ON " if is_voice else "Off"
            # self.logger.info(f"Voice: {status}, P:{prob:.2f}")
            return is_voice
        else:
            current_time = time.time()
            if current_time - self.last_false_time >= feedback_rate:
                self.last_false_time = current_time
                return False
            else:
                return True


class VadSegmenter:
    def __init__(self, vad_processor: VadProcessor, min_duration_sec, max_duration_sec, extra_duration_sec):
        self.vad_processor = vad_processor
        self.hop_size, self.chunk_size_bytes, self.vad_name = self.vad_processor.get_specs()
        
        self.min_wipe_samples = int(min_duration_sec * 16000)
        self.max_speech_samples = int(max_duration_sec * 16000)
        
        extra_chunks = int(extra_duration_sec * 16000 / self.hop_size)
        self.pre_buffer = deque(maxlen=extra_chunks)
        self.post_buffer = deque(maxlen=extra_chunks)
        
        self.current_speech_buffer = [] 
        self.vad_accum_buffer = np.array([], dtype=np.int16)
        
        self.is_speaking = False

    def push_chunk(self, raw_audio_chunk: np.ndarray, feedback_rate: float):
        result_segment = None
        self.vad_accum_buffer = np.concatenate([self.vad_accum_buffer, raw_audio_chunk])
        
        while len(self.vad_accum_buffer) >= self.hop_size:
            chunk_np = self.vad_accum_buffer[:self.hop_size]
            self.vad_accum_buffer = self.vad_accum_buffer[self.hop_size:]
            
            is_voice = self.vad_processor.is_speech(chunk_np.tobytes(), feedback_rate)
            if is_voice:
                if not self.is_speaking:
                    self.current_speech_buffer.extend(list(self.pre_buffer))
                    self.pre_buffer.clear()
                    self.is_speaking = True
                
                self.current_speech_buffer.append(chunk_np)
                
                current_len_samples = len(self.current_speech_buffer) * self.hop_size
                if current_len_samples > self.max_speech_samples:
                    result_segment = self._finalize_segment()
                    self.is_speaking = False 
            
            else:
                if self.is_speaking:
                    self.post_buffer.append(chunk_np)
                    
                    if self.vad_name == "None" or len(self.post_buffer) == self.post_buffer.maxlen:
                        result_segment = self._finalize_segment()
                        self.is_speaking = False
                else:
                    self.pre_buffer.append(chunk_np)
        
        return result_segment

    def _finalize_segment(self):
        duration_samples = len(self.current_speech_buffer) * self.hop_size
        if duration_samples < self.min_wipe_samples:
            self.current_speech_buffer = []
            self.post_buffer.clear()
            return None
        
        full_segment = self.current_speech_buffer + list(self.post_buffer)
        
        self.current_speech_buffer = []
        self.post_buffer.clear()
        self.pre_buffer.clear()
        
        return full_segment