import numpy as np
import logging
import time
from rclpy.node import Node
from ten_vad import TenVad

logger = logging.getLogger(__name__)

class VadProcessor(Node):
    def __init__(self, node: Node):
        super().__init__('vad_processor_node') 
        self.node = node

        self.declare_parameter('vad_name', 'ten_vad')
        self.declare_parameter('hop_size', 256)
        self.declare_parameter('threshold', 0.5)
        
        self.vad_name = self.get_parameter('vad_name').get_parameter_value().string_value
        self.hop_size = self.get_parameter('hop_size').get_parameter_value().integer_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        
        self.vad_model = None
        self.last_false_time = time.time()

        if self.vad_name == 'ten_vad':
            self.vad_model = TenVad(self.hop_size, self.threshold)
            self.get_logger().info("VAD model 'ten_vad' loaded.")
        else:
            self.get_logger().warn(f"Unknown VAD name: {self.vad_name}. Falling back to time-based VAD logic.")
            self.vad_model = None

    def get_hop_size(self):
        self.hop_size = self.get_parameter('hop_size').get_parameter_value().integer_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.vad_chunk_size_bytes = 2 * 1 * self.hop_size         
        return self.hop_size, self.vad_chunk_size_bytes
    
    def vad_processor(self, resampled_data):
        if self.vad_model:
            audio_np = np.frombuffer(resampled_data, dtype=np.int16)
            prob, _ = self.vad_model.process(audio_np)
            is_voice = prob > self.threshold
            self.get_logger().info(f"{self.vad_name} Status: Speaking {is_voice} (Probability: {prob:.2f})")
            return is_voice
        else:
            current_time = time.time()
            if current_time - self.last_false_time >= 0.1: 
                self.last_false_time = current_time
                self.get_logger().info(f"Not recognizing (time-based)")
                return False
            else:
                self.get_logger().info("Recognizing (time-based)")
                return True