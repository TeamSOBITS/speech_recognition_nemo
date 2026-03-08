import os
import torch
import numpy as np
import nemo.collections.asr as nemo_asr
from .base_engine import BaseEngine

class NemoEngine(BaseEngine):
    def __init__(self, node):
        super().__init__(node)
        self.is_streamable = False
        self.use_external_vad = True
        self._load_model()
        
        self.node.get_logger().info(
            f"[NeMo] Engine Ready. (Model: {self.model_name}, Device: {self.model.device})"
        )

    def _load_model(self):
        self.node.declare_parameter('nemo_model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        self.node.declare_parameter('nemo_device', '')
        
        self.model_name = self.node.get_parameter('nemo_model_name').value
        device_param = self.node.get_parameter('nemo_device').value

        if device_param:
            device = device_param
        else:
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        
        try:
            self.node.get_logger().info(f"[NeMo] Loading model: {self.model_name}")
            self.model = nemo_asr.models.EncDecRNNTBPEModel.from_pretrained(
                model_name=self.model_name
            ).to(torch.device(device))
            self.node.get_logger().info(f"[NeMo] {self.model_name} loaded on: {self.model.device}")
            self.model.eval()
        except Exception as e:
            self.node.get_logger().fatal(f"[NeMo] Model loading failed: {e}")
            raise e

    def transcribe(self, audio_path):
        if not os.path.exists(audio_path):
            return ""

        try:
            with torch.no_grad():
                result = self.model.transcribe([audio_path])
                
                if isinstance(result[0], str):
                    text = result[0]
                else:
                    text = result[0].text if hasattr(result[0], 'text') else str(result[0])
                
                return text.strip()
        except Exception as e:
            self.node.get_logger().error(f"[NeMo] Transcription error: {e}")
            return f"Error: {str(e)}"

    def put_chunk(self, chunk_np):
        return None