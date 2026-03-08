from abc import ABC, abstractmethod

class BaseEngine(ABC):
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        self.is_streamable = False
        self.use_external_vad = True

    @abstractmethod
    def _load_model(self):
        pass

    @abstractmethod
    def transcribe(self, audio_path):
        pass

    def init_stream(self):
        pass

    def put_chunk(self, chunk_np):
        return None