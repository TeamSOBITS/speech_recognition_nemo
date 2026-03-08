import subprocess
import re
import os
import numpy as np
import wave
import threading
import queue
import time
import shutil
import rclpy
import glob

class AudioSystem:
    def __init__(self, logger, use_echo_cancel=False, noise_suppression=False, 
                 analog_gain=False, digital_gain=False, mic_volume="100%", sample_rate=16000):
        self.logger = logger
        self.use_echo_cancel = use_echo_cancel
        self.noise_suppression = noise_suppression
        self.analog_gain = analog_gain
        self.digital_gain = digital_gain
        self.target_mic_volume = mic_volume
        self.sample_rate = sample_rate

        self.audio_q = queue.Queue()
        self.is_running = False
        self.capture_proc = None
        
        self.aec_module_index = None
        self.original_default_sink = None
        self.original_mic_volume = None
        self.source_to_modify = None
        self.active_source = None

        self._initialize_pulseaudio()

    def _initialize_pulseaudio(self):
        try:
            info = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True).stdout
            default_source = next((l.split(':', 1)[1].strip() for l in info.splitlines() if "Default Source" in l or "デフォルトソース" in l), None)
            default_sink = next((l.split(':', 1)[1].strip() for l in info.splitlines() if "Default Sink" in l or "デフォルトシンク" in l), None)
            
            self.original_default_sink = default_sink
            self.original_mic_volume = self._get_source_volume(default_source)

            if self.target_mic_volume is not None:
                v_str = str(self.target_mic_volume).strip()
                if v_str.isdigit():
                    self.target_mic_volume = v_str + "%"
                else:
                    self.target_mic_volume = v_str

            def status_icon(flag): return "\033[92m[ON]\033[0m" if flag else "\033[31m[OFF]\033[0m"
            
            self.logger.info("="*50)
            self.logger.info(" Audio System Configuration Status:")
            self.logger.info(f"  - Echo Cancellation:  {status_icon(self.use_echo_cancel)}")
            self.logger.info(f"  - Noise Suppression:  {status_icon(self.noise_suppression)}")
            self.logger.info(f"  - Analog Gain Ctrl:   {status_icon(self.analog_gain)}")
            self.logger.info(f"  - Digital Gain Ctrl:  {status_icon(self.digital_gain)}")
            self.logger.info(f"  - Mic Volume:         {self.original_mic_volume} -> {self.target_mic_volume or 'Keep'}")
            self.logger.info(f"  - Sample Rate:        {self.sample_rate} Hz")
            self.logger.info("="*50)

            if any([self.use_echo_cancel, self.noise_suppression, self.analog_gain, self.digital_gain]):
                self.logger.info(f"Loading module-echo-cancel (Master Sink: {default_sink})")
                
                aec_args = (f"noise_suppression={int(self.noise_suppression)} "
                            f"analog_gain_control={int(self.analog_gain)} "
                            f"digital_gain_control={int(self.digital_gain)}")
                
                subprocess.run(["pactl", "unload-module", "module-echo-cancel"], stderr=subprocess.DEVNULL)
                
                cmd = [
                    'pactl', 'load-module', 'module-echo-cancel',
                    f'source_master={default_source}',
                    f'sink_master={default_sink}',
                    'source_name=mic_aec',
                    'sink_name=speaker_aec',
                    'aec_method=webrtc',
                    f'aec_args="{aec_args}"'
                ]

                res = subprocess.run(cmd, capture_output=True, text=True, check=True)
                self.aec_module_index = int(res.stdout.strip())
                
                subprocess.run(['pactl', 'set-default-sink', 'speaker_aec'], check=True)
                self.active_source = "mic_aec"
            else:
                self.active_source = default_source

            self.source_to_modify = self.active_source
            
            if self.target_mic_volume:
                subprocess.run(['pactl', 'set-source-volume', self.source_to_modify, self.target_mic_volume], check=True)
                new_vol = self._get_source_volume(self.source_to_modify)
                self.logger.info(f"\033[94m[AudioSystem] Volume updated: {new_vol}\033[0m")
            
            self.logger.info(f"\033[92m[AudioSystem] Active Source: {self.active_source}\033[0m")

        except Exception as e:
            self.logger.error(f"PulseAudio config error: {e}")
            self.active_source = "@DEFAULT_SOURCE@"

    def _get_source_volume(self, source_name):
        try:
            res = subprocess.run(['pactl', 'get-source-volume', source_name], capture_output=True, text=True).stdout
            match = re.search(r'(\d+)%', res)
            return match.group(0) if match else "100%"
        except: return "100%"

    def start_recording(self, chunk_size=3200):
        if self.is_running: return
        self.is_running = True
        time.sleep(0.2)

        def _capture_loop():
            cmd = [
                'parec', '-d', self.active_source, 
                '--format=s16le', '--channels=1', 
                f'--rate={self.sample_rate}', 
                '--file-format=raw', '--latency-msec=40'
            ]
            try:
                self.capture_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                while self.is_running and rclpy.ok():
                    chunk = self.capture_proc.stdout.read(chunk_size)
                    if not chunk: break
                    self.audio_q.put(np.frombuffer(chunk, dtype=np.int16))
            except Exception as e:
                self.logger.error(f"Capture error: {e}")
            finally:
                self.is_running = False

        threading.Thread(target=_capture_loop, daemon=True).start()
        self.logger.info(f"\033[94m[AudioSystem] Recording started: {self.active_source} at {self.sample_rate}Hz\033[0m")

    def read(self, timeout=0.1):
        try: return self.audio_q.get(timeout=timeout)
        except queue.Empty: return None

    def stop(self):
        self.is_running = False
        if self.capture_proc:
            self.capture_proc.terminate()
            self.capture_proc = None
        
        if self.source_to_modify and self.original_mic_volume:
            subprocess.run(['pactl', 'set-source-volume', self.source_to_modify, self.original_mic_volume], stderr=subprocess.DEVNULL)
        
        if self.original_default_sink:
            subprocess.run(['pactl', 'set-default-sink', self.original_default_sink], stderr=subprocess.DEVNULL)

        if self.aec_module_index:
            subprocess.run(['pactl', 'unload-module', str(self.aec_module_index)], stderr=subprocess.DEVNULL)
            self.aec_module_index = None
        self.logger.info("[AudioSystem] Stopped.")

class AudioPlayer:
    def __init__(self, logger, sound_files_path):
        self.logger = logger
        self.path = sound_files_path

    def play(self, filename):
        full_path = os.path.join(self.path, filename)
        if not os.path.exists(full_path): return
        player = "paplay" if shutil.which("paplay") else "ffplay -nodisp -autoexit -loglevel quiet"
        try: subprocess.run(f"{player} {full_path}", shell=True)
        except: pass

class AudioStorage:
    def __init__(self, logger, directory):
        self.logger = logger
        self.dir = directory
        if not os.path.exists(self.dir): os.makedirs(self.dir)
        self._current_wf = None

    def cleanup_files(self, pattern="*.wav"):
        files = glob.glob(os.path.join(self.dir, pattern))
        for f in files:
            try: os.remove(f)
            except: pass

    def start_write_session(self, filename, sample_rate=16000):
        if self._current_wf: self.close_write_session()
        self._current_wf = wave.open(os.path.join(self.dir, filename), 'wb')
        self._current_wf.setnchannels(1)
        self._current_wf.setsampwidth(2)
        self._current_wf.setframerate(sample_rate)

    def write_chunk(self, data):
        if self._current_wf and data is not None:
            self._current_wf.writeframes(data.tobytes() if isinstance(data, np.ndarray) else data)

    def close_write_session(self):
        if self._current_wf:
            self._current_wf.close()
            self._current_wf = None