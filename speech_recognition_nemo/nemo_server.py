# ROS 2 のクライアントライブラリをインポート
import rclpy
from rclpy.node import Node
# ActionServer, ゴールとキャンセルに対するレスポンスをインポート
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 複数のコールバックを同時に処理するためのコールバックグループをインポート
from rclpy.callback_groups import ReentrantCallbackGroup
# マルチスレッドでのノード実行を可能にするエグゼキュータをインポート
from rclpy.executors import MultiThreadedExecutor
# 定義されたアクションインターフェースをインポート
from sobits_interfaces.action import SpeechRecognition

# ROSパッケージの共有ディレクトリを取得するユーティリティをインポート
from ament_index_python.packages import get_package_share_directory

# NeMoの自動音声認識（ASR）コレクションをインポート
import nemo.collections.asr as nemo_asr
# 外部プロセスを起動するためのライブラリをインポート
import subprocess
# 数値計算のためのNumPyライブラリをインポート
import numpy as np
# PyTorchライブラリをインポート
import torch
# スレッドを扱うためのライブラリをインポート
import threading
# スレッドセーフなキューを扱うためのライブラリをインポート
import queue
# 時間を扱うためのライブラリをインポート
import time
# WAVファイルを扱うためのライブラリをインポート
import wave
# OS関連の操作を行うライブラリをインポート
import os
# 正規表現を扱うためのライブラリをインポート
import re
# ファイルパスのパターンマッチングを行うライブラリをインポート
import glob
# 音声信号のリサンプリングを行うライブラリをインポート
from scipy.signal import resample_poly 

# 同じディレクトリにあるVAD（音声活動検出）モジュールをインポート
from .vad import VadProcessor

# NeMoサーバーノードを定義
class NemoServer(Node):
    def __init__(self):
        # ノード名を'nemo_server'として初期化
        super().__init__('nemo_server')

        # PulseAudioの出力からソース名を取得する正規表現をコンパイル
        self.PULSEAUDIO_SOURCE_NAME_PATTERN = re.compile(r'^\s*(?:Name|名前):\s*(.+)\s*$')
        # PulseAudioの出力からサンプルレートとチャンネル数を取得する正規表現をコンパイル
        self.PULSEAUDIO_SAMPLE_SPEC_PATTERN = re.compile(r'^\s*(?:Sample Specification|サンプル仕様):\s*(\S+)\s+(\d+)ch\s+(\d+)Hz')

        # 'model_name'パラメータを宣言し、デフォルト値を設定
        self.declare_parameter('model_name', 'nvidia/parakeet-tdt-0.6b-v2')
        # パラメータの値を取得
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        # 'min_wipe_duration'パラメータを宣言
        self.declare_parameter('min_wipe_duration', 0.2)

        # 共有ディレクトリにある音声ファイルへのパスを構築
        self.SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
        # 現在のパッケージの共有ディレクトリを取得
        share_dir = get_package_share_directory('speech_recognition_nemo')
        # 音声ファイルを保存するディレクトリのパスを構築
        self.sound_file_directory = os.path.join(os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..')),
                                                 'src', 'speech_recognition_nemo', 'sound_file')
        # 音声ファイルを保存するディレクトリが存在しない場合は作成
        os.makedirs(self.sound_file_directory, exist_ok=True)
        # 最終的な音声ファイルの名前を定義
        self.wav_path = os.path.join(self.sound_file_directory, 'output.wav')
        # 出力パスをログに記録
        self.get_logger().info(f"Output path: {self.wav_path}")
        # 連番のファイル名のためのカウンターを初期化
        self.file_counter = 0 

        # PulseAudioのソース情報を取得
        source_name, sample_rate, channels = self.get_pulseaudio_source_info()

        # ソース情報の取得に失敗した場合のフォールバック設定
        if source_name is None:
            self.get_logger().warn("Failed to get default microphone. Using fallback settings.")
            self.source_name = "default"  # デフォルトのソース名を指定
            self.sample_rate = 44100      # デフォルトのサンプルレート
            self.channels = 2             # デフォルトのチャンネル数
        else:
            # 取得したソース情報を設定
            self.source_name = source_name
            self.sample_rate = sample_rate if sample_rate is not None else 44100
            self.channels = channels if channels is not None else 2
        
        # 最終的な設定をログに記録
        self.get_logger().info(f"Microphone: {self.source_name}, Sample rate: {self.sample_rate} Hz, Channels: {self.channels}")

        try:
            # モデルのロードを開始することをログに記録
            self.get_logger().info(f"Loading model: {self.model_name}")
            # NeMoモデルを事前学習済みモデルからロードし、GPUまたはCPUに配置
            self.model = nemo_asr.models.EncDecRNNTBPEModel.from_pretrained(
                model_name=self.model_name
            ).to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))
        except Exception as e:
            # モデルのロードが失敗した場合、致命的なエラーをログに記録
            self.get_logger().fatal(f"Model loading failed: {e}")
            return

        # VADプロセッサーを初期化
        self.vad_processor = VadProcessor(self)
        # VADのホップサイズとチャンクサイズ（バイト単位）を取得
        self.hop_size, self.vad_chunk_size_bytes = self.vad_processor.get_hop_size()

        # ActionServerを初期化
        self.action_server = ActionServer(
            self,
            SpeechRecognition,
            "speech_recognition",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # ログメッセージの色を設定
        YELLOW = '\033[93m'
        ENDC = '\033[0m'
        # 最終的な設定情報を色付きでログに記録
        self.get_logger().info(f"Microphone: {YELLOW}{self.source_name}{ENDC}")
        self.get_logger().info(f"Sample Rate: {self.sample_rate} Hz, Channels: {self.channels}")
        self.get_logger().info(f"{YELLOW}NeMo Server is READY and waiting for requests.{ENDC}")

        # parecプロセスを保持する変数を初期化
        self.parec_proc = None

    def get_pulseaudio_source_info(self):
        try:
            # pactl infoコマンドを実行してデフォルトソースを取得
            info = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True)
            default_source = None
            for line in info.stdout.splitlines():
                if "Default Source:" in line or "デフォルトソース:" in line:
                    default_source = line.split(':', 1)[1].strip()
                    break
            if not default_source:
                self.get_logger().warn("Default PulseAudio source not found.")
                return None, None, None

            # pactl list sourcesコマンドを実行して詳細なソース情報を取得
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

            # デフォルトソースに対応するブロックを見つけ、サンプルレートとチャンネルを解析
            for block in blocks:
                for line in block:
                    m = self.PULSEAUDIO_SOURCE_NAME_PATTERN.match(line)
                    if m and m.group(1).strip() == default_source:
                        rate, channels = self.parse_sample_rate_and_channels(block)
                        return default_source, rate, channels

            self.get_logger().warn(f"Could not find detailed info for source '{default_source}'.")
            return default_source, None, None

        except Exception as e:
            # コマンド実行中にエラーが発生した場合
            self.get_logger().error(f"PulseAudio source info error: {e}")
            return None, None, None

    def parse_sample_rate_and_channels(self, lines):
        # PulseAudioの出力からサンプルレートとチャンネル数を解析
        for line in lines:
            m = self.PULSEAUDIO_SAMPLE_SPEC_PATTERN.match(line)
            if m:
                return int(m.group(3)), int(m.group(2))
        return None, None

    def play_sound(self, filename):
        # ffplayを使用して、指定された音声ファイルを再生
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
        # 録音された音声をターゲットのサンプルレートにリサンプリング
        if channels > 1:
            audio_np = audio_np.reshape(-1, channels)
            audio_np = audio_np.mean(axis=1)
        return resample_poly(audio_np, target_sr, orig_sr)

    def goal_callback(self, goal_request):
        # アクションのゴールリクエストを受信した際のコールバック
        self.get_logger().info("Goal received")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # アクションのキャンセルリクエストを受信した際のコールバック
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # リクエストからタイムアウト時間とサイレントモードのフラグを取得
        timeout_sec = goal_handle.request.timeout_sec
        silent = goal_handle.request.silent_mode
        self.min_wipe_duration = self.get_parameter('min_wipe_duration').get_parameter_value().double_value


        self.get_logger().info(f"Recording started for {timeout_sec} seconds")
        if not silent:
            start_sound_thread = threading.Thread(target=self.play_sound, args=('start_sound.mp3',), daemon=True)
            start_sound_thread.start()

        self.file_counter = 0 
        self._cleanup_files()
        
        # 音声データチャンクを格納するスレッドセーフなキューを初期化
        audio_q = queue.Queue()
        # VADおよびフィードバック用のオーディオバッファを初期化
        audio_buffer = []
        
        # VADのホップサイズに基づいてチャンクサイズを設定
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
        
        vad_audio_buffer = np.array([], dtype=np.int16)
        
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
            
            current_audio_np = np.frombuffer(chunk, dtype=np.int16)
            resampled_data = self.resample_audio(current_audio_np, self.sample_rate, 16000, self.channels)
            
            vad_audio_buffer = np.concatenate([vad_audio_buffer, resampled_data.astype(np.int16)])
            
            if len(vad_audio_buffer) >= self.hop_size:
                vad_chunk = vad_audio_buffer[:self.hop_size]
                vad_audio_buffer = vad_audio_buffer[self.hop_size:]

                is_voice_now = self.vad_processor.vad_processor(vad_chunk.tobytes())
            
                if is_voice_now:
                    if not is_potential_speaking:
                        is_potential_speaking = True
                        potential_speech_start_time = now
                    # 音声が検出されたらバッファに追加
                    audio_buffer.append(vad_chunk)
                else:
                    if is_potential_speaking:
                        is_potential_speaking = False

                        # バッファに音声データが一定時間分溜まっているか確認
                        duration_sec = len(audio_buffer) * self.hop_size / 16000
                        if duration_sec < self.min_wipe_duration:
                            audio_buffer = []
                            continue
                        
                        self.get_logger().info("Speech segment ended. Processing feedback.")

                        # ファイルに保存
                        self.file_counter += 1
                        feedback_wav_path = os.path.join(self.sound_file_directory, f'feedback_{self.file_counter}.wav')
                        
                        # バッファの内容をWAVファイルとして保存
                        if self._save_buffer_to_wav([b''.join(audio_buffer)], feedback_wav_path, 16000, 1): # VADでモノラル化しているためチャンネルは1
                            try:
                                # WAVファイルを直接 NeMo モデルに渡す
                                with torch.no_grad():
                                    feedback_result = self.model.transcribe([feedback_wav_path])
                                feedback_text = feedback_result[0].text if feedback_result and feedback_result[0].text.strip() else "No speech recognized."
                                self.get_logger().info(f"Feedback Result: {feedback_text}")
                                feedback = SpeechRecognition.Feedback()
                                feedback.addition_text = feedback_text
                                goal_handle.publish_feedback(feedback)
                            except Exception as e:
                                self.get_logger().error(f"Feedback error: {e}")
                        
                        audio_buffer = []
                      
        # 録音ループ終了
        if self.parec_proc:
            self.parec_proc.terminate()
        thread.join(timeout=2.0)
        
        response.result_text = "No audio recorded."
        
        # 最終的な音声ファイルと認識処理
        if audio_buffer:
            # 最後のセグメントの音声認識を実行
            final_audio_data = np.concatenate(audio_buffer).astype(np.float32) / 32768.0
            
            if final_audio_data.size > 0:
                with torch.no_grad():
                    result = self.model.transcribe([final_audio_data])
                response.result_text = result[0].text if result and result[0].text.strip() else "No speech recognized."
                self.get_logger().info(f"Final Result: {response.result_text}, Score: {result[0].score:.2f}")

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
            # framesが既に結合されているバイト列を想定
            combined_frames = frames[0] if isinstance(frames, list) and len(frames) == 1 else b"".join(frames)

            with wave.open(file_path, 'wb') as wf:
                wf.setnchannels(channels)
                wf.setsampwidth(2) # 16bit = 2 bytes
                wf.setframerate(sample_rate)
                wf.writeframes(combined_frames)
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