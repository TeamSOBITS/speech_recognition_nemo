import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup 

from sobits_interfaces.action import SpeechRecognition 
from ament_index_python.packages import get_package_share_directory

import nemo.collections.asr as nemo_asr # NeMo ASRライブラリ
from playsound import playsound # サウンド再生
import pyaudio # オーディオ入出力
import wave # WAVファイル処理
import os # OS操作（パスなど）
import time # 時間計測
import threading # 非同期処理

class NemoServer(Node):
    # 音声ファイルのパス設定
    SOUND_FILES_PATH = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')
    # 録音済みWAVファイルの出力ディレクトリ
    AUDIO_OUTPUT_DIR = os.path.join(get_package_share_directory('speech_recognition_nemo'), 'sound_file')
    
    def __init__(self):
        super().__init__('nemo_server')
        self.initialized_successfully = False # 初期化成功フラグ

        # --- ROSパラメータ宣言と取得 ---
        self.declare_parameter('model', 'nvidia/parakeet-tdt-0.6b-v2') # NeMoモデル名
        self.declare_parameter('sample_rate', 48000) # サンプリングレート
        self.declare_parameter('chunk_size', 1024) # オーディオチャンクサイズ
        self.declare_parameter('channels', 1) # チャンネル数
        
        # パラメータ値をインスタンス変数に格納
        self.nemo_model_name = self.get_parameter('model').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.audio_format = pyaudio.paInt16 # PyAudioフォーマット

        # --- NeMoモデルのロード ---
        self.get_logger().info(f"NeMoモデルをロード中: {self.nemo_model_name}")
        try:
            self.model = nemo_asr.models.ASRModel.from_pretrained(self.nemo_model_name)
        except Exception as e: # モデルロード失敗時は致命的エラー
            self.get_logger().fatal(f"NeMoモデルのロードに失敗: {e}. ノード初期化を中止します。")
            return # 初期化中止

        # --- WAV出力ディレクトリの準備 ---
        try:
            os.makedirs(self.AUDIO_OUTPUT_DIR, exist_ok=True) # ディレクトリが存在しない場合のみ作成
            self.get_logger().info(f"WAV出力ディレクトリを準備: {self.AUDIO_OUTPUT_DIR}")
        except OSError as e: # ディレクトリ作成失敗時は致命的エラー
            self.get_logger().fatal(f"WAV出力ディレクトリ作成失敗: {e}. ノード初期化を中止します。")
            self.model = None # ファイル保存不可のためモデルも実質無効化
            return 
        
        # --- アクションサーバーの初期化 ---
        self.action_server = ActionServer(
            self, SpeechRecognition, "speech_recognition", # アクション名
            execute_callback=self._execute_speech_to_text, # ゴール実行コールバック
            callback_group=ReentrantCallbackGroup(), # 並列処理用コールバックグループ
            goal_callback=self._goal_callback, # ゴール受付コールバック
            cancel_callback=self._cancel_callback) # キャンセル受付コールバック
        
        self.initialized_successfully = True # 全ての初期化成功
        self.get_logger().warn('NeMo Server is READY and waiting for requests.') # 起動完了ログ

    def _goal_callback(self, goal_request): # ゴールリクエスト受付時の処理
        self.get_logger().info('ゴールリクエストを受信しました。')
        return GoalResponse.ACCEPT # ゴールを受け入れる

    def _cancel_callback(self, goal_handle): # キャンセルリクエスト受付時の処理
        self.get_logger().info('キャンセルリクエストを受信しました。')
        return CancelResponse.ACCEPT # キャンセルを受け入れる
    
    async def _execute_speech_to_text(self, goal_handle): # 音声認識実行のメイン処理
        response = SpeechRecognition.Result()
        audio_interface = None # PyAudioインターフェース
        audio_stream = None # オーディオストリーム

        # --- 事前チェック ---
        if self.model is None or not os.path.isdir(self.AUDIO_OUTPUT_DIR): # モデルまたはディレクトリの準備状況確認
            self.get_logger().error("音声認識のシステム準備ができていません。")
            goal_handle.abort() # ゴールを中止
            response.result_text = "システムエラー"
            return response

        # --- 開始音再生 (非同期) ---
        if not goal_handle.request.silent_mode: # サイレントモードでなければ再生
            threading.Thread(target=self._play_sound, args=('start_sound.mp3',)).start()

        try:
            audio_interface = pyaudio.PyAudio() # PyAudio初期化
            audio_stream = audio_interface.open( # オーディオストリームオープン
                format=self.audio_format, channels=self.channels,
                rate=self.sample_rate, input=True, frames_per_buffer=self.chunk_size)
            self.get_logger().info('録音開始。')

            # --- 録音ループ ---
            frames = []
            start_time = time.time()
            end_time = start_time + goal_handle.request.timeout_sec # タイムアウト秒まで録音
            while time.time() < end_time:
                if goal_handle.is_cancel_requested: # キャンセル要求があれば中断
                    goal_handle.canceled()
                    self.get_logger().info('録音中にキャンセルされました。')
                    response.result_text = ""
                    return response
                frames.append(audio_stream.read(self.chunk_size, exception_on_overflow=False)) # オーディオデータ読み込み
            self.get_logger().info(f"録音終了。時間: {time.time() - start_time:.2f}秒。")

            # --- WAVファイル保存 ---
            wav_path = os.path.join(self.AUDIO_OUTPUT_DIR, 'output.wav') # 出力パス構築
            with wave.open(wav_path, 'wb') as wf: # WAVファイル書き込み
                wf.setnchannels(self.channels)
                wf.setsampwidth(audio_interface.get_sample_size(self.audio_format))
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(frames)) # フレームデータ書き込み
            self.get_logger().info(f'音声をファイルに保存: {wav_path}')

            # --- 文字起こし ---
            self.get_logger().info('文字起こしを開始')
            transcription_start_time = time.time()
            result = self.model.transcribe([wav_path]) # NeMoで文字起こし
            transcribed_text = result[0].text if result else "" # 結果取得
            
            if not transcribed_text:
                self.get_logger().warn("文字起こし結果が空。")
                response.result_text = "音声が認識されませんでした。"
            else:
                self.get_logger().info(f"文字起こしテキスト: {transcribed_text}")
                response.result_text = transcribed_text

            self.get_logger().info(f"文字起こし完了 (時間: {time.time() - transcription_start_time:.2f}秒)。")

            goal_handle.succeed() # ゴール成功
            return response

        except Exception as e: # 処理中の予期せぬエラー
            self.get_logger().error(f"音声認識中にエラー発生: {e}")
            goal_handle.abort() # ゴール中止
            response.result_text = "認識処理中にエラーが発生しました"
            return response
        finally:
            # --- リソース解放 ---
            if audio_stream: # ストリームがあれば閉じる
                try: audio_stream.stop_stream(); audio_stream.close()
                except Exception as e: self.get_logger().warn(f"ストリームクリーンアップエラー: {e}")
            if audio_interface: # インターフェースがあれば終了
                try: audio_interface.terminate()
                except Exception as e: self.get_logger().warn(f"PyAudioクリーンアップエラー: {e}")

            # --- 終了音再生 (非同期) ---
            if not goal_handle.request.silent_mode: # サイレントモードでなければ再生
                threading.Thread(target=self._play_sound, args=('end_sound.mp3',)).start()

    def _play_sound(self, file_name): # サウンド再生ヘルパー関数
        try:
            playsound(os.path.join(self.SOUND_FILES_PATH, file_name))
        except Exception as e:
            self.get_logger().warn(f"サウンドファイル再生失敗 '{file_name}': {e}")

def main(args=None):
    rclpy.init(args=args)
    server = None # serverをNoneで初期化

    try:
        server = NemoServer() # ノードインスタンス作成
        if server.initialized_successfully: # 初期化成功ならスピン
            rclpy.spin(server)
        # 初期化失敗時は__init__でfatalログ出力済み

    except (KeyboardInterrupt, rclpy.utilities.ExternalShutdownException): # 割り込みまたは外部シャットダウン
        if server: # ロガーが利用可能なら
            server.get_logger().info('ノードがシャットダウンされました。')
        else: # ロガー利用不可なら直接出力
            print('ノードがシャットダウンされました。(ロガー利用不可)')
    finally:
        rclpy.shutdown() # ROS 2シャットダウン

if __name__ == '__main__':
    main()