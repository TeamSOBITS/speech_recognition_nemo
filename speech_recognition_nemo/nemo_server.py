import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup 

from sobits_interfaces.action import SpeechRecognition
from ament_index_python.packages import get_package_share_directory

import nemo.collections.asr as nemo_asr

from playsound import playsound
import pyaudio
import wave
import os
import time
import threading # playsound の非同期化のために追加

class NemoServer(Node):
    """
    NeMo を使用した音声認識アクションサーバーのノード。
    ROS 2 アクションを通じて音声認識サービスを提供します。
    """
    def __init__(self):
        super().__init__('nemo_server')

        # パラメータの宣言とデフォルト値の設定
        self.declare_parameter('model', 'nvidia/parakeet-tdt_ctc-0.6b-ja') # 使用する NeMo モデル
        self.declare_parameter('sample_rate',  48000) # サンプリングレート
        self.declare_parameter('chunk_size', 1024) # オーディオチャンクサイズ
        self.declare_parameter('channels', 1) # オーディオチャンネル数
        
        # 宣言したパラメータの値を取得
        # NeMo モデルをロード
        self.model = nemo_asr.models.ASRModel.from_pretrained(self.get_parameter('model').get_parameter_value().string_value)
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.audio_format = pyaudio.paInt16 # PyAudio で使用するオーディオフォーマット

        # パッケージ共有ディレクトリのパスを取得
        self.path = get_package_share_directory('speech_recognition_nemo')
        self.sound_folder_path = os.path.join(get_package_share_directory('sobits_interfaces'), 'mp3')

        # アクションサーバーの初期化
        self.server = ActionServer(
            self,
            SpeechRecognition, # アクションインターフェースの型
            "speech_recognition", # アクション名
            execute_callback=self.speech_to_text, # ゴールが受け入れられたときに実行されるコールバック
            callback_group=ReentrantCallbackGroup(), # 並行処理を可能にするためのコールバックグループ
            goal_callback=self.goal_callback, # ゴールリクエスト受信時のコールバック
            cancel_callback=self.cancel_callback) # キャンセルリクエスト受信時のコールバック
        
        self.get_logger().info('NeMo Server is ready and waiting for service requests.')

    def goal_callback(self, goal_request):
        """
        ゴールリクエストが来たときに呼び出されます。
        ゴールを受け入れるかどうかを決定します。
        """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT # ゴールを受け入れる

    def cancel_callback(self, goal_handle):
        """
        キャンセルリクエストが来たときに呼び出されます。
        キャンセルを受け入れるかどうかを決定します。
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT # キャンセルを受け入れる
    
    async def speech_to_text(self, goal_handle):
        """
        音声認識を実行するメインの非同期コールバック。
        アクションのゴールが受け入れられたときに実行されます。
        フィードバックは返さず、最終的な結果のみを返します。
        """
        response = SpeechRecognition.Result() # 結果メッセージ

        # playsound を非同期で実行するためのヘルパー関数
        def _play_sound_async(file_path):
            playsound(file_path)

        # サイレントモードでない場合、開始音を再生 (非同期)
        if (not goal_handle.request.silent_mode):
            threading.Thread(target=_play_sound_async, 
                             args=(os.path.join(self.sound_folder_path, 'start_sound.mp3'),)).start()

        audio = pyaudio.PyAudio()
        try:
            # オーディオストリームを開く
            stream = audio.open(format=self.audio_format,
                                channels=self.channels,
                                rate=self.sample_rate,
                                input=True, # 入力ストリームとして設定
                                frames_per_buffer=self.chunk_size) # バッファごとのフレーム数
        except Exception as e:
            self.get_logger().error(f"Audio stream error: {e}")
            goal_handle.abort() # ストリームエラーでゴールを中止
            response.result_text = "オーディオストリームエラー"
            return response

        frames = [] # 録音された全オーディオフレームを保存
        
        self.get_logger().info(f"Recording for {goal_handle.request.timeout_sec} seconds...")
        # 録音開始時刻を記録
        recording_start_time = time.time()
        
        # 録音終了予定時刻を計算
        recording_end_target_time = recording_start_time + goal_handle.request.timeout_sec

        # 実際の経過時間に基づいてオーディオを録音
        while time.time() < recording_end_target_time:
            # キャンセルリクエストがあるかチェック
            if goal_handle.is_cancel_requested:
                goal_handle.canceled() # ゴールをキャンセル済みとしてマーク
                self.get_logger().info('Goal canceled')
                response.result_text = "" # 結果を空にする
                return response # 関数を終了

            # PyAudio.Stream.read() から 'timeout' 引数を削除
            data = stream.read(self.chunk_size, exception_on_overflow=False)
            frames.append(data) # 全フレームに追加

        # 録音終了時刻を記録
        recording_end_time = time.time()
        recording_duration = recording_end_time - recording_start_time
        self.get_logger().info(f"Recording finished. Duration: {recording_duration:.2f} seconds (Requested: {goal_handle.request.timeout_sec} seconds).")

        # サイレントモードでない場合、終了音を再生 (非同期)
        if (not goal_handle.request.silent_mode):
            threading.Thread(target=_play_sound_async, 
                             args=(os.path.join(self.sound_folder_path, 'end_sound.mp3'),)).start()

        # オーディオストリームを停止して閉じる
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # 録音されたオーディオをファイルに保存
        output_wav_path = os.path.join(self.path, 'sound_file', 'output.wav')
        self.get_logger().info('Saving audio to: {}'.format(output_wav_path))
        with wave.open(output_wav_path, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(audio.get_sample_size(self.audio_format))
            wf.setframerate(self.sample_rate)
            wf.writeframes(b''.join(frames))

        # --- 文字起こし開始 ---
        self.get_logger().info('Starting transcription.')
        
        start_time = time.time() # 処理開始時刻を記録

        # 録音されたオーディオファイルを NeMo モデルで文字起こし
        # ファイルパスのリストを渡す方式に戻しています
        result = self.model.transcribe([output_wav_path], timestamps=True)

        end_time = time.time() # 処理終了時刻を記録
        elapsed_time = end_time - start_time # 経過時間を計算
        print(f"--- 文字起こしが完了 (処理時間: {elapsed_time:.2f}秒) ---")

        # 文字起こし結果の表示
        print("\n--- 文字起こしされたテキスト ---")
        transcribed_text = result[0].text if result and len(result) > 0 else ""
        print(transcribed_text)

        # セグメントレベルのタイムスタンプ表示 (オプション)
        if hasattr(result[0], 'timestamp') and 'segment' in result[0].timestamp:
            print("\n--- セグメントタイムスタンプ ---")
            segment_timestamps = result[0].timestamp['segment']
            for stamp in segment_timestamps:
                print(f"{stamp['start']:.2f}s - {stamp['end']:.2f}s : {stamp['segment']}")
        else:
            print("\nセグメントタイムスタンプは利用できません。")

        # 結果をアクションのレスポンスに設定
        response.result_text = transcribed_text
        goal_handle.succeed() # ゴールを成功としてマーク
        return response

def main(args=None):
    try:
        # ROS 2 Python クライアントライブラリを初期化
        rclpy.init(args=args)
        # NemoServer ノードのインスタンスを作成
        server = NemoServer()
        # ノードをスピンさせて、コールバックが処理されるようにする
        rclpy.spin(server)
        
    # キーボードからの割り込み (Ctrl+C) または外部シャットダウン時に終了
    except (KeyboardInterrupt, rclpy.utilities.ExternalShutdownException):
        pass # エラーメッセージを表示せずにクリーンに終了
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()