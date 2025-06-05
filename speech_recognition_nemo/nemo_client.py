import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from sobits_interfaces.action import SpeechRecognition
import sys

class SpeechRecognitionClient(Node):
    def __init__(self):
        super().__init__('speech_recognition_client')
        # アクションクライアントを初期化
        self._action_client = ActionClient(self, SpeechRecognition, 'speech_recognition')
        
        self.result_text = None  
        self._action_done_future = None # アクションの完了を待つためのFutureオブジェクト

    def send_goal(self, timeout_sec: int, feedback_rate: float, silent_mode: bool):
        """
        音声認識ゴールをアクションサーバーに送信します。

        Args:
            timeout_sec (int): 音声認識のタイムアウト時間（秒）。
            feedback_rate (float): フィードバックの頻度（秒）。この間隔で途中経過が送信されます。
            silent_mode (bool): Trueの場合、開始音と終了音を再生しません。
        """
        # 新しいリクエストごとにFutureをリセット
        self._action_done_future = Future() 

        goal_msg = SpeechRecognition.Goal()
        goal_msg.timeout_sec = timeout_sec
        goal_msg.feedback_rate = feedback_rate
        goal_msg.silent_mode = silent_mode

        self.get_logger().info('アクションサーバーを待機中...')
        # アクションサーバーが起動するまで待機
        self._action_client.wait_for_server()
        self.get_logger().info('アクションサーバーが起動しました。ゴールを送信中...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'ゴール応答の取得中にエラーが発生しました: {e}')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False)
            return

        if not goal_handle.accepted:
            self.get_logger().info('ゴールがサーバーによって拒否されました。')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False)
            return

        self.get_logger().info('ゴールがサーバーによって受け入れられました。結果を待機中...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.result_text = result.result_text
            self.get_logger().info(f'最終結果: {self.result_text}')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(True) # アクション成功を通知
        except Exception as e:
            self.get_logger().error(f'結果の取得中にエラーが発生しました: {e}')
            self.result_text = "エラーにより認識できませんでした。"
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False) # アクション失敗を通知

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'フィードバック: {feedback.addition_text}')

def main(args=None):
    rclpy.init(args=args)
    client = SpeechRecognitionClient()

    # 音声認識のパラメータを設定
    timeout_sec = 5  # 音声認識のタイムアウト時間（秒）
    feedback_rate = 0.5  # フィードバックの頻度（秒ごと）
    silent_mode = False  # サイレントモードの設定（True: 音声なし, False: 音声あり）

    print("何かキーを押してEnterで音声認識を開始")
    print("'q' と入力してEnterで終了します。")

    try:
        while rclpy.ok():
            # キー入力を待機
            user_input = input("準備ができました。> ")

            if user_input.lower() == 'q':
                print("終了します。")
                break

            # ゴールを送信
            client.send_goal(timeout_sec, feedback_rate, silent_mode)

            # アクションが完了するまで待機
            # _action_done_future が完了するか、ノードがシャットダウンされるまでスピン
            rclpy.spin_until_future_complete(client, client._action_done_future)

            # アクションが完了した後に結果を出力
            if client.result_text is not None:
                print(f"\n--- 認識されたテキスト: {client.result_text} ---")
                client.result_text = None # 次のリクエストのためにリセット
            else:
                print("\n--- 音声認識は完了しましたが、結果は得られませんでした。 ---")

    except KeyboardInterrupt:
        client.get_logger().info("キーボード割り込みによりシャットダウンします。")
    except Exception as e:
        client.get_logger().error(f"予期せぬエラーが発生しました: {e}")
    finally:
        client.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
if __name__ == '__main__':
    main()