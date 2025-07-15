import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from sobits_interfaces.action import SpeechRecognition
import sys

class SpeechRecognitionClient(Node):
    def __init__(self):
        super().__init__('speech_recognition_client')
        # Initialize the action client
        self._action_client = ActionClient(self, SpeechRecognition, 'speech_recognition')
        
        self.result_text = None  
        self._action_done_future = None # Future object to wait for action completion

    def send_goal(self, timeout_sec: int, feedback_rate: float, silent_mode: bool):
        """
        Sends a speech recognition goal to the action server.

        Args:
            timeout_sec (int): Timeout duration for speech recognition in seconds.
            feedback_rate (float): Frequency of feedback in seconds. Intermediate progress will be sent at this interval.
            silent_mode (bool): If True, start and end sounds will not be played.
        """
        # Reset the Future for each new request
        self._action_done_future = Future() 

        goal_msg = SpeechRecognition.Goal()
        goal_msg.timeout_sec = timeout_sec
        goal_msg.feedback_rate = feedback_rate
        goal_msg.silent_mode = silent_mode

        self.get_logger().info('Waiting for action server...')
        # Wait until the action server is started
        self._action_client.wait_for_server()
        self.get_logger().info('Action server started. Sending goal...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Error while getting goal response: {e}')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False)
            return

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the server.')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False)
            return

        self.get_logger().info('Goal was accepted by the server. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.result_text = result.result_text
            self.get_logger().info(f'Final Result: {self.result_text}')
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(True) # Notify action success
        except Exception as e:
            self.get_logger().error(f'Error while getting result: {e}')
            self.result_text = "Could not recognize due to an error."
            if self._action_done_future and not self._action_done_future.done():
                self._action_done_future.set_result(False) # Notify action failure

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.addition_text}')

def main(args=None):
    rclpy.init(args=args)
    client = SpeechRecognitionClient()

    # Set speech recognition parameters
    timeout_sec = 5  # Timeout duration for speech recognition in seconds
    feedback_rate = 1.5  # Feedback frequency (every X seconds)
    silent_mode = False  # Silent mode setting (True: no sound, False: with sound)

    print("Press any key and Enter to start speech recognition.")
    print("Type 'q' and Enter to exit.")

    try:
        while rclpy.ok():
            # Wait for key input
            user_input = input("Ready. > ")

            if user_input.lower() == 'q':
                print("Exiting.")
                break

            # Send the goal
            client.send_goal(timeout_sec, feedback_rate, silent_mode)

            # Wait until the action is complete
            # Spin until _action_done_future is complete or the node is shut down
            rclpy.spin_until_future_complete(client, client._action_done_future)

            # Output the result after the action is complete
            if client.result_text is not None:
                print(f"\n--- Recognized Text: {client.result_text} ---")
                client.result_text = None # Reset for the next request
            else:
                print("\n--- Speech recognition completed, but no result was obtained. ---")

    except KeyboardInterrupt:
        client.get_logger().info("Shutting down due to keyboard interrupt.")
    except Exception as e:
        client.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        client.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()