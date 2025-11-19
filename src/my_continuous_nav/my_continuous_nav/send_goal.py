import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys # sysモジュールをインポート
import math

class GoToGoalClient(Node):
    def __init__(self):
        super().__init__('go_to_goal_client')
        # Nav2のナビゲーションアクションクライアントを作成
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta_z):
        # 1. ゴールメッセージの作成
        goal_msg = NavigateToPose.Goal()
        
        # 2. 目的地の座標と姿勢を設定
        pose = PoseStamped()
        pose.header.frame_id = 'map' # 地図座標系を使用
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x) # float型にキャスト
        pose.pose.position.y = float(y) # float型にキャスト
        
        # ROS 2ではクォータニオン (x, y, z, w) で姿勢を表現する
        # ここでは単純化のため、z軸周りの回転 (Yaw) のみを設定する
        # (Yaw角をクォータニオンに変換する処理は省略し、簡単のため0.0と1.0を設定)
        # 厳密にはtf_transformationsなどを使って変換が必要です。
        pose.pose.orientation.z = float(theta_z)
        pose.pose.orientation.w = 1.0 

        goal_msg.pose = pose
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal request to x={x}, y={y}, Yaw={math.degrees(theta_z):.2f} degrees...')
        
        # 3. ゴールの送信と応答の処理
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Goal finished with status: {status}')
        # ゴール完了後もノードを維持するためにrclpy.shutdown()は呼び出さない

    def feedback_callback(self, feedback_msg):
        # ナビゲーション中のフィードバック（残りの距離など）を受信する
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')


def main(args=None):
    rclpy.init(args=args)
    
    action_client = GoToGoalClient()

    # --- ここからキーボード入力の処理 ---
    try:
        # キーボードからX座標を受け付ける
        x_input = input("Enter target X coordinate (m): ")
        x = float(x_input)
        
        # キーボードからY座標を受け付ける
        y_input = input("Enter target Y coordinate (m): ")
        y = float(y_input)
        
        # キーボードからZ軸周りの回転角を受け付ける
        theta_degrees_input = input("Enter target Z rotation (degrees): ")
        theta_degrees = float(theta_degrees_input)
        
        # 変換ロジック: degrees() -> radians()
        theta_z_rad = math.radians(theta_degrees)

    except ValueError:
        action_client.get_logger().error("Invalid input. Please enter a valid number.")
        action_client.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
        
    # --- ゴールの送信 ---
    action_client.send_goal(x, y, theta_z_rad) 
    
    # ノードが実行中の間、コールバックを処理し続ける (非同期処理)
    rclpy.spin(action_client)

    # 終了処理
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()