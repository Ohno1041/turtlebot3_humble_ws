import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import random
import math
import time

class ContinuousNavigator(Node):
    """TurtleBot3に連続的にナビゲーション目標を送信するROS 2ノード"""
    def __init__(self):
        super().__init__('continuous_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Action client created. Waiting for server...')

        # 標準ワールド内の到達可能なランダムな目標地点のリスト
        # turtlebot3_worldで到達可能な座標（向きも含む）
        self.target_poses = [
            (0.5, 0.5, 0.0),   # x, y, yaw (目標地点の向き)
            (-0.5, 0.5, math.pi / 2),
            (0.5, -0.5, -math.pi / 2),
            (-0.5, -0.5, math.pi)
        ]
        self.current_target_index = 0
        self.future = None
        self.is_navigating = False  # ナビゲーション状態フラグ
        self.next_goal_timer = None  # タイマーの参照を保持

        # アクションサーバーが利用可能になるまで待機
        self._action_client.wait_for_server()
        self.get_logger().info('Navigation Action Server is ready.')
        
        # 最初の目標地点を送信
        self.send_next_goal()

    def send_next_goal(self):
        """次の目標地点を生成し、Nav2に送信する"""
        if not self.target_poses:
            self.get_logger().warn('Target list is empty. Stopping.')
            return

        # 既にナビゲーション中の場合は無視
        if self.is_navigating:
            self.get_logger().debug('Already navigating, skipping new goal')
            return

        # 次のタイマーをキャンセル
        if self.next_goal_timer is not None:
            self.next_goal_timer.cancel()
            self.next_goal_timer = None

        # ターゲットを循環的に選択
        x, y, yaw = self.target_poses[self.current_target_index]
        self.current_target_index = (self.current_target_index + 1) % len(self.target_poses)

        self.is_navigating = True
        self.send_goal(x, y, yaw)

    def send_goal(self, x, y, yaw):
        """指定された座標にナビゲーションゴールを送信"""
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置 (x, y)
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # 向き (Quaternionに変換)
        # Z軸周りの回転（yaw）を四元数に変換
        # q = [w, x, y, z] = [cos(yaw/2), 0, 0, sin(yaw/2)]
        half_yaw = yaw * 0.5
        goal_pose.pose.orientation.w = math.cos(half_yaw)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(half_yaw)

        goal_msg.pose = goal_pose

        # デバッグ: 四元数の値を確認
        self.get_logger().info(f'Sending goal to ({x:.2f}, {y:.2f}) with yaw {yaw:.2f} rad ({math.degrees(yaw):.1f}°)')
        self.get_logger().info(f'Quaternion: w={goal_pose.pose.orientation.w:.3f}, x={goal_pose.pose.orientation.x:.3f}, y={goal_pose.pose.orientation.y:.3f}, z={goal_pose.pose.orientation.z:.3f}')

        self.future = self._action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """目標送信の応答を処理するコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            self.send_next_goal() # 拒否された場合も次に進む
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        
        # 結果を待機
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """目標達成の結果を処理するコールバック"""
        result = future.result().result
        status = future.result().status

        # ナビゲーション完了フラグを更新
        self.is_navigating = False

        # 現在の目標地点をログ出力
        prev_index = (self.current_target_index - 1) % len(self.target_poses)
        current_target = self.target_poses[prev_index]
        self.get_logger().info(f'Goal result for target {prev_index}: ({current_target[0]:.2f}, {current_target[1]:.2f}, {current_target[2]:.2f})')

        if status == 4: # GoalStatus.STATUS_SUCCEEDED (ROS 2 Humble)
            self.get_logger().info('Goal succeeded! 🎉')
            self.get_logger().info('Staying at goal position for 5 seconds...')
            # 成功時は5秒間その場で待機
            self.next_goal_timer = self.create_timer(5.0, self.send_next_goal_delayed)
        elif status == 6: # GoalStatus.STATUS_ABORTED
            self.get_logger().warn(f'Goal was aborted (Status: {status}). This target may be unreachable.')
            self.get_logger().warn('Skipping to next target in 5 seconds...')
            # 中止された場合は長めに待機
            self.next_goal_timer = self.create_timer(5.0, self.send_next_goal_delayed)
        else:
            self.get_logger().warn(f'Goal failed with status: {status}. Retrying in 3 seconds...')
            # その他の失敗は短い待機
            self.next_goal_timer = self.create_timer(3.0, self.send_next_goal_delayed)

    def send_next_goal_delayed(self):
        """遅延実行用の次のゴール送信"""
        if self.next_goal_timer is not None:
            self.next_goal_timer.cancel()
            self.next_goal_timer = None
        self.send_next_goal()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """オイラー角 (ロール、ピッチ、ヨー) をクォータニオンに変換する"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        return q


def main(args=None):
    rclpy.init(args=args)
    navigator = ContinuousNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()