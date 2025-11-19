import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseSubscriber(Node):
    """ロボットの現在の姿勢 (位置と向き) を購読し、表示するノード"""
    def __init__(self):
        super().__init__('pose_subscriber')
        # /amcl_pose トピックを購読
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10)
        self.get_logger().info('Pose Subscriber Node has started. Subscribing to /amcl_pose.')

    def pose_callback(self, msg):
        """購読した姿勢データを処理するコールバック関数"""
        # 位置 (Position)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 向き (Orientation - Quaternion)
        q = msg.pose.pose.orientation
        
        # クォータニオンをオイラー角(ヨー角)に変換するのは複雑なので、
        # ここでは分かりやすく位置情報のみを表示します
        self.get_logger().info(
            f'Current Pose: X={x:.2f}, Y={y:.2f}, Qz={q.z:.2f}, Qw={q.w:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()