import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info('Waiting for /spawn_entity service...')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.spawn_model_template = self._get_obstacle_sdf()
        self.spawn_obstacle_loop()

    def _get_obstacle_sdf(self):
        # 0.2m x 0.2m x 0.5m の立方体モデルのSDF定義
        sdf = """<?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="custom_obstacle">
            <pose>0 0 0.25 0 0 0</pose> 
            <link name="link">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.05</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
                  <iyy>0.05</iyy><iyz>0.0</iyz>
                  <izz>0.05</izz>
                </inertia>
              </inertial>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.2 0.2 0.5</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.2 0.2 0.5</size>
                  </box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>"""
        return sdf

    def spawn_obstacle(self, x, y, name="obstacle"):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.spawn_model_template
        
        # モデルの初期位置を設定 (Z=0.25 は地面から浮かないようにするため)
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = 0.25 
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Successfully spawned obstacle "{name}" at ({x}, {y})')
        else:
            self.get_logger().error(f'Failed to spawn obstacle: {future.result().status_message}')

    def spawn_obstacle_loop(self):
        self.get_logger().info('--- Obstacle Spawner Initialized ---')
        while rclpy.ok():
            try:
                # ユーザーからの入力を受け付け
                x_input = input("Enter X coordinate for obstacle (or 'q' to quit): ")
                if x_input.lower() == 'q':
                    break
                
                y_input = input("Enter Y coordinate for obstacle: ")

                x = float(x_input)
                y = float(y_input)
                
                # ユニークな名前を生成
                obstacle_name = f"custom_obs_{x}_{y}".replace('.', '_').replace('-', 'n')
                
                self.spawn_obstacle(x, y, obstacle_name)
                
            except ValueError:
                self.get_logger().error("Invalid input. Please enter numbers.")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    spawner = ObstacleSpawner()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()