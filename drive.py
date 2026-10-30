"""ROS 2 node: drive a simulated differential-drive robot toward a goal."""
import math
import sys
from pathlib import Path
import joblib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GoalDriver(Node):
    def __init__(self, gx, gy):
        super().__init__('ai_goal_driver')
        self.gx, self.gy = gx, gy
        self.model = joblib.load(Path(__file__).resolve().parent / 'direction_model.joblib')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.last_odom = None
        self.done = False
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'AI goal driver started. Goal=({gx}, {gy})')

    def on_odom(self, msg):
        self.last_odom = msg

    def tick(self):
        if self.done or self.last_odom is None:
            return
        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = self.gx - p.x, self.gy - p.y
        distance = math.hypot(dx, dy)
        cmd = Twist()
        if distance < 0.20:
            self.done = True
            self.publisher.publish(cmd)
            self.get_logger().info(f'GOAL REACHED: x={p.x:.2f}, y={p.y:.2f}, error={distance:.2f} m')
            return
        desired = math.atan2(dy, dx)
        error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))
        action = self.model.predict([[error]])[0]
        if action == 'LEFT':
            cmd.angular.z = 0.5
        elif action == 'RIGHT':
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = min(0.25, max(0.08, distance * 0.4))
        self.publisher.publish(cmd)
        self.get_logger().info(f'position=({p.x:.2f},{p.y:.2f}) dist={distance:.2f} angle={error:.2f} AI={action}', throttle_duration_sec=1.0)

    def stop(self):
        self.publisher.publish(Twist())

def main():
    if len(sys.argv) != 3:
        print('Usage: python3 drive.py GOAL_X GOAL_Y')
        sys.exit(2)
    rclpy.init()
    node = GoalDriver(float(sys.argv[1]), float(sys.argv[2]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
