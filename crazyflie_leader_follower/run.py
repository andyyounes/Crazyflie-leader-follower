import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import numpy as np
import threading
import time

SAFE_DISTANCE = 0.3  
OFFSET_VECTOR = np.array([-SAFE_DISTANCE, 0.0, 0.0])  

class LeaderNode(Node):
    def __init__(self, swarm):
        super().__init__('leader_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/cf5/leader_pose', 10)
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_leader = swarm.allcfs.crazyflies[1]
        self.z = 0.5
        self.waypoints = [
            np.array([1.0, 1.0, 0.5]),
            np.array([2.0, 1.5, 0.5]),
            np.array([2.0, 0.0, 0.5]),
            np.array([1.0, -1.0, 0.5]),
            np.array([0.0, 0.0, 0.5])
        ]
        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

    def run_mission(self):
        
        self.cf_leader.takeoff(targetHeight=self.z, duration=2.0)
        time.sleep(10.0)

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'Leader going to waypoint {i + 1}: {wp}')

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(wp[0])
            msg.pose.position.y = float(wp[1])
            msg.pose.position.z = float(wp[2])
            msg.pose.orientation.w = 1.0
            self.publisher_.publish(msg)

            time.sleep(0.5)  

            self.cf_leader.goTo(wp, yaw=0.0, duration=2.0)
            time.sleep(10.0)
        
        time.sleep(10.0)
        self.get_logger().info('Leader landing...')
        self.cf_leader.land(targetHeight=0.02, duration=2.0)
        
        self.get_logger().info('Leader mission complete.')

        msg = PoseStamped()
        msg.pose.position.z = -1.0  # Convention: z < 0 means "land"
        self.publisher_.publish(msg)


class FollowerNode(Node):
    def __init__(self, swarm):
        super().__init__('follower_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/cf5/leader_pose',
            self.listener_callback,
            10)

        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_follower = swarm.allcfs.crazyflies[0]

        self.leader_pos = None
        self.follower_pos = None
        self.running = True

        # Initial takeoff
        self.cf_follower.takeoff(targetHeight=0.5, duration=2.0)
        time.sleep(5.0)
        self.follower_pos = np.array([0.0, 0.0, 0.5])  


    def listener_callback(self, msg):
        if msg.pose.position.z < 0.0:
            self.get_logger().info('Received landing signal. Landing...')
            time.sleep(5.0)
            self.shutdown()
            self.running = False
            return

        leader_pos = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z])

        self.get_logger().info(f'Follower received leader pos: {leader_pos}')
        time.sleep(2.0)
        target = leader_pos + OFFSET_VECTOR
        dist = np.linalg.norm(target - self.follower_pos)

        if dist >= SAFE_DISTANCE:
            self.get_logger().info(f'Moving follower to {target}, dist = {dist:.2f}')
            self.cf_follower.goTo(target, yaw=0.0, duration=2.0)
            self.follower_pos = target
    def shutdown(self):
        self.cf_follower.land(targetHeight=0.02, duration=2.0)
        time.sleep(3.0)
        self.get_logger().info('Follower landed.')
    


def main():
    swarm = Crazyswarm()
    leader = LeaderNode(swarm)
    follower = FollowerNode(swarm)

    executor = MultiThreadedExecutor()
    executor.add_node(leader)
    executor.add_node(follower)
    executor.spin()

    leader.destroy_node()
    follower.shutdown()
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
