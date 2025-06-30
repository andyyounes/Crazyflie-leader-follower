import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_py import Crazyswarm
import numpy as np
import threading
import time




SAFE_DISTANCE = 0.3  
OFFSET_VECTOR = np.array([-SAFE_DISTANCE, 0.0, 0.0])  

class LeaderNode(Node):
    def __init__(self, swarm):
        super().__init__('leader_node')
        self.subscription = self.create_subscription(
            LogDataGeneric,
            '/cf231/kalman_var',
            self.wait_for_position_estimator,
            10)
        
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_leader = swarm.allcfs.crazyflies[0]
        self.z = 0.3

        self.var_y_history = [1000] * 10
        self.var_x_history = [1000] * 10
        self.var_z_history = [1000] * 10
        self.kalman_valid = False
        self.waypoints = [
            # np.array([1.0, 1.0, 0.5]),
            # np.array([2.0, 1.5, 0.5]),
            # np.array([2.0, 0.0, 0.5]),
            # np.array([1.0, -1.0, 0.5]),
            np.array([0.3, 0.0, 0.3])
            # np.array([0.5, 0.0, 0.3])

        ]

        self.should_finish = False
        
        self.cf_leader.setParam('kalman.resetEstimation', 1)
        self.timeHelper.sleep(0.1)
        self.cf_leader.setParam('kalman.resetEstimation', 0)
        
        self.cf_leader.setParam('kalman.initialX', 0.0)
        self.cf_leader.setParam('kalman.initialY', 1.0)
        self.cf_leader.setParam('kalman.initialZ', 0.0)

        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

 
        # self.run_mission()

    def run_mission(self):

        while not self.should_finish:
            if self.kalman_valid:

                self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
                time.sleep(5.0)


                for i, wp in enumerate(self.waypoints):
                    self.get_logger().info(f'Leader going to waypoint {i + 1}: {wp}')
                    # self.cf_leader.cmdFullState(
                    #     pos=wp,
                    #     vel=[0.0, 0.0, 0.0],
                    #     acc=[0.0, 0.0, 0.0],
                    #     yaw=0.0,
                    #     omega=[0.0, 0.0, 0.0]
                    # )
                    self.cf_leader.goTo(wp, yaw=0.0, duration=2.0)
                    time.sleep(2.0)
                
                time.sleep(5.0)
                self.get_logger().info('Leader landing...')
                self.cf_leader.land(targetHeight=0.02, duration=2.0)
                
                self.get_logger().info('Leader mission complete.')
                self.should_finish = True
            else:
                time.sleep(1)
                self.get_logger().info('Kalman Not Valid')

    def wait_for_position_estimator(self, msg):

        
        self.get_logger().info('Waiting for estimator to find position...')

        threshold = 0.001

        var_x =  msg.values[0]
        var_y = msg.values[1]
        var_z = msg.values[2]
        self.get_logger().info(f"VarX: {var_x} VarY: {var_y} VarZ: {var_z} ")

        self.var_x_history.append(var_x)
        self.var_x_history.pop(0)
        self.var_y_history.append(var_y)
        self.var_y_history.pop(0)
        self.var_z_history.append(var_z)
        self.var_z_history.pop(0)

        self.get_logger().info(f"X History {self.var_x_history}")
        self.get_logger().info(f"y History {self.var_y_history}")
        self.get_logger().info(f"Z History {self.var_z_history}")

        min_x = min(self.var_x_history)
        max_x = max(self.var_x_history)
        min_y = min(self.var_y_history)
        max_y = max(self.var_y_history)
        min_z = min(self.var_z_history)
        max_z = max(self.var_z_history)

        self.get_logger().info(f"MinX: {min_x} MinY: {min_y} MinZ: {min_z}")
        self.get_logger().info(f"MaxX: {max_x} MaxY: {max_y} MaxZ: {max_z}")
    
        self.get_logger().info("{} {} {}".format(max_x - min_x, max_y - min_y, max_z - min_z))

        if (max_x - min_x) < threshold and (
                max_y - min_y) < threshold and (
                max_z - min_z) < threshold:
            self.kalman_valid = True
            self.destroy_subscription(self.subscription)


        
class FollowerNode(Node):
    def __init__(self, swarm):
        super().__init__('follower_node')
  
        self.swarm = swarm

        self.subscription = self.create_subscription(
        LogDataGeneric,
        '/cf5/kalman_var',
        self.wait_for_position_estimator,
        10)
    
        self.leader_pos_subscription = self.create_subscription(
        PoseStamped,
        '/cf231/pose',
        self.get_leader_pose,
        10)

        self.timeHelper = swarm.timeHelper
        self.cf_follower = swarm.allcfs.crazyflies[1]
        self.z = 0.3
        self.kalman_valid = False
        self.waypoints = [
            # np.array([1.0, 1.0, 0.5]),
            # np.array([2.0, 1.5, 0.5]),
            # np.array([2.0, 0.0, 0.5]),
            # np.array([1.0, -1.0, 0.5]),
            np.array([0.3, 0.0, 0.3])
        ]

        self.var_y_history = [1000] * 10
        self.var_x_history = [1000] * 10
        self.var_z_history = [1000] * 10

        self.should_finish = False

        self.cf_follower.setParam('kalman.resetEstimation', 1)
        self.timeHelper.sleep(0.1)
        self.cf_follower.setParam('kalman.resetEstimation', 0)
        
        self.cf_follower.setParam('kalman.initialX', 0.0)
        self.cf_follower.setParam('kalman.initialY', -1.0)
        self.cf_follower.setParam('kalman.initialZ', 0.0)


        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()
        self.leader_pose = None

    def get_leader_pose(self,msg):


        self.leader_pose = msg

    def run_mission(self):

        while not self.should_finish:
            if self.kalman_valid:

                self.cf_follower.takeoff(targetHeight=self.z, duration=5.0)
                time.sleep(10.0)


               
                if self.leader_pose is not None:
                    self.get_logger().info(f'Leader pose x: {self.leader_pose.pose.position.x} y: {self.leader_pose.pose.position.y} z: {self.leader_pose.pose.position.z}')
                    leader_pose = np.array([self.leader_pose.pose.position.x,self.leader_pose.pose.position.y - 0.2, self.leader_pose.pose.position.z])
                    self.cf_follower.goTo(leader_pose, yaw=0.0, duration=2.0,relative=False)
                    # self.cf_follower.cmdFullState(leader_pose, vel=[0.0, 0.0, 0.0], acc=[0.0, 0.0, 0.0], yaw=0.0,omega=[0.0, 0.0, 0.0])
                time.sleep(5.0)
                # time.sleep(2.0)
            
                # time.sleep(5.0)
                # self.get_logger().info('Leader landing...')
                self.cf_follower.land(targetHeight=0.02, duration=2.0)
                
                # self.get_logger().info('Leader mission complete.')
                self.should_finish = True
            else:
                time.sleep(1)
                self.get_logger().info('Kalman Not Valid')

    def wait_for_position_estimator(self, msg):

        
        self.get_logger().info('Waiting for estimator to find position...')

        threshold = 0.001

        var_x =  msg.values[0]
        var_y = msg.values[1]
        var_z = msg.values[2]
        self.get_logger().info(f"VarX: {var_x} VarY: {var_y} VarZ: {var_z} ")

        self.var_x_history.append(var_x)
        self.var_x_history.pop(0)
        self.var_y_history.append(var_y)
        self.var_y_history.pop(0)
        self.var_z_history.append(var_z)
        self.var_z_history.pop(0)

        self.get_logger().info(f"X History {self.var_x_history}")
        self.get_logger().info(f"y History {self.var_y_history}")
        self.get_logger().info(f"Z History {self.var_z_history}")

        min_x = min(self.var_x_history)
        max_x = max(self.var_x_history)
        min_y = min(self.var_y_history)
        max_y = max(self.var_y_history)
        min_z = min(self.var_z_history)
        max_z = max(self.var_z_history)

        self.get_logger().info(f"MinX: {min_x} MinY: {min_y} MinZ: {min_z}")
        self.get_logger().info(f"MaxX: {max_x} MaxY: {max_y} MaxZ: {max_z}")
    
        self.get_logger().info("{} {} {}".format(max_x - min_x, max_y - min_y, max_z - min_z))

        if (max_x - min_x) < threshold and (
                max_y - min_y) < threshold and (
                max_z - min_z) < threshold:
            self.kalman_valid = True
            self.destroy_subscription(self.subscription)

def main():
    swarm = Crazyswarm()
    leader = LeaderNode(swarm)
    follower = FollowerNode(swarm)

    executor = SingleThreadedExecutor()
    executor.add_node(leader)
    executor.add_node(follower)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
  
