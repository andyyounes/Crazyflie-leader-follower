import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_py import Crazyswarm
import numpy as np
import threading
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.msg import Position


SAFE_DISTANCE = 0.3  
OFFSET_VECTOR = np.array([-SAFE_DISTANCE, 0.0, 0.0])  
class LeaderNodeJoystick(Node):
    def __init__(self, swarm):
        super().__init__('leader_joy_node')
        self.get_logger().info("Here")

        self.subscription = self.create_subscription(
            LogDataGeneric,
            '/cf231/kalman_var',
            self.wait_for_position_estimator,
            10)
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )
        # self.cmd_vel_sub = self.create_subscription(
        #     Twist,
        #     '/cmd_vel',
        #     self.cmd_vel_callback,
        # 10)

        self.get_logger().info("Here")
        self.cmd_vel_pub  = self.create_publisher(
            Position,
            '/cf231/cmd_position',
            10)
        
        self.right_button_x = 0
        self.right_button_y = 0

        
        self.cmd_vel = Twist()
        self.v = [0.0, 0.0, 0.0]
        self.yaw_rate = 0.0

        self.sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        if self.sim_time:
            self.publisher = self.create_publisher(
            PoseStamped,
            '/cf231/pose',
            10)

        self.joystick = False
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_leader = swarm.allcfs.crazyflies[0]
        self.z = 0.5

        self.var_y_history = [1000] * 10
        self.var_x_history = [1000] * 10
        self.var_z_history = [1000] * 10
        self.kalman_valid = False

        self.goal_pos = [0.0,0.1,0.0]
        self.should_finish = False
        if not self.sim_time:

            self.cf_leader.setParam('kalman.resetEstimation', 1)
            self.timeHelper.sleep(0.1)
            self.cf_leader.setParam('kalman.resetEstimation', 0)
            
            self.cf_leader.setParam('kalman.initialX', 0.0)
            self.cf_leader.setParam('kalman.initialY', 1.0)
            self.cf_leader.setParam('kalman.initialZ', 0.0)
        else:
            self.get_logger().info("sim time joystick")

        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

    def joystick_callback(self, msg):
        self.joy_axes = msg.axes
        self.right_button_x = -self.joy_axes[2] 
        self.right_button_y = self.joy_axes[3]

        self.goal_pos[0] += self.right_button_x * 0.01
        self.goal_pos[1] += self.right_button_y * 0.01
        if msg.buttons[0] == 1:
            self.get_logger().info("enabling joystick")

            self.joystick = True


    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z

        yaw_rate = msg.angular.z
        self.v = [vx, vy, vz]
        self.yaw = yaw_rate

    def run_mission(self):
        # Simulation Mode (skip Kalman check)
        
        if self.sim_time:
            self.get_logger().info("Simulation mode detected. Skipping Kalman validation.")
            self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
            time.sleep(3.0)

            try:
                while rclpy.ok() and not self.should_finish:
                    x = self.joy_axes[0] * 0.1  # left stick horizontal (left-right)
                    y = self.joy_axes[1] * 0.1  # left stick vertical (forward-backward)
                    z_offset = self.joy_axes[4] * 0.05  # right stick vertical (up-down)

                    if abs(x) > 0.05 or abs(y) > 0.05 or abs(z_offset) > 0.02:
                        self.z += z_offset
                        self.z = np.clip(self.z, 0.1, 2.0)
                        target = np.array([x, y, 0.0])
                        self.get_logger().info(f"[SIM JOYSTICK] Moving by: {target}, Altitude: {self.z}")
                        self.cf_leader.goTo(target, yaw=0.0, duration=5.0, relative=True)
                    time.sleep(5.0)
            except KeyboardInterrupt:
                pass

            self.get_logger().info("Landing (sim)...")
            self.cf_leader.land(targetHeight=0.02, duration=2.0)
            self.should_finish = True

        # Real Flight Mode (requires Kalman filter convergence)
        else:
            while not self.should_finish:
                if self.kalman_valid:
                    self.get_logger().info("Kalman converged. Starting real flight.")
                    # self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
                    # time.sleep(5.0)

                    try:
                        while rclpy.ok():
                                if self.joystick:

                                    self.get_logger().info(f"Sending Pose {self.goal_pos[0]} {self.goal_pos[1]} {0.3}")

                                    # command = Position()
                                    # command.x= self.goal_pos[0]
                                    # command.y= self.goal_pos[1]
                                    # command.z= 0.3
                                    self.cf_leader.goTo(np.array([self.goal_pos[0],self.goal_pos[1],0.3]), yaw=0.0, duration=2.0,relative=False)


                                    # self.cmd_vel_pub.publish(command)
                                    # self.joystick = False
                                    # self.cf_leader.cmdVelocityWorld(self.v, self.yaw_rate)
                                time.sleep(0.5)
                    except KeyboardInterrupt:
                        pass

                    self.get_logger().info("Landing (real)...")
                    self.cf_leader.land(targetHeight=0.02, duration=2.0)
                    self.should_finish = True
                else:
                    self.get_logger().info("Waiting for Kalman filter to converge...")
                    time.sleep(1)
            '''
            while not self.should_finish:
                if self.kalman_valid:
                    self.get_logger().info("Kalman converged. Starting real flight.")
                    self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
                    time.sleep(5.0)

                    try:
                        while rclpy.ok() and not self.should_finish:
                            x = self.joy_axes[0] * 0.1
                            y = self.joy_axes[1] * 0.1
                            z_offset = self.joy_axes[4] * 0.05

                            if abs(x) > 0.05 or abs(y) > 0.05 or abs(z_offset) > 0.02:
                                self.z += z_offset
                                self.z = np.clip(self.z, 0.1, 2.0)
                                target = np.array([x, y, 0.0])
                                self.get_logger().info(f"[HW JOYSTICK] Moving by: {target}, Altitude: {self.z}")
                                self.cf_leader.goTo(target, yaw=0.0, duration=2.0, relative=True)

                            time.sleep(2.0)
                    except KeyboardInterrupt:
                        pass

                    self.get_logger().info("Landing (real)...")
                    self.cf_leader.land(targetHeight=0.02, duration=2.0)
                    self.should_finish = True
                else:
                    self.get_logger().info("Waiting for Kalman filter to converge...")
                    time.sleep(1)
'''
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

class LeaderNode(Node):
    def __init__(self, swarm):
        super().__init__('leader_node')
        self.subscription = self.create_subscription(
            LogDataGeneric,
            '/cf231/kalman_var',
            self.wait_for_position_estimator,
            10)

        self.sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        if self.sim_time:
            self.publisher = self.create_publisher(
            PoseStamped,
            '/cf231/pose',
            10)
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_leader = swarm.allcfs.crazyflies[0]
        self.z = 0.5

        self.var_y_history = [1000] * 10
        self.var_x_history = [1000] * 10
        self.var_z_history = [1000] * 10
        self.kalman_valid = False
        self.waypoints = [
            # np.array([1.0, 1.0, 0.5]),
            # np.array([2.0, 1.5, 0.5]),
            # np.array([2.0, 0.0, 0.5]),
            # np.array([1.0, -1.0, 0.5]),
            np.array([1.0, 0.0, 0.5])
        ]


        self.should_finish = False
        if not self.sim_time:
            self.cf_leader.setParam('kalman.resetEstimation', 1)
            self.timeHelper.sleep(0.1)
            self.cf_leader.setParam('kalman.resetEstimation', 0)
            
            self.cf_leader.setParam('kalman.initialX', 0.0)
            self.cf_leader.setParam('kalman.initialY', 1.0)
            self.cf_leader.setParam('kalman.initialZ', 0.0)
        else:
            self.get_logger().info("Simulation mode detected — skipping Kalman param reset.")

    

        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

 
        # self.run_mission()

    def run_mission(self):
        if self.sim_time:
            self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
            time.sleep(5.0)


            for i, wp in enumerate(self.waypoints):
                pose_msg = PoseStamped()
                self.get_logger().info(f'Leader going to waypoint {i + 1}: {wp}')

                self.cf_leader.goTo(wp, yaw=0.0, duration=2.0)
                time.sleep(10.0)
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "cf231"
                pose_msg.pose.position.x = float(wp[0])
                pose_msg.pose.position.y = float(wp[1])
                pose_msg.pose.position.z = float(wp[2])
                pose_msg.pose.orientation.w = 1.0  # neutral orientation
    
                self.publisher.publish(pose_msg)

            
            time.sleep(5.0)
            self.get_logger().info('Leader landing...')
            self.cf_leader.land(targetHeight=0.02, duration=2.0)
            
            self.get_logger().info('Leader mission complete.')
            self.should_finish = True
        else:
            while not self.should_finish:
                if self.kalman_valid:

                    self.cf_leader.takeoff(targetHeight=self.z, duration=5.0)
                    time.sleep(5.0)


                    for i, wp in enumerate(self.waypoints):
                        self.get_logger().info(f'Leader going to waypoint {i + 1}: {wp}')

                        self.cf_leader.goTo(wp, yaw=0.0, duration=2.0)
                        time.sleep(2.0)
                        self.publisher.publish(pose_msg)

                    
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
            # self.destroy_subscription(self.subscription)


        
class FollowerNode(Node):
    def __init__(self, swarm):
        super().__init__('follower_node')
  
        self.swarm = swarm

        self.subscription = self.create_subscription(
        LogDataGeneric,
        '/cf5/kalman_var',
        self.wait_for_position_estimator,
        10)
        self.sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        self.leader_pos_subscription = self.create_subscription(
        PoseStamped,
        '/cf231/pose',
        self.get_leader_pose,
        10)

        self.timeHelper = swarm.timeHelper
        self.cf_follower = swarm.allcfs.crazyflies[1]
        self.z = 0.5
        self.kalman_valid = False
        self.waypoints = [
            # np.array([1.0, 1.0, 0.5]),
            # np.array([2.0, 1.5, 0.5]),
            # np.array([2.0, 0.0, 0.5]),
            np.array([1.0, -1.0, 0.5]),
            np.array([1.0, 0.0, 0.5])
        ]

        self.var_y_history = [1000] * 10
        self.var_x_history = [1000] * 10
        self.var_z_history = [1000] * 10

        self.should_finish = False
        if not self.sim_time:
            self.cf_follower.setParam('kalman.resetEstimation', 1)
            self.timeHelper.sleep(0.1)
            self.cf_follower.setParam('kalman.resetEstimation', 0)
            
            self.cf_follower.setParam('kalman.initialX', 0.0)
            self.cf_follower.setParam('kalman.initialY', -1.0)
            self.cf_follower.setParam('kalman.initialZ', 0.0)
        else:
            self.get_logger().info("sim time")
            self.kalman_valid = True

        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()
        self.leader_pose = None

    def get_leader_pose(self,msg):
        if self.sim_time:
            position = msg.pose.position
            leader_pose = np.array([position.x, position.y - 0.3, position.z])
            self.get_logger().info(f'[FOLLOWER] Going to: {leader_pose}')
            self.cf_follower.goTo(leader_pose, yaw=0.0, duration=2.0, relative=False)
            self.should_finish = True
        else:
            self.leader_pose =  msg.pose.position
    def run_mission(self):
        if self.sim_time:
            self.cf_follower.takeoff(targetHeight=self.z, duration=5.0)
            time.sleep(1.0)

            time.sleep(30.0) 

            self.cf_follower.land(targetHeight=0.02, duration=2.0)
            self.should_finish = True
        else:
            while not self.should_finish:
                if self.kalman_valid:

                    self.cf_follower.takeoff(targetHeight=self.z, duration=5.0)
                    time.sleep(7.0)


                    # for i, wp in enumerate(self.waypoints):
                    # self.get_logger().info(f'Leader going to waypoint {i + 1}: {wp}')
                    if self.leader_pose is not None:
                        self.get_logger().info(f'Leader pose x: {self.leader_pose.x} y: {self.leader_pose.y} z: {self.leader_pose.z}')
                        leader_pose = np.array([self.leader_pose.x,self.leader_pose.y - 0.2, self.leader_pose.z])
                        self.cf_follower.goTo(leader_pose, yaw=0.0, duration=2.0,relative=False)

                    time.sleep(5.0)

                    # self.cf_follower.land(targetHeight=0.02, duration=2.0)
                    
                    # self.should_finish = True
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
    leader_joy = LeaderNodeJoystick(swarm)
    # leader = LeaderNode(swarm)
    follower = FollowerNode(swarm)

    executor = SingleThreadedExecutor()
    executor.add_node(leader_joy)
    # executor.add_node(leader)
    executor.add_node(follower)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

