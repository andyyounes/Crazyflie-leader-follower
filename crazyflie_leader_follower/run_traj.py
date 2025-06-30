import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import numpy as np
import threading
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


SAFE_DISTANCE = 0.3  
OFFSET_VECTOR = np.array([-SAFE_DISTANCE, -SAFE_DISTANCE, 0.0])  
class Trajectory:
    def __init__(self):
        pass

    @staticmethod
    def circle_trajectory(center, radius, height, num_points=50, revolutions=2):
        """Generate circular trajectory"""
        points = []
        for i in range(num_points * revolutions):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = height
            points.append(np.array([x, y, z]))
        return points
    @staticmethod
    def visualize_trajectory(trajectory, title="Drone Trajectory", show_3d=True, save_plot=False, filename="trajectory.png"):
        """Visualize trajectory using matplotlib"""
        if not trajectory:
            print("No trajectory to visualize")
            return
        
        # Extract coordinates
        x_coords = [point[0] for point in trajectory]
        y_coords = [point[1] for point in trajectory]
        z_coords = [point[2] for point in trajectory]
        
        if show_3d:
            # 3D plot
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Plot trajectory
            ax.plot(x_coords, y_coords, z_coords, 'b-', linewidth=2, label='Leader Trajectory')
            ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='green', s=100, label='Start')
            ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='red', s=100, label='End')
            
            # Plot follower trajectory (offset)
            follower_x = [x + OFFSET_VECTOR[0] for x in x_coords]
            follower_y = [y + OFFSET_VECTOR[1] for y in y_coords]
            follower_z = [z + OFFSET_VECTOR[2] for z in z_coords]
            ax.plot(follower_x, follower_y, follower_z, 'r--', linewidth=2, alpha=0.7, label='Follower Trajectory')
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'{title} - 3D View')
            ax.legend()
            ax.grid(True)
            
        else:
            # 2D plots
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
            
            # XY plot
            ax1.plot(x_coords, y_coords, 'b-', linewidth=2, label='Leader')
            ax1.scatter(x_coords[0], y_coords[0], color='green', s=100, label='Start')
            ax1.scatter(x_coords[-1], y_coords[-1], color='red', s=100, label='End')
            
            # Follower trajectory
            follower_x = [x + OFFSET_VECTOR[0] for x in x_coords]
            follower_y = [y + OFFSET_VECTOR[1] for y in y_coords]
            ax1.plot(follower_x, follower_y, 'r--', linewidth=2, alpha=0.7, label='Follower')
            
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('XY View')
            ax1.legend()
            ax1.grid(True)
            ax1.axis('equal')
            
            # XZ plot
            ax2.plot(x_coords, z_coords, 'b-', linewidth=2)
            ax2.scatter(x_coords[0], z_coords[0], color='green', s=100)
            ax2.scatter(x_coords[-1], z_coords[-1], color='red', s=100)
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Z (m)')
            ax2.set_title('XZ View (Side)')
            ax2.grid(True)
            
            # YZ plot
            ax3.plot(y_coords, z_coords, 'b-', linewidth=2)
            ax3.scatter(y_coords[0], z_coords[0], color='green', s=100)
            ax3.scatter(y_coords[-1], z_coords[-1], color='red', s=100)
            ax3.set_xlabel('Y (m)')
            ax3.set_ylabel('Z (m)')
            ax3.set_title('YZ View (Side)')
            ax3.grid(True)
            
            # Time series plot
            time_points = np.linspace(0, len(trajectory)-1, len(trajectory))
            ax4.plot(time_points, x_coords, 'r-', label='X', linewidth=2)
            ax4.plot(time_points, y_coords, 'g-', label='Y', linewidth=2)
            ax4.plot(time_points, z_coords, 'b-', label='Z', linewidth=2)
            ax4.set_xlabel('Trajectory Point')
            ax4.set_ylabel('Position (m)')
            ax4.set_title('Position vs Time')
            ax4.legend()
            ax4.grid(True)
            
            plt.tight_layout()
        
        
        plt.show()
        
        print(f"\n=== Trajectory Statistics ===")
        print(f"Total points: {len(trajectory)}")
        print(f"X range: {min(x_coords):.2f} to {max(x_coords):.2f} m")
        print(f"Y range: {min(y_coords):.2f} to {max(y_coords):.2f} m")
        print(f"Z range: {min(z_coords):.2f} to {max(z_coords):.2f} m")
        
        total_distance = 0
        for i in range(1, len(trajectory)):
            dist = np.linalg.norm(trajectory[i] - trajectory[i-1])
            total_distance += dist
        print(f"Total trajectory length: {total_distance:.2f} m")
        
        # Calculate average speed (assuming 2 seconds per point)
        estimated_time = len(trajectory) * 2.0  # 2 seconds per point
        avg_speed = total_distance / estimated_time
        print(f"Estimated flight time: {estimated_time:.1f} seconds")
        print(f"Average speed: {avg_speed:.2f} m/s")
     
class LeaderNode(Node):
    def __init__(self, swarm):
        super().__init__('leader_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/cf231/leader_pose', 10)
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_leader = swarm.allcfs.crazyflies[0]
        self.z = 0.3
        self.trajectory_generator = Trajectory()
        self.trajectory = self.trajectory_generator.circle_trajectory(
            center=[0.0, 0.0], radius=0.8, height=0.5, num_points=30, revolutions=2
        )
        self.trajectory_generator.visualize_trajectory(
            self.trajectory, 
            title="Circle", 
            show_3d=True, 
            save_plot=True
            # filename="drone_trajectory.png"
        )
        # self.waypoints = [
        #     # np.array([0.3, 0.3, 0.3]),
        #     np.array([0.5, 0.5, 0.3]),
        #     np.array([-0.5, 0.5, 0.3]),

        #     np.array([0.5, 0.5 , 0.3]),
        #     np.array([-1.0, 0.5 , 0.3]),


        #     # np.array([2.0, 0.0, 0.5]),
        #     # np.array([1.0, -1.0, 0.5]),
        #     # np.array([0.0, 0.0, 0.5])
        # ]
        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

    def run_mission(self):
        
        self.cf_leader.takeoff(targetHeight=self.z, duration=2.0)
        time.sleep(10.0)
        # with waypoints
        # for i, wp in enumerate(self.waypoints):
        for i, wp in enumerate(self.trajectory):
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
            time.sleep(2.0)
        
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
            '/cf231/leader_pose',
            self.listener_callback,
            10)

        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.cf_follower = swarm.allcfs.crazyflies[1]

        self.leader_pos = None
        # self.follower_pos = None
        self.running = True

        # Initial takeoff
        self.cf_follower.takeoff(targetHeight=0.3, duration=2.0)
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

    executor = SingleThreadedExecutor()
    executor.add_node(leader)
    executor.add_node(follower)
    executor.spin()

    leader.destroy_node()
    follower.shutdown()
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
