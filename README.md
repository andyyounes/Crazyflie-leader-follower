# crazyflies-leader-follower
Simple leader go to waypoints - and follower subscirbes to it's position

```bash
source install/setup.bash
ros2 launch crazyflie launch.py rviz:=True backend:=sim
ros2 run crazyflie_leader_follower run --ros-args -p use_sim_time:=True
```
